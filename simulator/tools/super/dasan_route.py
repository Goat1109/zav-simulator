#!/usr/bin/env python3
"""Continuous interior bag section and return; no exterior component search.

Offsets from measured poses are bounded (35cm by default). Optional local gap
repairs stay in the recorded corridor column and check every edge plus observed
floor/roof witnesses. Distant components and outdoor shortcuts are never joined.
The map is read only offline; runtime SUPER uses its simulated LiDAR.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import heapq
import itertools
import numpy as np
from scipy.spatial import cKDTree
from pcd_scene import read_pcd


def sampled_segment(a,b,spacing=.06):
    return np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/spacing))+1))


def connect_near_bag(start,end,reference,distance,clearance,lateral_limit,vertical_limit,audit=None):
    """Local search confined to the recorded corridor column.

    Every edge is sampled, and every grid point has a wall-free horizontal
    witness to bag XY at its flight height. This cannot join distant floors or
    use an unconstrained outdoor component as a shortcut.
    """
    resolution=.10
    if audit is None:audit={}
    audit.update(start=np.asarray(start).tolist(),end=np.asarray(end).tolist())
    lower=np.floor((np.minimum(start,end)-[lateral_limit+.2,lateral_limit+.2,vertical_limit+.2])/resolution)*resolution
    upper=np.maximum(start,end)+[lateral_limit+.2,lateral_limit+.2,vertical_limit+.2]
    shape=np.ceil((upper-lower)/resolution).astype(int)+1
    if int(np.prod(shape))>700000:audit['result']='grid_too_large';return None
    ijk=np.indices(shape).reshape(3,-1).T;points=lower+ijk*resolution
    nearest=cKDTree(reference[:,:2]).query(points[:,:2])[1];roots=reference[nearest].copy();roots[:,2]=points[:,2]
    allowed=(np.linalg.norm(points[:,:2]-roots[:,:2],axis=1)<=lateral_limit+.10)&(np.abs(points[:,2]-reference[nearest,2])<=vertical_limit)
    allowed&=distance(points)>=clearance+.005
    ids=np.flatnonzero(allowed)
    witness=roots[ids,None,:]+np.linspace(0,1,9)[None,:,None]*(points[ids,None,:]-roots[ids,None,:])
    allowed[ids]&=(distance(witness.reshape(-1,3)).reshape(-1,9)>=.10).all(axis=1)
    audit['free_nodes']=int(allowed.sum())
    def key(index):return int(np.ravel_multi_index(tuple(index),tuple(shape)))
    def endpoint(point):
        ids=np.flatnonzero(allowed)
        for index in ids[np.argsort(np.linalg.norm(points[ids]-point,axis=1))[:20]]:
            if np.linalg.norm(points[index]-point)>.4:break
            if (distance(sampled_segment(point,points[index]))>=clearance).all():return int(index)
        return None
    first,last=endpoint(start),endpoint(end)
    if first is None or last is None:audit['result']='endpoint_blocked';return None
    queue=[(float(np.linalg.norm(points[first]-end)),0.,first)];cost={first:0.};parent={}
    steps=np.array([(a,b,c)for a,b,c in itertools.product((-1,0,1),repeat=3)if a or b or c])
    while queue:
        _,g,current=heapq.heappop(queue)
        if g>cost.get(current,float('inf'))+1e-9:continue
        if current==last:
            route=[current]
            while route[-1]!=first:route.append(parent[route[-1]])
            result=np.r_[[start],points[route[::-1]],[end]]
            # A roof and floor are required along each connector, not merely
            # clearance in an unobserved space outside the building.
            for p in result[::max(1,int(.4/resolution))]:
                for sign in (-1,1):
                    ray=p+np.arange(.25,3.,.08)[:,None]*np.array([0.,0.,sign])
                    if not np.any(distance(ray)<.02):
                        audit.update(result='missing_roof_or_floor',point=p.tolist(),direction=sign);return None
            audit['result']='connected'
            return result
        neighbours=ijk[current]+steps
        neighbours=neighbours[((neighbours>=0)&(neighbours<shape)).all(axis=1)]
        for index in neighbours:
            other=key(index)
            next_cost=g+float(np.linalg.norm(points[current]-points[other]))
            if not allowed[other] or next_cost>=cost.get(other,float('inf'))-1e-9:continue
            if not (distance(sampled_segment(points[current],points[other]))>=clearance).all():continue
            cost[other]=next_cost;parent[other]=current
            heapq.heappush(queue,(next_cost+float(np.linalg.norm(points[other]-end)),next_cost,other))
    from scipy.ndimage import label
    labels,_=label(allowed.reshape(shape),structure=np.ones((3,3,3),dtype=bool));labels=labels.ravel()
    audit.update(result='no_path',reachable_nodes=len(cost),start_component=int(labels[first]),end_component=int(labels[last]))
    return None


def build(assets,output,clearance=.55,start_seconds=18.,minimum_length=15.,
          lateral_limit=.35,vertical_limit=.35,end_seconds=None,anchor=None,connect_gaps=False,reverse_section=False,
          offset_weight=.2,smoothness_weight=0.,max_forward_length=None,scenario="dasan",yaw_policy='travel',clip_start=False):
    began=time.monotonic();pcd=assets/f'{scenario}.pcd';geometry=read_pcd(pcd)
    centers=np.unique(np.floor(geometry.astype(float)/.1).astype(np.int32),axis=0)*.1+.05
    tree=cKDTree(centers);half_diagonal=float(np.sqrt(3)*.05)
    def distance(p):return tree.query(np.asarray(p),workers=4 if len(np.asarray(p))>10000 else 1)[0]-half_diagonal
    def visible(a,b,margin=clearance):return bool(np.all(distance(sampled_segment(a,b))>=margin))
    data=np.genfromtxt(assets/f'{scenario}_trajectory.csv',delimiter=',',names=True)
    bag=np.column_stack([data[k]for k in ('x','y','z')]);times=data['t']-data['t'][0]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(bag,axis=0),axis=1))]
    stations=np.unique(np.searchsorted(arc,np.arange(0,arc[-1],.35)))
    if clip_start:stations=stations[times[stations]>=start_seconds]
    offsets=np.array([(a,b)for a in np.arange(-lateral_limit,lateral_limit+.001,.05)
                      for b in np.arange(-vertical_limit,vertical_limit+.001,.05)])
    runs=[];points=[];source=[];last_normal=np.array([1.,0.,0.])
    def finish():
        if len(points)>1:
            p=np.array(points);length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
            runs.append(dict(path=p,indices=np.array(source),length=length,
                first_seconds=float(times[source[0]]),last_seconds=float(times[source[-1]])))
        points.clear();source.clear()
    for number,index in enumerate(stations):
        if end_seconds is not None and times[index]>end_seconds:break
        centre=bag[index]
        tangent=bag[min(index+6,len(bag)-1)]-bag[max(0,index-6)]
        normal=np.array([-tangent[1],tangent[0],0.])
        if np.linalg.norm(normal)>.05:normal/=np.linalg.norm(normal);last_normal=normal
        else:normal=last_normal
        candidates=centre+offsets[:,0,None]*normal+offsets[:,1,None]*np.array([0.,0.,1.])
        d=distance(candidates);valid=np.flatnonzero(d>=clearance+.015)
        valid=sorted(valid,key=lambda i:d[i]-offset_weight*np.linalg.norm(offsets[i])-
            (smoothness_weight*np.linalg.norm(candidates[i]-points[-1])if points else 0.),reverse=True)
        chosen=None
        for candidate in valid[:20]:
            p=candidates[candidate]
            witness=np.r_[centre[:2],p[2]] if vertical_limit>.35 else centre
            if not visible(witness,p,margin=.10):continue
            if points and (np.linalg.norm(p-points[-1])>1.1 or not visible(points[-1],p)):continue
            chosen=p;break
        if chosen is None:
            finish()
            for candidate in valid[:20]:
                p=candidates[candidate]
                witness=np.r_[centre[:2],p[2]] if vertical_limit>.35 else centre
                if visible(witness,p,margin=.10):chosen=p;break
        if chosen is not None:points.append(chosen);source.append(int(index))
        if number%500==0:print(f'[Interior route] {number}/{len(stations)}, completed sections={len(runs)}',flush=True)
    finish()
    bridges=[];attempts=[]
    if connect_gaps:
        merged=[]
        for run in (r for r in runs if r['length']>=2.):
            if merged and run['first_seconds']-merged[-1]['last_seconds']<10.:
                previous=merged[-1];a=previous['indices'][-1];b=run['indices'][0]
                ia,ib=a,b
                audit=dict(start_seconds=previous['last_seconds'],end_seconds=run['first_seconds'])
                first_ref=max(0,int(np.searchsorted(times,previous['last_seconds']-2.)))
                last_ref=min(len(bag),int(np.searchsorted(times,run['first_seconds']+2.))+1)
                connector=connect_near_bag(previous['path'][-1],run['path'][0],bag[first_ref:last_ref],distance,clearance,lateral_limit,vertical_limit,audit)
                attempts.append(audit)
                trim_previous=trim_next=0
                if connector is None:
                    for trim_previous,trim_next in ((3,0),(0,3),(3,3),(6,6),(10,10)):
                        if len(previous['path'])<=trim_previous+2 or len(run['path'])<=trim_next+2:continue
                        ia=previous['indices'][-1-trim_previous];ib=run['indices'][trim_next]
                        first_ref=max(0,int(np.searchsorted(times,times[ia]-2.)))
                        last_ref=min(len(bag),int(np.searchsorted(times,times[ib]+2.))+1)
                        audit=dict(start_seconds=float(times[ia]),end_seconds=float(times[ib]),trim_nodes=[trim_previous,trim_next])
                        connector=connect_near_bag(previous['path'][-1-trim_previous],run['path'][trim_next],bag[first_ref:last_ref],distance,clearance,lateral_limit,vertical_limit,audit)
                        attempts.append(audit)
                        if connector is not None:break
                if connector is not None:
                    bridges.append(dict(start_seconds=float(times[ia]),end_seconds=float(times[ib]),
                        length_m=float(np.linalg.norm(np.diff(connector,axis=0),axis=1).sum())))
                    previous['path']=np.r_[previous['path'][:len(previous['path'])-trim_previous-1],connector,run['path'][trim_next+1:]]
                    previous['indices']=np.r_[previous['indices'][:len(previous['indices'])-trim_previous-1],np.rint(np.linspace(ia,ib,len(connector))).astype(int),run['indices'][trim_next+1:]]
                    previous['length']=float(np.linalg.norm(np.diff(previous['path'],axis=0),axis=1).sum())
                    previous['last_seconds']=run['last_seconds'];continue
            merged.append(run.copy())
        runs=merged
    if reverse_section:
        for run in runs:
            run['path']=run['path'][::-1].copy();run['indices']=run['indices'][::-1].copy()
    summary=[dict(section=i,length_m=r['length'],start_bag_seconds=r['first_seconds'],end_bag_seconds=r['last_seconds'])for i,r in enumerate(runs)]
    output.parent.mkdir(parents=True,exist_ok=True)
    sections_name='dasan_interior_sections.json'if output.stem=='dasan_mission'else output.stem+'-sections.json'
    output.with_name(sections_name).write_text(json.dumps(summary,indent=2)+'\n')
    eligible=[(i,r)for i,r in enumerate(runs)if r['length']>=minimum_length and r['last_seconds']>=start_seconds]
    if not eligible:raise ValueError('No sufficiently long continuous interior route with this clearance')
    selected=None
    anchored=json.loads(anchor.read_text())if anchor else None
    if anchored and anchored['map_sha256']!=hashlib.sha256(pcd.read_bytes()).hexdigest():
        raise ValueError('Anchor mission belongs to another map')
    for number,run in eligible:
        offsets_to_try=(np.flatnonzero(np.linalg.norm(run['path']-anchored['takeoff'],axis=1)<2.)
                        if anchored else range(min(30,len(run['path'])-2)))
        for offset in offsets_to_try:
            q=run['path'][offset]
            if anchored:
                takeoff=np.array(anchored['takeoff'])
                if np.linalg.norm(q-takeoff)>2. or not visible(takeoff,q):continue
                selected=(number,run,offset,anchored['landing_pad']['center'][2],anchored['interior_spawn_boundaries'])
                break
            tangent=run['path'][min(offset+3,len(run['path'])-1)]-q
            normal=np.array([-tangent[1],tangent[0],0.]);normal/=max(np.linalg.norm(normal),1e-9)
            def ray_hit(direction,maximum):
                distances=np.arange(.25,maximum,.04);hit=np.flatnonzero(distance(q+distances[:,None]*direction)<.02)
                return float(distances[hit[0]])if len(hit)else None
            floor_distance=ray_hit(np.array([0.,0.,-1.]),3.)
            ceiling_distance=ray_hit(np.array([0.,0.,1.]),2.5)
            left=ray_hit(normal,2.);right=ray_hit(-normal,2.)
            if any(v is None for v in (floor_distance,ceiling_distance,left,right)):continue
            local=geometry[np.linalg.norm(geometry[:,:2]-q[:2],axis=1)<.32]
            floor_height=q[2]-floor_distance
            floor=local[(local[:,2]>floor_height-.20)&(local[:,2]<floor_height+.20)]
            if len(floor)<10:continue
            pad_z=float(np.ceil(floor[:,2].max()/.1)*.1+.02)
            if not .8<q[2]-pad_z<2.8:continue
            if not visible(np.r_[q[:2],pad_z+.60],q,margin=.50):continue
            selected=(number,run,offset,pad_z,dict(floor_m=floor_distance,ceiling_m=ceiling_distance,left_wall_m=left,right_wall_m=right))
            break
        if selected:break
    if selected is None:raise ValueError('No supported enclosed spawn in the continuous interior sections')
    number,run,offset,pad_z,boundaries=selected
    forward=run['path'][offset:];indices=run['indices'][offset:]
    if anchored and np.linalg.norm(forward[0]-anchored['takeoff'])>1e-6:
        forward=np.r_[[anchored['takeoff']],forward]
        indices=np.r_[indices[0],indices]
    if max_forward_length is not None:
        if max_forward_length<=0:raise ValueError('max_forward_length must be positive')
        arc_forward=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(forward,axis=0),axis=1))]
        if arc_forward[-1]>max_forward_length:
            k=int(np.searchsorted(arc_forward,max_forward_length))
            fraction=(max_forward_length-arc_forward[k-1])/(arc_forward[k]-arc_forward[k-1])
            forward=np.r_[forward[:k],[forward[k-1]+fraction*(forward[k]-forward[k-1])]]
            indices=indices[:k+1]
    takeoff=forward[0]
    # Simplify each leg separately: simplifying an out-and-back polyline in
    # one pass can cut across its reversal and omit the true end of the leg.
    leg=[forward[0]];i=0
    while i<len(forward)-1:
        j=i+1
        for k in range(i+2,len(forward)):
            if np.linalg.norm(forward[k]-forward[i])>3.:break
            if visible(forward[i],forward[k]):j=k
        if np.linalg.norm(forward[j]-leg[-1])>.03:leg.append(forward[j])
        i=j
    assert np.allclose(leg[-1],forward[-1])
    waypoints=leg+leg[-2::-1]
    samples=np.concatenate([sampled_segment(a,b)for a,b in zip(waypoints[:-1],waypoints[1:])])
    measured=distance(samples)
    if measured.min()<clearance-1e-6:raise ValueError('Final line simplification failed clearance')
    heading=forward[min(3,len(forward)-1)]-forward[0]
    if anchored:heading=np.array([np.cos(anchored['spawn_yaw']),np.sin(anchored['spawn_yaw']),0.])
    spawn=[float(takeoff[0]),float(takeoff[1]),pad_z+.14]
    yaws=[None]*len(waypoints)
    if yaw_policy=='path-heading':
        delta=np.diff(waypoints,axis=0)
        headings=np.arctan2(delta[:,1],delta[:,0])
        yaws=np.r_[headings,headings[-1]].tolist()
    result=dict(schema_version=2,scenario=scenario,map_sha256=hashlib.sha256(pcd.read_bytes()).hexdigest(),
        basis='Connected indoor bag-derived section and exact return; only validated local repairs, no exterior joins',
        spawn=spawn,spawn_yaw=float(np.arctan2(heading[1],heading[0])),takeoff_height=float(takeoff[2]),takeoff=takeoff.tolist(),
        landing_pad=dict(center=[spawn[0],spawn[1],pad_z],half_size=.5),
        interior_spawn_boundaries=boundaries,source_section=number,
        source_bag_first_index=int(indices[0]),source_bag_last_index=int(indices[-1]),
        source_bag_start_seconds=float(times[indices[0]]),source_bag_end_seconds=float(times[indices[-1]]),
        outbound_recording_direction='reverse'if reverse_section else 'forward',
        spawn_distance_from_bag_start_m=float(np.linalg.norm(np.asarray(spawn)-bag[0])),
        lateral_adjustment_limit_m=lateral_limit,vertical_adjustment_limit_m=vertical_limit,same_side_witness_clearance_m=.10,
        witness_policy='Horizontal segment from bag XY at flight height'if vertical_limit>.35 else 'Direct bag-point segment',
        validated_local_connectors=[b for b in bridges if
            b['start_seconds']>=min(times[indices[0]],times[indices[-1]]) and
            b['end_seconds']<=max(times[indices[0]],times[indices[-1]])],
        connector_attempts=attempts,
        connector_attempts_scope='All examined bag sections; only in-range validated connectors are part of this route',
        offline_candidate_cost=dict(offset_weight=offset_weight,smoothness_weight=smoothness_weight),
        requested_max_forward_length_m=max_forward_length,
        offline_clearance_m=clearance,minimum_validated_center_clearance_m=float(measured.min()),
        length_m=float(np.linalg.norm(np.diff(waypoints,axis=0),axis=1).sum()),measured_bag_length_m=float(arc[-1]),
        waypoints=np.array(waypoints[1:]).tolist(),
        path_lookahead_m=4.0,
        yaw_waypoints=yaws,
        yaw_basis=('Explicit world-frame horizontal route tangent at each viewpoint' if yaw_policy=='path-heading' else
                   'Unspecified yaw: SUPER faces its replanned direction of travel; external exploration may supply explicit yaw goals'),
        viewpoints=[dict(x=float(p[0]),y=float(p[1]),z=float(p[2]),yaw=yaw)for p,yaw in zip(waypoints,yaws)],
        excluded_intervals='All other disconnected/narrow sections; never connected through exterior space',
        construction_wall_s=time.monotonic()-began,ground_truth_feedback=False,
        arrival_policy='Polyline progress with moving lookahead; intermediate waypoints do not require stopping; final landing requires speed below 0.7m/s within 0.15m',
        validation='Static clearance and enclosed spawn witnesses; source camera and flight checks required')
    output.write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(output.with_suffix('.npz'),path=np.array(waypoints),samples=samples,clearance=measured,
                        bag=bag,selected_bag=bag[indices],forward=forward)
    print(json.dumps({k:v for k,v in result.items()if k!='waypoints'},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    p.add_argument('--output',type=Path)
    p.add_argument('--scenario',choices=('dasan','frontier'),default='dasan')
    p.add_argument('--yaw-policy',choices=('travel','path-heading'),default='travel')
    p.add_argument('--clearance',type=float,default=.55)
    p.add_argument('--start-bag-seconds',type=float,default=18.)
    p.add_argument('--clip-start',action='store_true',help='Exclude earlier bag samples instead of only selecting sections ending after start')
    p.add_argument('--minimum-length',type=float,default=15.)
    p.add_argument('--lateral-limit',type=float,default=.35)
    p.add_argument('--vertical-limit',type=float,default=.35)
    p.add_argument('--end-bag-seconds',type=float)
    p.add_argument('--anchor',type=Path,help='Preserve the validated spawn/pad of an earlier mission')
    p.add_argument('--connect-gaps',action='store_true',help='Repair only short gaps within the bag corridor, with roof/floor witnesses')
    p.add_argument('--reverse-section',action='store_true',help='Start at the other end of the selected observed section')
    p.add_argument('--offset-weight',type=float,default=.2,help='Offline route preference for measured bag positions; not planner penalties')
    p.add_argument('--smoothness-weight',type=float,default=0.)
    p.add_argument('--max-forward-length',type=float,help='Stop before an excluded interval; return on the same validated prefix')
    args=p.parse_args();build(args.assets,args.output or args.assets/f'{args.scenario}_mission.json',args.clearance,args.start_bag_seconds,args.minimum_length,
        args.lateral_limit,args.vertical_limit,args.end_bag_seconds,args.anchor,args.connect_gaps,args.reverse_section,
        args.offset_weight,args.smoothness_weight,args.max_forward_length,args.scenario,args.yaw_policy,args.clip_start)
