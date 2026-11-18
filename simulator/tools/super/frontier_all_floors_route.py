#!/usr/bin/env python3
"""Visit each recorded Frontier floor corridor and finish at the highest landing.

Consumes the clearance-checked dense bag candidate. The default retains floor
corridor visits and omits descent. Optional stair-only mode is a separate test.
The offline map is never published into the online planner.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from pcd_scene import read_pcd
from dasan_route import sampled_segment


def build(candidate, output, assets, corridor_reach=None, turn_exit_holds=False, ascent_only=False, no_return=True, align_live_frame=False, stop_at_viewpoints=False):
    mission=json.loads(candidate.read_text());data=np.load(candidate.with_suffix('.npz'))
    geometry=read_pcd(assets/'frontier.pcd')
    centers=np.unique(np.floor(geometry.astype(float)/.1).astype(np.int32),axis=0)*.1+.05
    tree=cKDTree(centers);clearance=.55
    candidate_clearance=mission['offline_clearance_m']
    def distance(points):return tree.query(points,workers=4 if len(points)>10000 else 1)[0]-np.sqrt(3)*.05
    def visible(a,b):return bool((distance(sampled_segment(a,b,.04))>=clearance).all())
    forward=data['forward']
    if forward[:,2].max()<data['bag'][:,2].max()-1.:
        raise ValueError('Candidate does not reach every recorded Frontier level')
    top=int(np.argmax(forward[:,2]));forward=forward[:top+1]
    route=[forward[0]];i=0
    while i<len(forward)-1:
        j=i+1
        for k in range(i+2,len(forward)):
            if np.linalg.norm(forward[k]-forward[i])>3.:break
            if visible(forward[i],forward[k]):j=k
        route.append(forward[j]);i=j
    outbound=np.array(route)
    corridor_trims=[]
    if corridor_reach is not None:
        clipped=[];i=0
        while i<len(outbound):
            if outbound[i,0]<-6.5 and outbound[i,1]>corridor_reach and i>0:
                j=i
                while j<len(outbound) and outbound[j,0]<-6.5 and outbound[j,1]>corridor_reach:j+=1
                if j<len(outbound):
                    a=outbound[i-1]+(outbound[i]-outbound[i-1])*(corridor_reach-outbound[i-1,1])/(outbound[i,1]-outbound[i-1,1])
                    b=outbound[j-1]+(outbound[j]-outbound[j-1])*(corridor_reach-outbound[j-1,1])/(outbound[j,1]-outbound[j-1,1])
                    if visible(a,b):
                        clipped.extend([a,b]);corridor_trims.append(dict(first_vertex=i,last_vertex=j-1,limit_y_m=corridor_reach))
                        i=j;continue
            clipped.append(outbound[i]);i+=1
        outbound=np.array(clipped)
    # The input used 6cm sampling; the export uses 4cm. Repair only tiny
    # sub-millimetre deficits exposed by the denser sampling, never relax the
    # 0.55m minimum. Every modified adjacent edge is checked again below.
    numeric_adjustments=[]
    for _ in range(10):
        changed=False
        for i,(a,b) in enumerate(zip(outbound[:-1],outbound[1:])):
            dense=sampled_segment(a,b,.04);values=distance(dense);minimum=float(values.min())
            if minimum>=clearance:continue
            if minimum<clearance-.002:raise ValueError('Input edge needs a geometric repair beyond numerical sampling adjustment')
            point=dense[values.argmin()];nearest=centers[tree.query(point)[1]]
            delta=(point-nearest)/np.linalg.norm(point-nearest)*.001
            if i>0:outbound[i]+=delta
            outbound[i+1]+=delta;changed=True
            numeric_adjustments.append(dict(edge=i,previous_clearance_m=minimum,shift_m=delta.tolist()))
        if not changed:break
    # Horizontal corridor/connector observations separate the recorded level
    # bands. Use those bands rather than a fragile single-vertex Y reversal:
    # centering the route can spread a landing turn over several vertices.
    raw=data['bag'];floor_samples=np.sort(raw[(raw[:,0]<-3.)&((raw[:,1]>8.)|(np.abs(raw[:,1])<2.)),2])
    groups=np.split(floor_samples,np.flatnonzero(np.diff(floor_samples)>.8)+1)
    raw_levels=[float(np.median(g))for g in groups if len(g)>=10]
    raw_levels=[z for z in raw_levels if z>=outbound[0,2]-.8 and z<outbound[-1,2]-1.]
    raw_levels.append(float(outbound[-1,2]))
    levels=[dict(vertex=0,z=float(outbound[0,2]))];stair_pauses=[];previous=0
    for target in raw_levels[1:]:
        if target==raw_levels[-1]:arrival=len(outbound)-1
        else:
            candidates=[i for i in range(previous+1,len(outbound))if
                outbound[i,0]>-2.5 and outbound[i,1]<2.5 and target-.75<=outbound[i,2]<=target+.35]
            if not candidates:raise ValueError(f'No observed arrival on level band {target:.2f}')
            arrival=candidates[0]
        candidates=[i for i in range(previous+1,arrival)if outbound[i,0]>-2.5 and
                    outbound[i,2]>outbound[previous,2]+.8 and outbound[i,2]<outbound[arrival,2]-.4]
        if candidates:
            maximum_y=max(outbound[i,1]for i in candidates)
            plateau=[i for i in candidates if outbound[i,1]>=maximum_y-.15]
            midpoint=max(plateau,key=lambda i:float(distance(outbound[i:i+1])[0]))
            stair_pauses.append(midpoint)
            if turn_exit_holds and midpoint+1<arrival:stair_pauses.append(midpoint+1)
        stair_pauses.append(arrival);levels.append(dict(vertex=arrival,z=float(outbound[arrival,2])))
        previous=arrival
    stair_pauses=sorted(set(stair_pauses))
    junctions=[];groups=[];current=[]
    for i in range(1,len(outbound)-1):
        if outbound[i,0]<-6.5 and outbound[i,1]<3.:current.append(i)
        elif current:groups.append(current);current=[]
    if current:groups.append(current)
    def angle(i):
        u=outbound[i,:2]-outbound[i-1,:2];v=outbound[i+1,:2]-outbound[i,:2]
        return float(np.arccos(np.clip(u@v/max(np.linalg.norm(u)*np.linalg.norm(v),1e-9),-1.,1.)))
    for group in groups:
        i=max(group,key=angle)
        if angle(i)>np.deg2rad(15.):junctions.append(i)
    pauses=sorted(set([*stair_pauses,*junctions]))
    reverse=outbound if ascent_only else outbound[::-1]
    protected=set(stair_pauses) if ascent_only else {len(outbound)-1-i for i in stair_pauses}
    returned=[reverse[0]];return_indices=[0];shortcuts=[];i=0
    while i<len(reverse)-1:
        barrier=min([j for j in protected if j>i]+[len(reverse)-1])
        nearby=np.flatnonzero(np.linalg.norm(reverse[i:barrier+1]-reverse[i],axis=1)<=1.5)+i
        next_index=i+1
        for j in nearby[::-1]:
            if j<=i+2:break
            removed=float(np.linalg.norm(np.diff(reverse[i:j+1],axis=0),axis=1).sum())
            if removed>5. and visible(reverse[i],reverse[j]):
                next_index=int(j);shortcuts.append(dict(from_reverse_vertex=i,to_reverse_vertex=int(j),
                    replaced_arc_m=removed,join_m=float(np.linalg.norm(reverse[j]-reverse[i]))));break
        returned.append(reverse[next_index]);return_indices.append(next_index);i=next_index
    if ascent_only:
        path=np.array(returned)
        index_map={source:i for i,source in enumerate(return_indices)}
        levels=[dict(vertex=index_map[level['vertex']],z=level['z'])for level in levels]
        pause_indices=[index_map[i]for i in stair_pauses if i<len(outbound)-1]
        junctions=[]
    elif no_return:
        path=outbound
        pause_indices=[i for i in pauses if i<len(outbound)-1]
    else:
        path=np.concatenate([outbound,np.array(returned)[1:]])
        pause_indices=pauses.copy()
        for i,source in enumerate(return_indices[1:],start=1):
            if source in protected and i<len(return_indices)-1:pause_indices.append(len(outbound)-1+i)
    samples=np.concatenate([sampled_segment(a,b,.04)for a,b in zip(path[:-1],path[1:])])
    measured=distance(samples)
    if measured.min()<clearance-1e-6:
        for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
            dense=sampled_segment(a,b,.04);values=distance(dense)
            if values.min()<clearance-1e-6:print('BAD_EDGE',index,float(values.min()),dense[values.argmin()].tolist(),a.tolist(),b.tolist(),flush=True)
        raise ValueError(f'All-floor route clearance {measured.min():.6f} below {clearance:.6f}')
    trajectory=np.genfromtxt(assets/'frontier_trajectory.csv',delimiter=',',names=True)
    bag=np.column_stack([trajectory[k]for k in ('x','y','z')])
    source_indices=cKDTree(bag).query(data['selected_bag'])[1]
    end_index=int(source_indices[top]);delta=np.diff(path,axis=0)
    yaws=np.r_[np.arctan2(delta[:,1],delta[:,0]),np.arctan2(delta[-1,1],delta[-1,0])].tolist()
    mission.pop('flight_validation',None)
    observation_candidates=sorted(set(pause_indices))
    if not stop_at_viewpoints:pause_indices=[]
    mission.update(offline_clearance_m=clearance,candidate_clearance_m=candidate_clearance,
        basis=('Stair-only one-way ascent through every recorded level; finish at highest landing, no corridor loops or return' if ascent_only else
               'Ascend to each recorded floor and visit its bag corridor; finish at highest landing, no descent or ground return' if no_return else
               'Every observed level via ascending bag route; checked stair-spine return excludes repeated corridor loops'),
        return_to_start=not(ascent_only or no_return),ascent_only=ascent_only or no_return,stairs_only=ascent_only,
        source_bag_end_seconds=float(trajectory['t'][end_index]-trajectory['t'][0]),source_bag_last_index=end_index,
        length_m=float(np.linalg.norm(delta,axis=1).sum()),waypoints=path[1:].tolist(),yaw_waypoints=yaws,
        viewpoints=[dict(x=float(p[0]),y=float(p[1]),z=float(p[2]),yaw=y)for p,y in zip(path,yaws)],
        continuous_waypoints=not stop_at_viewpoints,
        viewpoint_pause_indices=sorted(set(pause_indices)),viewpoint_wait_seconds=3. if stop_at_viewpoints else 0.,end_wait_seconds=3.,
        observation_candidate_indices=observation_candidates,
        viewpoint_pause_basis=('Observed stair mid-landings, floor arrivals and corridor junctions' if stop_at_viewpoints else
                               'Continuous waypoint traversal; only the final endpoint requests hold and landing'),
        corridor_junction_viewpoints=junctions,allow_blocked_loop_skip=False,bag_map_alignment=align_live_frame,
        corridor_trims=corridor_trims,turn_exit_holds=turn_exit_holds,
        numeric_clearance_adjustments=numeric_adjustments,
        blocked_loop_policy='Only a nearby same-height return join after 8 seconds stopped without progress; never omit stairs or levels',
        all_floor_levels=levels,recorded_level_bands_m=raw_levels,ascending_vertices=len(path)if ascent_only or no_return else len(outbound),loop_shortcuts=shortcuts if not no_return else [],return_loop_shortcuts=[] if ascent_only or no_return else shortcuts,
        maximum_commanded_height_m=float(path[:,2].max()),
        minimum_validated_center_clearance_m=float(measured.min()),
        validation='Offline dense edge clearance only; full GUI flight required')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(mission,indent=2)+'\n')
    np.savez_compressed(output.with_suffix('.npz'),path=path,samples=samples,clearance=measured,
                        bag=bag,forward=path if ascent_only or no_return else outbound,return_path=np.empty((0,3)) if ascent_only or no_return else np.array(returned))
    print(json.dumps({k:mission[k]for k in ('length_m','all_floor_levels','viewpoint_pause_indices',
          'maximum_commanded_height_m','source_bag_end_seconds','return_loop_shortcuts',
          'minimum_validated_center_clearance_m')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    p.add_argument('--corridor-reach',type=float,help='Retain this Y extent of each observed floor corridor')
    p.add_argument('--turn-exit-holds',action='store_true',help='Observe immediately after each stair-landing reversal')
    p.add_argument('--ascent-only',action='store_true',help='Remove corridor loops, ascend all levels and finish at the highest landing')
    p.add_argument('--no-return',action='store_true',default=True,help='Default: keep floor corridor visits and finish at highest landing without descending')
    p.add_argument('--align-live-frame',action='store_true',help='Use live LiDAR/reference-map registration in the bag path publisher')
    p.add_argument('--stop-at-viewpoints',action='store_true',help='Legacy diagnostic mode: stop and wait at intermediate observation points')
    a=p.parse_args();build(a.candidate,a.output,a.assets,a.corridor_reach,a.turn_exit_holds,a.ascent_only,a.no_return,a.align_live_frame,a.stop_at_viewpoints)
