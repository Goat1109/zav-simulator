#!/usr/bin/env python3
"""Conservative removal of camera-confirmed, subsequently empty human traces.

Candidates must project into confident person detections AND have repeated,
temporally separated LiDAR free-space evidence. Structural planar patches and
large components are retained. Writes a separate candidate; never carves a
mission corridor through arbitrary obstacles.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import time
import numpy as np
from scipy.ndimage import minimum_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
import yaml
from pcd_scene import read_pcd
from merge_dasan_fastlivo import write_pcd


def header(payload):
    sec,nano,length=struct.unpack_from('<iiI',payload)
    return sec+nano*1e-9,12+length


def image_message(data):
    payload=memoryview(data)[4:];stamp,offset=header(payload);offset=(offset+3)//4*4
    height,width,length=struct.unpack_from('<III',payload,offset);offset+=12
    encoding=bytes(payload[offset:offset+length-1]).decode();offset+=length
    offset+=1;offset=(offset+3)//4*4
    step,count=struct.unpack_from('<II',payload,offset);offset+=8
    if encoding not in ('rgb8','bgr8'):raise ValueError(encoding)
    image=np.frombuffer(payload,dtype=np.uint8,count=count,offset=offset).reshape(height,step)[:,:width*3].reshape(height,width,3)
    return stamp,image[:,:,::-1].copy()if encoding=='rgb8'else image.copy()


POINT=np.dtype({'names':['time','x','y','z','reflectivity','tag','line'],
    'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],
    'offsets':[0,4,8,12,16,17,18],'itemsize':20})


def lidar_message(data):
    # CDR omits the trailing struct padding of the final array element.
    payload=memoryview(data[4:]+b'\0');stamp,offset=header(payload);offset=(offset+7)//8*8
    _,point_num=struct.unpack_from('<QI',payload,offset);offset+=16
    count=struct.unpack_from('<I',payload,offset)[0];offset+=4
    if count!=point_num:raise ValueError('Livox CDR point count mismatch')
    return stamp,np.frombuffer(payload,dtype=POINT,count=count,offset=offset)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    p.add_argument('--bag',type=Path,default=Path.home()/'Bags/Dasan')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',type=Path,default=Path.home()/'Downloads/yolov8m.pt')
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    ws=Path(__file__).resolve().parents[3];assets=args.assets
    points=read_pcd(assets/'dasan.pcd');tree=cKDTree(points)
    source_sha=hashlib.sha256((assets/'dasan.pcd').read_bytes()).hexdigest()
    trajectory=np.genfromtxt(assets/'dasan_trajectory.csv',delimiter=',',names=True)
    t0=trajectory['t'][0];times=trajectory['t']-t0
    positions=np.column_stack([trajectory[k]for k in ('x','y','z')])
    rotations=Slerp(times,Rotation.from_quat(np.column_stack([trajectory[k]for k in ('qx','qy','qz','qw')])))
    config=ws/'src/FAST-LIVO2/config'
    ext=yaml.safe_load((config/'zenith_mid360s.yaml').read_text())['/laserMapping']['ros__parameters']['extrin_calib']
    cam=yaml.safe_load((config/'zenith_camera_pinhole.yaml').read_text())['/laserMapping']['ros__parameters']['camera']
    rcl=Rotation.from_matrix(np.array(ext['Rcl']).reshape(3,3)).as_matrix()
    ril=Rotation.from_matrix(np.array(ext['extrinsic_R']).reshape(3,3)).as_matrix();til=np.array(ext['extrinsic_T'])
    tcl=np.array(ext['Pcl']);k1,k2,p1,p2=[cam[f'cam_d{i}']for i in range(4)]
    semantic=np.zeros(len(points),dtype=np.uint16);detections=[]
    conn=sqlite3.connect(f'file:{next(args.bag.glob("*.db3"))}?mode=ro',uri=True)
    camera_topic=conn.execute("select id from topics where name='/camera/image_raw'").fetchone()[0]
    ids=[r[0]for r in conn.execute('select id from messages where topic_id=? order by timestamp',(camera_topic,))][::5]
    from ultralytics import YOLO
    import cv2
    model=YOLO(str(args.model));started=time.monotonic()
    for offset in range(0,len(ids),8):
        batch=[];stamps=[]
        for row in ids[offset:offset+8]:
            stamp,image=image_message(conn.execute('select data from messages where id=?',(row,)).fetchone()[0])
            if not times[0]<=stamp-t0<=times[-1]:continue
            batch.append(image);stamps.append(stamp-t0)
        if not batch:continue
        results=model.predict(batch,device=0,classes=[0],conf=.65,verbose=False,imgsz=640)
        for image,stamp,result in zip(batch,stamps,results):
            boxes=result.boxes.xyxy.cpu().numpy();confidence=result.boxes.conf.cpu().numpy()
            if not len(boxes):continue
            position=np.array([np.interp(stamp,times,positions[:,i])for i in range(3)])
            rotation=rotations(stamp).as_matrix()
            index=np.array(tree.query_ball_point(position,18.),dtype=int)
            optical=(((points[index]-position)@rotation-til)@ril)@rcl.T+tcl
            keep=optical[:,2]>.5;index=index[keep];optical=optical[keep]
            x,y=optical[:,0]/optical[:,2],optical[:,1]/optical[:,2];rad=x*x+y*y
            gain=1+k1*rad+k2*rad*rad
            u=cam['cam_fx']*(x*gain+2*p1*x*y+p2*(rad+2*x*x))+cam['cam_cx']
            v=cam['cam_fy']*(y*gain+p1*(rad+2*y*y)+2*p2*x*y)+cam['cam_cy']
            valid=(u>=2)&(u<638)&(v>=2)&(v<510)&(rad<1.8)
            index=index[valid];depth=optical[valid,2];u=u[valid];v=v[valid]
            pixel=(v.astype(int)//4)*160+(u.astype(int)//4)
            zbuffer=np.full(128*160,np.inf,np.float32);np.minimum.at(zbuffer,pixel,depth)
            zbuffer=minimum_filter(zbuffer.reshape(128,160),size=3,mode='constant',cval=np.inf).ravel()
            visible=depth<zbuffer[pixel]+.2
            for box,conf in zip(boxes,confidence):
                x1,y1,x2,y2=box;dx=x2-x1;dy=y2-y1
                inside=(u>x1+.15*dx)&(u<x2-.15*dx)&(v>y1+.08*dy)&(v<y2-.08*dy)&visible
                semantic[index[inside]]+=1
                detections.append(dict(seconds=float(stamp),box=box.tolist(),confidence=float(conf),projected_points=int(inside.sum())))
            if len(detections)<=10:
                cv2.imwrite(str(out/f'person-{len(detections):03d}.jpg'),result.plot())
        if offset%200==0:print(f'[People] camera {offset}/{len(ids)}, detections={len(detections)}, candidate points={np.count_nonzero(semantic)}',flush=True)
    np.savez_compressed(out/'semantic.npz',votes=semantic,source_sha256=source_sha)
    (out/'detections.json').write_text(json.dumps(detections,indent=2)+'\n')
    print(f'[People] semantic candidates={np.count_nonzero(semantic)}; validating later free-space observations',flush=True)
    resolution=.2
    cells,inverse=np.unique(np.floor(points.astype(float)/resolution).astype(np.int32),axis=0,return_inverse=True)
    lower=cells.min(0)-1;shape=cells.max(0)-lower+2;stride=np.array([int(shape[1])*int(shape[2]),int(shape[2]),1])
    lookup=np.full(int(np.prod(shape)),-1,np.int32);flat=(cells-lower)@stride;lookup[flat]=np.arange(len(cells),dtype=np.int32)
    def occupied_indices(p):
        c=np.floor(p/resolution).astype(np.int32)-lower;ok=((c>=0)&(c<shape)).all(1)
        return lookup[c[ok]@stride]
    hit=np.zeros(len(cells),np.uint16);miss=np.zeros(len(cells),np.uint16)
    first=np.full(len(cells),np.inf,np.float32);last=np.full(len(cells),-np.inf,np.float32)
    lidar_topic=conn.execute("select id from topics where name='/livox/lidar'").fetchone()[0]
    lidar_ids=[r[0]for r in conn.execute('select id from messages where topic_id=? order by timestamp',(lidar_topic,))][::5]
    ray_distance=np.arange(.5,25.,.2)
    for number,row in enumerate(lidar_ids):
        stamp,raw=lidar_message(conn.execute('select data from messages where id=?',(row,)).fetchone()[0])
        xyz=np.column_stack([raw[k]for k in ('x','y','z')]).astype(float);length=np.linalg.norm(xyz,axis=1)
        valid=np.isfinite(xyz).all(1)&(length>.8)&(length<25.)&(raw['line']<4)
        ids_valid=np.flatnonzero(valid);ids_valid=ids_valid[::max(1,int(np.ceil(len(ids_valid)/1600)))]
        if not len(ids_valid):continue
        query=stamp-t0+raw['time'][ids_valid].astype(float)*1e-9
        valid=(query>=times[0])&(query<=times[-1]);query=query[valid];ids_valid=ids_valid[valid]
        if not len(query):continue
        xyz=xyz[ids_valid];length=length[ids_valid]
        translation=np.column_stack([np.interp(query,times,positions[:,i])for i in range(3)])
        rotation=rotations(query)
        origin=translation+rotation.apply(np.broadcast_to(til,xyz.shape))
        endpoint=translation+rotation.apply(xyz@ril.T+til)
        indices=occupied_indices(endpoint);indices=np.unique(indices[indices>=0]);hit[indices]+=1
        direction=(endpoint-origin)/length[:,None]
        free=origin[:,None,:]+direction[:,None,:]*ray_distance[None,:,None]
        valid=ray_distance[None,:]<length[:,None]-.6
        indices=occupied_indices(free[valid]);indices=np.unique(indices[indices>=0])
        miss[indices]+=1;first[indices]=np.minimum(first[indices],stamp-t0);last[indices]=np.maximum(last[indices],stamp-t0)
        if number%200==0:print(f'[People] ray evidence {number}/{len(lidar_ids)}',flush=True)
    conn.close()
    candidate_votes=np.bincount(inverse,weights=semantic,minlength=len(cells))
    candidate=(candidate_votes>0)&(miss>=8)&(hit<=4)&(last-first>=20.)
    # Retain flat structural floor/wall/ceiling patches even if pose drift puts
    # a few free rays through their fused surface.
    coarse_tree=cKDTree(cells*resolution+resolution/2)
    ids=np.flatnonzero(candidate)
    if len(ids):
        distance,near=coarse_tree.query(cells[ids]*resolution+resolution/2,k=20,workers=4)
        q=cells[near].astype(float)*resolution;q-=q.mean(1,keepdims=True)
        eigen,vector=np.linalg.eigh(np.einsum('nki,nkj->nij',q,q))
        planar=eigen[:,0]/np.maximum(eigen[:,1],1e-9)<.03
        candidate[ids[planar]]=False
    # Only human-sized sparse components, never a large architectural surface.
    ids=np.flatnonzero(candidate);accepted=np.zeros(len(cells),bool);components=[]
    if len(ids):
        coords=cells[ids];parent=np.arange(len(ids))
        def find(i):
            while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
            return i
        for a,b in cKDTree(coords).query_pairs(1.8):parent[find(a)]=find(b)
        groups={}
        for i in range(len(ids)):groups.setdefault(find(i),[]).append(i)
        for group in groups.values():
            selected=ids[group];world=cells[selected]*resolution+resolution/2
            extent=np.ptp(world,axis=0)
            if len(group)<3 or len(group)>400 or max(extent[:2])>2.4 or extent[2]>2.5:continue
            accepted[selected]=True;components.append(dict(voxels=len(group),lower=world.min(0).tolist(),upper=world.max(0).tolist()))
    remove=accepted[inverse]
    np.savez_compressed(out/'evidence.npz',cells=cells,hit=hit,miss=miss,first=first,last=last,semantic=semantic,remove=remove)
    write_pcd(out/'removed.pcd',points[remove])
    for name in ('dasan_recipe.json','dasan_mapping.yaml','dasan_trajectory.csv'):
        shutil.copyfile(assets/name,out/name)
    write_pcd(out/'dasan.pcd',points[~remove])
    colors=np.load(assets/'dasan_color.npz');new_sha=hashlib.sha256((out/'dasan.pcd').read_bytes()).hexdigest()
    np.savez_compressed(out/'dasan_color.npz',rgb=colors['rgb'][~remove],direct=colors['direct'][~remove],
        filled=colors['filled'][~remove],geometry_sha256=new_sha)
    write_pcd(out/'dasan_rgb.pcd',points[~remove],colors['rgb'][~remove])
    report=dict(source_sha256=source_sha,filtered_sha256=new_sha,model=str(args.model),
        model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),detections=len(detections),
        semantic_points=int(np.count_nonzero(semantic)),removed_points=int(remove.sum()),
        removed_components=components,remaining_points=int((~remove).sum()),wall_s=time.monotonic()-started,
        conditions='person conf>=.65, foreground projection, >=8 free scans spanning >=20s, <=4 endpoint scans; planar/large structures retained',
        limitations='Conservative candidate filter; standing people and uncertain traces may remain. Review detections/removal before adoption.')
    recipe=json.loads((out/'dasan_recipe.json').read_text());recipe['generated_pcd_sha256']=new_sha
    recipe['points']=int((~remove).sum());recipe['human_trace_filter']=report
    (out/'dasan_recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
    color_meta=json.loads((assets/'dasan_color.json').read_text());color_meta.update(geometry_sha256=new_sha,points=int((~remove).sum()),
        direct_camera_points=int(colors['direct'][~remove].sum()),uncoloured_points=int((~(colors['direct'][~remove]|colors['filled'][~remove])).sum()),
        color_sha256=hashlib.sha256((out/'dasan_color.npz').read_bytes()).hexdigest())
    (out/'dasan_color.json').write_text(json.dumps(color_meta,indent=2)+'\n')
    (out/'people-filter-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
