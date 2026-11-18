#!/usr/bin/env python3
"""Project the bag's calibrated RGB camera onto the measured Dasan surface.

Uses camera header timestamps, interpolated LIO IMU poses, lens distortion,
coarse conservative z-buffer occlusion, and the nearest well-centred view.
Unobserved points remain labelled as uncoloured, never assigned invented RGB.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import time
import numpy as np
from scipy.ndimage import minimum_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from pcd_scene import read_pcd


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,default=Path.home()/'Bags/Dasan')
    parser.add_argument('--diagnostics',type=Path,default=Path('log/dasan-20261117'))
    parser.add_argument('--stride',type=int,default=5)
    parser.add_argument('--max-range',type=float,default=35.)
    args=parser.parse_args()
    args.diagnostics.mkdir(parents=True,exist_ok=True)
    ws=Path(__file__).resolve().parents[3];assets=ws/'simulator/assets/super'
    points=read_pcd(assets/'dasan.pcd');tree=cKDTree(points)
    trajectory=np.genfromtxt(assets/'dasan_trajectory.csv',delimiter=',',names=True)
    stamps=trajectory['t'];t0=stamps[0];times=stamps-t0
    positions=np.column_stack([trajectory[k]for k in ('x','y','z')])
    rotations=Slerp(times,Rotation.from_quat(np.column_stack([trajectory[k]for k in ('qx','qy','qz','qw')])))
    config=ws/'src/FAST-LIVO2/config'
    params=yaml.safe_load((config/'zenith_mid360s.yaml').read_text())['/laserMapping']['ros__parameters']
    camera=yaml.safe_load((config/'zenith_camera_pinhole.yaml').read_text())['/laserMapping']['ros__parameters']['camera']
    ext=params['extrin_calib'];r_cl=Rotation.from_matrix(np.array(ext['Rcl']).reshape(3,3)).as_matrix()
    r_il=Rotation.from_matrix(np.array(ext['extrinsic_R']).reshape(3,3)).as_matrix()
    t_il=np.array(ext['extrinsic_T']);t_cl=np.array(ext['Pcl'])
    k1,k2,p1,p2=[camera[f'cam_d{i}']for i in range(4)]
    rgb=np.full((len(points),3),128,dtype=np.uint8)
    quality=np.zeros(len(points),dtype=np.float32)
    db=next(args.bag.glob('*.db3'))
    conn=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    topic=conn.execute("select id from topics where name='/camera/image_raw'").fetchone()[0]
    ids=[v[0]for v in conn.execute('select id from messages where topic_id=? order by timestamp',(topic,))][::args.stride]
    frames=0;next_report=time.monotonic();skipped=0
    for row_id in ids:
        payload=conn.execute('select data from messages where id=?',(row_id,)).fetchone()[0]
        msg=deserialize_message(payload,Image)
        timestamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-t0
        if not times[0]<=timestamp<=times[-1]:skipped+=1;continue
        if (msg.width,msg.height)!=(camera['cam_width'],camera['cam_height']):
            raise ValueError('Bag image dimensions differ from camera calibration')
        if msg.encoding not in ('rgb8','bgr8'):raise ValueError(msg.encoding)
        image=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='bgr8':image=image[:,:,::-1]
        position=np.array([np.interp(timestamp,times,positions[:,i])for i in range(3)])
        rotation=rotations(timestamp).as_matrix()
        indices=np.array(tree.query_ball_point(position,args.max_range),dtype=np.int64)
        # world -> IMU -> lidar -> optical camera; row-vector implementation.
        local=((points[indices]-position)@rotation-t_il)@r_il
        optical=local@r_cl.T+t_cl
        valid=optical[:,2]>.5
        indices=indices[valid];optical=optical[valid]
        x,y=optical[:,0]/optical[:,2],optical[:,1]/optical[:,2]
        radius=x*x+y*y;radial=1+k1*radius+k2*radius*radius
        u=camera['cam_fx']*(x*radial+2*p1*x*y+p2*(radius+2*x*x))+camera['cam_cx']
        v=camera['cam_fy']*(y*radial+p1*(radius+2*y*y)+2*p2*x*y)+camera['cam_cy']
        valid=(u>=3)&(u<msg.width-3)&(v>=3)&(v<msg.height-3)&(radius<1.8)
        indices=indices[valid];depth=optical[valid,2];radius=radius[valid]
        u=np.rint(u[valid]).astype(np.int32);v=np.rint(v[valid]).astype(np.int32)
        # Four-pixel depth cells and a one-cell neighborhood reduce projection
        # through holes in sampled foreground surfaces. Unknown depth is ignored.
        scale=4;h=(msg.height+scale-1)//scale;w=(msg.width+scale-1)//scale
        zbuffer=np.full(h*w,np.inf,dtype=np.float32)
        pixel=(v//scale)*w+u//scale
        np.minimum.at(zbuffer,pixel,depth)
        zbuffer=minimum_filter(zbuffer.reshape(h,w),size=3,mode='constant',cval=np.inf).reshape(-1)
        visible=depth<=zbuffer[pixel]+.18+.006*depth
        score=(1./(1.+radius))/(depth*depth)
        better=visible&(score>quality[indices])
        selected=indices[better]
        rgb[selected]=image[v[better],u[better]]
        quality[selected]=score[better]
        frames+=1
        if frames in (20,200):
            from PIL import Image as PIL
            PIL.fromarray(image).save(args.diagnostics/f'color-source-{frames}.png')
            overlay=image.copy()
            # The camera-coloured projected surface should follow image edges.
            overlay[v[visible],u[visible]]=[255,80,30]
            PIL.fromarray(overlay).save(args.diagnostics/f'color-projection-{frames}.png')
        if time.monotonic()>=next_report:
            print(f'[Colour] {frames}/{len(ids)} images; coloured={np.count_nonzero(quality)}/{len(points)}',flush=True)
            next_report=time.monotonic()+20
    conn.close()
    # Interpolate only tiny sampling holes, retaining the direct-view mask.
    direct=quality>0;filled=np.zeros(len(points),dtype=bool)
    if direct.any() and (~direct).any():
        ids_direct=np.flatnonzero(direct);ids_missing=np.flatnonzero(~direct)
        distance,near=cKDTree(points[direct]).query(points[~direct],distance_upper_bound=.3,workers=4)
        take=np.isfinite(distance);rgb[ids_missing[take]]=rgb[ids_direct[near[take]]];filled[ids_missing[take]]=True
    np.savez_compressed(assets/'dasan_color.npz',rgb=rgb,direct=direct,filled=filled,quality=quality,
                        geometry_sha256=hashlib.sha256((assets/'dasan.pcd').read_bytes()).hexdigest())
    packed=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
    records=np.empty(len(points),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for i,k in enumerate(('x','y','z')):records[k]=points[:,i]
    records['rgb']=packed
    with (assets/'dasan_rgb.pcd').open('wb')as file:
        file.write((f'VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n').encode())
        records.tofile(file)
    metadata=dict(method='Calibrated RGB projection using camera header time and interpolated LIO pose',
                  frames_used=frames,frames_skipped_outside_pose_interval=skipped,image_stride=args.stride,
                  max_range_m=args.max_range,points=len(points),direct_camera_points=int(direct.sum()),
                  small_gap_interpolated_points=int(filled.sum()),uncoloured_points=int((~(direct|filled)).sum()),
                  exposure='Original recorded RGB bytes; no invented colours',
                  camera_calibration=camera,extrinsic_calibration=ext,
                  geometry_sha256=hashlib.sha256((assets/'dasan.pcd').read_bytes()).hexdigest(),
                  color_sha256=hashlib.sha256((assets/'dasan_color.npz').read_bytes()).hexdigest(),
                  limitations=['Uses the repository camera calibration; the bag contains no calibration topic.',
                               'Pose drift, camera/LiDAR timing and low exposure limit colour alignment.',
                               'Unobserved points are neutral grey; the RGB mask is preserved in NPZ.'])
    (assets/'dasan_color.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
