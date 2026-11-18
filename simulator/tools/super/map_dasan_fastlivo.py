#!/usr/bin/env python3
"""Replay the real bag through FAST-LIVO2 and retain native world/RGB scans.

No PX4 bridge or control node runs. The isolated mapper's PCD writer is disabled
so existing Log/PCD files are preserved. Full scan and native colour fragments,
measurement odometry, parameters and counters go into a fresh output directory.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def cloud_values(msg, color):
    fields={f.name:f for f in msg.fields}
    offset=[fields[k].offset for k in ('x','y','z')]
    xyz=np.column_stack([np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,
        offset=o,strides=(msg.row_step,msg.point_step)).reshape(-1)for o in offset])
    finite=np.isfinite(xyz).all(1)
    if not color:return xyz[finite]
    packed=np.ndarray((msg.height,msg.width),dtype='<u4',buffer=msg.data,
                      offset=fields['rgb'].offset,strides=(msg.row_step,msg.point_step)).reshape(-1)[finite]
    rgb=np.column_stack([(packed>>16)&255,(packed>>8)&255,packed&255]).astype(np.uint8)
    return xyz[finite],rgb


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,default=Path.home()/'Bags/Dasan')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--domain',type=int,default=72)
    args=parser.parse_args()
    ws=Path(__file__).resolve().parents[3];out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False);(out/'fragments').mkdir()
    params=yaml.safe_load((ws/'src/FAST-LIVO2/config/zenith.yaml').read_text())
    p=params['/laserMapping']['ros__parameters']
    p['common']['img_en']=1
    p['pcd_save']['pcd_save_en']=False
    p['evo']['pose_output_en']=False
    p['use_sim_time']=True
    (out/'mapping.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    camera=ws/'src/FAST-LIVO2/config/zenith_camera_pinhole.yaml'
    shutil.copyfile(camera,out/'camera.yaml')
    # The existing implementation always opens diagnostic mat logs in ROOT/Log.
    for name in ('mat_pre.txt','mat_out.txt'):
        path=ws/'src/FAST-LIVO2/Log'/name
        if path.exists():shutil.copyfile(path,out/f'previous_{name}')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    rclpy.init();node=Node('dasan_fastlivo_capture')
    file=(out/'trajectory.csv').open('x');writer=csv.writer(file)
    writer.writerow(['t','x','y','z','qx','qy','qz','qw'])
    counts={'odom':0,'full_scans':0,'rgb_scans':0,'full_points':0,'rgb_points':0}
    stamps={'odom':[],'full_scans':[]}
    buffers={'full':[],'rgb':[]};rgb_buffers=[];sequence={'full':0,'rgb':0}
    def flush(kind):
        if not buffers[kind]:return
        values={'xyz':np.concatenate(buffers[kind])}
        if kind=='rgb':values['rgb']=np.concatenate(rgb_buffers);rgb_buffers.clear()
        np.savez(out/'fragments'/f'{kind}_{sequence[kind]:04d}.npz',**values)
        buffers[kind].clear();sequence[kind]+=1
    def odom(msg):
        v,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        writer.writerow([stamp,v.x,v.y,v.z,q.x,q.y,q.z,q.w]);stamps['odom'].append(stamp)
        counts['odom']+=1
        if counts['odom']%100==0:file.flush()
    def cloud(msg,kind):
        if kind=='rgb':
            xyz,rgb=cloud_values(msg,True);rgb_buffers.append(rgb)
        else:
            xyz=cloud_values(msg,False)
            stamps['full_scans'].append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        buffers[kind].append(xyz)
        counts[f'{kind}_scans']+=1;counts[f'{kind}_points']+=len(xyz)
        if len(buffers[kind])>=100:flush(kind)
    qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry,'/aft_mapped_to_init',odom,qos)
    node.create_subscription(PointCloud2,'/cloud_registered_full',lambda m:cloud(m,'full'),qos)
    node.create_subscription(PointCloud2,'/cloud_registered',lambda m:cloud(m,'rgb'),qos)
    mapper_log=(out/'mapper.log').open('x')
    mapper=subprocess.Popen([str(ws/'install/fast_livo/lib/fast_livo/fastlivo_mapping'),
        '--ros-args','--params-file',str(out/'mapping.yaml'),'--params-file',str(out/'camera.yaml')],
        stdout=mapper_log,stderr=subprocess.STDOUT)
    player=None;complete=False
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        player_log=(out/'playback.log').open('x')
        player=subprocess.Popen(['ros2','bag','play',str(args.bag.resolve()),'--rate',str(args.rate),
            '--clock','--read-ahead-queue-size','10000','--disable-keyboard-controls'],
            stdout=player_log,stderr=subprocess.STDOUT)
        next_report=time.monotonic()
        while player.poll() is None:
            if mapper.poll() is not None:raise RuntimeError('FAST-LIVO2 exited; inspect mapper.log')
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>=next_report:
                print(f'[FAST-LIVO2] {counts}',flush=True);next_report=time.monotonic()+25
        if player.returncode:raise RuntimeError('Bag playback failed')
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        complete=True
    finally:
        if player is not None and player.poll() is None:
            player.send_signal(signal.SIGINT);player.wait(timeout=30)
        if mapper.poll() is None:
            mapper.send_signal(signal.SIGINT);mapper.wait(timeout=45)
        for kind in buffers:flush(kind)
        file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for name in ('mat_pre.txt','mat_out.txt'):
            path=ws/'src/FAST-LIVO2/Log'/name
            if path.exists():shutil.copyfile(path,out/name)
            previous=out/f'previous_{name}'
            if previous.exists():shutil.copyfile(previous,path)
        gaps={}
        for name,values in stamps.items():
            dt=np.diff(values)
            gaps[name]={'count':len(values),'stamp_first':values[0] if values else None,
                'stamp_last':values[-1] if values else None,'backwards':int((dt<0).sum()),
                'max_gap_s':float(dt.max()) if len(dt)else None}
        (out/'capture.json').write_text(json.dumps(dict(complete=complete,backend='FAST-LIVO2',
            rate=args.rate,counts=counts,stamps=gaps,bag=str(args.bag.resolve())),indent=2)+'\n')
    if not complete or counts['full_scans']<100:raise RuntimeError('Incomplete replay')
    print(f'[FAST-LIVO2] Complete: {out}',flush=True)


if __name__=='__main__':main()
