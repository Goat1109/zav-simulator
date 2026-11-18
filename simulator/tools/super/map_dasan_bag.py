#!/usr/bin/env python3
"""Reconstruct a raw Mid360 bag with Super-LIO in an isolated ROS domain.

Run after sourcing simulator/ros_setup.sh. Input bag is read only. Each output
directory must be new; native mapper fragments and the measured trajectory are
retained alongside the mapping configuration and playback logs.
"""
import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home()/'Bags/Dasan')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--domain', type=int, default=71)
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[3]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    config = yaml.safe_load((ws/'src/Super-LIO/super_lio/config/livox_360.yaml').read_text())
    params = config['/**']['ros__parameters']
    params.update({
        'lio.map.save_map': True,
        'lio.map.save_map_dir': os.path.relpath(out, ws/'src/Super-LIO/super_lio'),
        'lio.map.map_name': 'dasan_raw.pcd', 'lio.map.ds_size': .15,
        'lio.map.save_interval': 250, 'lio.sensor.blind': .5,
        'lio.sensor.voxel_fliter_size': .2, 'lio.sensor.filter_rate': 2,
        'lio.sensor.gravity_norm': 9.80665,
        # Mid360 factory LiDAR -> internal IMU translation, also used by ZAV.
        'lio.extrinsic.lidar_imu': [-.011, -.02329, -.04412, 1.,0.,0.,0.,1.,0.,0.,0.,1.],
        'lio.output.robot': False, 'lio.output.map': False, 'lio.eva.timer': False})
    (out/'mapping.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    rclpy.init()
    node = Node('dasan_mapping_trajectory')
    file = (out/'trajectory.csv').open('x')
    writer = csv.writer(file)
    writer.writerow(['t','x','y','z','qx','qy','qz','qw'])
    count = 0
    def odom(msg):
        nonlocal count
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        writer.writerow([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                         p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        count += 1
        if count % 100 == 0: file.flush()
    node.create_subscription(Odometry, '/lio/odom', odom, 1000)
    mapper_log = (out/'mapper.log').open('x')
    mapper = subprocess.Popen([str(ws/'install/super_lio/lib/super_lio/super_lio_node'),
                               '--ros-args','--params-file',str(out/'mapping.yaml')],
                              stdout=mapper_log, stderr=subprocess.STDOUT)
    player = None
    try:
        until = time.monotonic()+3
        while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.1)
        player_log = (out/'playback.log').open('x')
        player = subprocess.Popen(['ros2','bag','play',str(args.bag.resolve()),
            '--rate',str(args.rate),'--topics','/livox/lidar','/livox/imu',
            '--read-ahead-queue-size','10000','--disable-keyboard-controls'],
            stdout=player_log, stderr=subprocess.STDOUT)
        next_report = time.monotonic()
        while player.poll() is None:
            if mapper.poll() is not None: raise RuntimeError('Mapper exited; inspect mapper.log')
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic()>=next_report:
                print(f'[Mapping] measured poses={count}', flush=True)
                next_report = time.monotonic()+20
        if player.returncode: raise RuntimeError('Bag playback failed; inspect playback.log')
        until = time.monotonic()+5
        while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if player is not None and player.poll() is None:
            player.send_signal(signal.SIGINT)
            player.wait(timeout=30)
        if mapper.poll() is None:
            mapper.send_signal(signal.SIGINT)
            mapper.wait(timeout=180)
        file.close()
        node.destroy_node()
        rclpy.shutdown()
    if count<100 or not (out/'dasan_raw.pcd').is_file():
        raise RuntimeError('Incomplete reconstruction')
    print(f'[Mapping] Finished: {out}, poses={count}', flush=True)


if __name__ == '__main__':
    main()
