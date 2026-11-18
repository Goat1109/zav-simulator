#!/usr/bin/env python3
"""Passive flight recorder. Truth is diagnostic only; never publishes control."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import math
from pathlib import Path
import time
import sys
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--mission',type=Path)
    p.add_argument('--scenario',required=True);p.add_argument('--backend',required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    ws=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(ws/'simulator/pegasus/simulator/logic/sensors'))
    from fastlivo_calibration import load_calibration
    import os
    calibration=load_calibration(os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR',str(ws/'src/FAST-LIVO2/config')),[.01215,0.,.11503],[0.,22.,0.])
    metadata={'schema_version':1,'created_at':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
        'scenario':args.scenario,'backend':args.backend,'planner_config_path':str(args.config),
        'planner':yaml.safe_load(args.config.read_text().expandtabs()),
        'imu_position':calibration['imu_position'],'imu_orientation':calibration['imu_orientation'],
        'topics':{'truth':'/drone0/ground_truth/odometry','lio':'/zav/super/odom'},
        'frames':{'truth':'mounted IMU pose/twist in isaac_world/mid360_imu; converted offline to body origin/world ENU','lio':'body origin; linear velocity in world ENU'},
        'completion':'recording','samples':{'truth':0,'lio':0},'invalid_samples':0}
    def capture_scene():
        if args.scenario not in ('mixed','dasan') or 'map_recipe' in metadata:
            return
        assets = ws / 'simulator/assets/super'
        recipe, geometry = assets / f'{args.scenario}_recipe.json', assets / f'{args.scenario}.json'
        if recipe.exists() and geometry.exists():
            recipe_data, geometry_data = json.loads(recipe.read_text()), json.loads(geometry.read_text())
            if recipe_data['generated_pcd_sha256'] == geometry_data['source_sha256']:
                metadata['map_recipe'] = recipe_data
                metadata['scene_geometry'] = geometry_data
    capture_scene()
    if args.mission:
        metadata['mission']=json.loads(args.mission.read_text())
        metadata['mission_path']=str(args.mission.resolve())
    (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    rclpy.init();node=Node('zav_super_flight_telemetry');state='UNKNOWN';finish=None;last_stamp=None
    file=(args.output/'samples.jsonl').open('w',buffering=65536)
    def odom(source,msg):
        nonlocal last_stamp
        p,q,v,w=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        expected=('isaac_world','mid360_imu')if source=='truth' else ('world','zav_body')
        if (msg.header.frame_id,msg.child_frame_id)!=expected:
            metadata['invalid_samples']+=1;return
        row={'source':source,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
             'state':state,'p':[p.x,p.y,p.z],'q':[q.x,q.y,q.z,q.w],
             'v':[v.x,v.y,v.z],'w':[w.x,w.y,w.z]}
        last_stamp=row['stamp'];file.write(json.dumps(row,allow_nan=True)+'\n');metadata['samples'][source]+=1
    def status(msg):
        nonlocal state,finish
        if state!=msg.data:
            file.write(json.dumps({'event':'state','state':msg.data,'stamp':last_stamp})+'\n')
        state=msg.data
        if state=='LANDED' and finish is None:finish=time.monotonic()+1.
    node.create_subscription(Odometry,'/zav/super/odom',lambda m:odom('lio',m),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/drone0/ground_truth/odometry',lambda m:odom('truth',m),qos_profile_sensor_data)
    node.create_subscription(String,'/zav/super/controller_status',status,10)
    def mission(msg):
        try:metadata['last_mission_status']=json.loads(msg.data)
        except ValueError:return
        if metadata.get('_last_phase')!=metadata['last_mission_status'].get('phase') or metadata.get('_last_index')!=metadata['last_mission_status'].get('reached_waypoints'):
            file.write(json.dumps({'event':'mission','stamp':last_stamp,**metadata['last_mission_status']})+'\n')
            metadata['_last_phase']=metadata['last_mission_status'].get('phase')
            metadata['_last_index']=metadata['last_mission_status'].get('reached_waypoints')
    node.create_subscription(String,'/zav/super/mission_status',mission,10)
    def goal(msg):
        p,q=msg.pose.position,msg.pose.orientation
        orientation=[q.x,q.y,q.z,q.w]
        file.write(json.dumps({'event':'lookahead_goal','stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'frame':msg.header.frame_id,'p':[p.x,p.y,p.z],
            'q':orientation if all(math.isfinite(v)for v in orientation)else None})+'\n')
    last_path=None
    def path(msg):
        nonlocal last_path
        vertices=[[p.pose.position.x,p.pose.position.y,p.pose.position.z,
                   p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]for p in msg.poses]
        signature=(msg.header.frame_id,repr(vertices))
        if signature==last_path:return
        last_path=signature
        file.write(json.dumps({'event':'global_path','stamp':last_stamp,'frame':msg.header.frame_id,'vertices':vertices})+'\n')
    node.create_subscription(PoseStamped,'/planning/click_goal',goal,10)
    def trajectory(msg):
        if not msg.type & PolynomialTrajectory.POSITION_TRAJ:return
        file.write(json.dumps({'event':'polynomial','stamp':last_stamp,'start':msg.start_wt_pos,
            'duration':list(msg.time_pos),'x':list(msg.coef_pos_x),'y':list(msg.coef_pos_y),'z':list(msg.coef_pos_z),
            'yaw_start':msg.start_wt_yaw,'yaw_duration':list(msg.time_yaw),'yaw':list(msg.coef_yaw)})+'\n')
    node.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',trajectory,qos_profile_sensor_data)
    node.create_subscription(NavPath,'/mission/global_path',path,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(.5,file.flush)
    node.get_logger().info(f'Flight recording: {args.output}; report after landing or launcher Ctrl+C')
    completed=False
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            if finish is not None and time.monotonic()>=finish:completed=True;break
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        file.close();metadata['last_controller_state']=state;metadata['completion']='LANDED' if state=='LANDED' else 'interrupted'
        capture_scene()
        (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if completed:
        from flight_metrics import analyze
        try:analyze(args.output)
        except Exception as exc:print(f'[SUPER telemetry] Report failed; raw data retained: {exc}',file=sys.stderr)

if __name__=='__main__':main()
