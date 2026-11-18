#!/usr/bin/env python3
"""Bag-derived upstream route surrogate, not EPIC and not a timed trajectory.

Replaces the future exploration publisher in GUI integration tests. Only the
explicit --land-at-end test driver asks for landing; the route consumer never
lands automatically on completion of a live exploration path.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath, Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super/dasan_mission.json')
    parser.add_argument('--replace-after-meters',type=float,default=0.,help='Test one live suffix replacement after this path progress')
    parser.add_argument('--land-at-end',action='store_true')
    parser.add_argument('--wait-at-end-seconds',type=float,default=2.)
    args=parser.parse_args();mission=json.loads(args.mission.read_text())
    rclpy.init();node=Node('bag_exploration_route_source')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=node.create_publisher(NavPath,'/mission/exploration_path',qos)
    landing=node.create_client(Trigger,'/zav/super/land')
    state={'mission':{},'controller':'','position':None,'speed':float('inf')}
    def status(msg):
        try:state['mission']=json.loads(msg.data)
        except ValueError:pass
    def odom(msg):
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        state['position']=(p.x,p.y,p.z);state['speed']=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    node.create_subscription(String,'/zav/super/mission_status',status,qos)
    node.create_subscription(String,'/zav/super/controller_status',lambda m:state.update(controller=m.data),10)
    node.create_subscription(Odometry,'/zav/super/odom',odom,10)
    points=[mission['takeoff'],*mission['waypoints']];original=list(points)
    arc=[0.]
    for a,b in zip(points[:-1],points[1:]):arc.append(arc[-1]+math.dist(a,b))
    yaw=float(mission['spawn_yaw']);published=False;replaced=False;last_publish=0.;settled=None;future=None;land_acknowledged=False
    def event(name,**values):print(json.dumps({'event':name,'wall_monotonic_s':time.monotonic(),**values}),flush=True)
    event('source_ready',mission=str(args.mission),vertices=len(points),yaw_policy='constant spawn yaw, independent of travel direction',land_at_end=args.land_at_end)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            controller=state['controller'];m=state['mission'];now=time.monotonic()
            if controller.startswith(('FAULT','STOPPED')) or m.get('phase')=='ABORTED':
                event('failed',controller=controller,mission=m);return 1
            if controller=='LANDED':event('landed',mission=m);return 0
            if future is not None and future.done() and not land_acknowledged:
                response=future.result();land_acknowledged=True
                event('land_service_response',success=response.success,message=response.message)
                if not response.success:return 1
            if controller not in ('READY','TRACKING') or future is not None:continue
            if args.replace_after_meters>0 and not replaced and m.get('path_progress_m',0.) is not None and m.get('path_progress_m',0.)>=args.replace_after_meters:
                progress=m['path_progress_m'];index=next((i for i,s in enumerate(arc)if s>progress+.5),len(original)-1)
                points=[state['position'],*original[index:]];replaced=True
                event('replace_route',old_progress_m=progress,vertices=len(points));last_publish=0.
            if not published or now-last_publish>1.:
                msg=NavPath();msg.header.frame_id='world';msg.header.stamp=node.get_clock().now().to_msg()
                for point in points:
                    pose=PoseStamped();pose.header=msg.header
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
                    pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);msg.poses.append(pose)
                publisher.publish(msg);last_publish=now
                if not published:event('initial_route_published');published=True
            if m.get('phase')=='WAITING_PATH' and state['speed']<.3:
                if settled is None:settled=now;event('route_complete_waiting_for_next',mission=m)
                if args.land_at_end and now-settled>args.wait_at_end_seconds and landing.service_is_ready():
                    future=landing.call_async(Trigger.Request());event('explicit_test_driver_land_request')
            else:settled=None
    except KeyboardInterrupt:event('interrupted');return 130
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
