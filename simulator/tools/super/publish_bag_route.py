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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath, Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


def find_return_join(points,progress,position):
    """Only skip a blocked, planar out-and-back loop onto its nearby return.

    A sub-metre join cannot bypass a stair flight; all omitted vertices must
    remain on the same height band. SUPER still checks the new goal online.
    """
    arc=[0.]
    for a,b in zip(points[:-1],points[1:]):arc.append(arc[-1]+math.dist(a,b))
    begin=next((i for i,s in enumerate(arc)if s>=progress),len(points)-1)
    for i in range(begin,len(points)):
        if arc[i]-progress<5.:continue
        if math.dist(position,points[i])>1. or abs(position[2]-points[i][2])>.25:continue
        heights=[p[2]for p in points[begin:i+1]]
        if max(heights)-min(heights)>.65:continue
        return i,arc[i]-progress-math.dist(position,points[i])
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=('dasan','frontier'),default='dasan')
    parser.add_argument('--mission',type=Path)
    parser.add_argument('--replace-after-meters',type=float,default=0.,help='Test one live suffix replacement after this path progress')
    parser.add_argument('--land-at-end',action='store_true')
    parser.add_argument('--wait-at-end-seconds',type=float)
    parser.add_argument('--wait-at-viewpoint-seconds',type=float)
    parser.add_argument('--yaw-policy',choices=('mission','fixed'),default='mission',
                        help='mission: honor each viewpoint yaw (None uses travel direction); fixed: independent-yaw regression')
    args=parser.parse_args()
    if args.mission is None:args.mission=Path(__file__).resolve().parents[2]/f'assets/super/{args.scenario}_mission.json'
    mission=json.loads(args.mission.read_text())
    if args.wait_at_end_seconds is None:args.wait_at_end_seconds=float(mission.get('end_wait_seconds',2.))
    if args.wait_at_viewpoint_seconds is None:args.wait_at_viewpoint_seconds=float(mission.get('viewpoint_wait_seconds',.5))
    rclpy.init();node=Node('bag_exploration_route_source')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest_sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher=node.create_publisher(NavPath,'/mission/exploration_path',qos)
    correction_pub=node.create_publisher(String,'/mission/bag_frame_correction',10)
    landing=node.create_client(Trigger,'/argo/super/land')
    state={'mission':{},'controller':'','position':None,'speed':float('inf'),'stamp':None,
           'cloud':None,'cloud_stamp':None}
    alignment=None
    if mission.get('bag_map_alignment',False):
        from bag_map_alignment import BagMapAlignment,cloud_xyz
        from pcd_scene import read_pcd
        from sensor_msgs.msg import PointCloud2
        reference=Path(__file__).resolve().parents[2]/f'assets/super/{args.scenario}.pcd'
        import hashlib
        if hashlib.sha256(reference.read_bytes()).hexdigest()!=mission.get('map_sha256'):
            raise ValueError('Bag route and localization reference map hashes differ')
        alignment=BagMapAlignment(read_pcd(reference))
        def cloud(msg):
            if msg.header.frame_id!='world':return
            try:points=cloud_xyz(msg)
            except (ValueError,TypeError):return
            state['cloud']=points
            state['cloud_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        node.create_subscription(PointCloud2,'/argo/super/cloud_world',cloud,latest_sensor)
        if args.replace_after_meters>0:raise ValueError('Aligned bag chunks cannot use the suffix replacement test')
    def status(msg):
        try:state['mission']=json.loads(msg.data)
        except ValueError:pass
    def odom(msg):
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        state['position']=(p.x,p.y,p.z);state['speed']=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        state['stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    node.create_subscription(String,'/argo/super/mission_status',status,qos)
    node.create_subscription(String,'/argo/super/controller_status',lambda m:state.update(controller=m.data),1)
    node.create_subscription(Odometry,'/argo/super/odom',odom,latest_sensor)
    points=[mission['takeoff'],*mission['waypoints']];original=list(points)
    yaws=mission.get('yaw_waypoints',[None]*len(points))
    if args.yaw_policy=='fixed':yaws=[float(mission['spawn_yaw'])]*len(points)
    if len(yaws)!=len(points):raise ValueError('One optional yaw required per route vertex')
    original_yaws=list(yaws)
    pauses=mission.get('viewpoint_pause_indices',[])
    if any(type(i)is not int or i<=0 or i>=len(points)-1 for i in pauses) or pauses!=sorted(set(pauses)):
        raise ValueError('Viewpoint pause indices must be ordered unique interior route vertices')
    if pauses and args.replace_after_meters>0:raise ValueError('Use either viewpoint chunks or suffix replacement in one test')
    boundaries=[0,*pauses,len(points)-1];chunk=0;awaiting_revision=1
    points=original[boundaries[0]:boundaries[1]+1];yaws=original_yaws[boundaries[0]:boundaries[1]+1]
    active_global_points=list(points);active_global_yaws=list(yaws)
    arc=[0.]
    for a,b in zip(points[:-1],points[1:]):arc.append(arc[-1]+math.dist(a,b))
    published=False;replaced=False;last_publish=0.;settled=None;future=None;land_acknowledged=False
    alignment_pending=alignment is not None;alignment_wait=None;alignment_attempt=0.
    alignment_watch_revision=None;alignment_watch_progress=0.;alignment_watch_stamp=None
    alignment_recoveries=0
    correction_sequence=0;last_tracking_alignment=0.
    watch_revision=None;watch_progress=0.;watch_stamp=None
    def event(name,**values):print(json.dumps({'event':name,'wall_monotonic_s':time.monotonic(),**values}),flush=True)
    def drain_feedback():
        # Registration can take longer than one odometry interval. Consume the
        # latest ready samples before using position and matching route progress.
        for _ in range(16):rclpy.spin_once(node,timeout_sec=0.)
    event('source_ready',mission=str(args.mission),vertices=len(original),chunks=len(boundaries)-1,yaw_policy=args.yaw_policy,land_at_end=args.land_at_end)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            drain_feedback()
            controller=state['controller'];m=state['mission'];now=time.monotonic()
            if controller.startswith(('FAULT','STOPPED')) or m.get('phase')=='ABORTED':
                event('failed',controller=controller,mission=m);return 1
            if controller=='LANDED':event('landed',mission=m);return 0
            if future is not None and future.done() and not land_acknowledged:
                response=future.result();land_acknowledged=True
                event('land_service_response',success=response.success,message=response.message)
                if not response.success:return 1
            if controller not in ('READY','TRACKING') or future is not None:continue
            if alignment_pending:
                if alignment_wait is None:alignment_wait=now
                if (now-alignment_attempt>1. and state['cloud']is not None and
                    state['stamp']is not None and state['cloud_stamp']is not None and
                    abs(state['stamp']-state['cloud_stamp'])<.5 and state['speed']<.3):
                    alignment_attempt=now
                    result=alignment.register(state['cloud'],state['position'])
                    drain_feedback()
                    event('bag_map_alignment',chunk=chunk+1,sim_time_s=state['stamp'],**result)
                    if result['accepted']:
                        points,yaws=alignment.route_in_live_frame(points,yaws,state['position'])
                        alignment_pending=False
                if alignment_pending:
                    if now-alignment_wait>20. and landing.service_is_ready():
                        future=landing.call_async(Trigger.Request())
                        event('bag_map_alignment_failed_land_request',chunk=chunk+1)
                    continue
            progress=m.get('path_progress_m');revision=m.get('route_revision',0)
            if (alignment is not None and m.get('phase')=='RUNNING' and revision>=awaiting_revision and
                progress is not None and state['stamp']is not None):
                if revision!=alignment_watch_revision or progress>alignment_watch_progress+.15:
                    alignment_watch_revision=revision;alignment_watch_progress=progress
                    alignment_watch_stamp=state['stamp']
                stalled=(state['stamp']-alignment_watch_stamp>5. and state['speed']<.3 and alignment_recoveries<3)
                if state['speed']>=.3 and now-last_tracking_alignment>1. and state['cloud']is not None and abs(state['stamp']-state['cloud_stamp'])<.3:
                    import numpy as np
                    last_tracking_alignment=now
                    old_rotation=alignment.rotation.copy();old_translation=alignment.translation.copy()
                    result=alignment.register(state['cloud'],state['position'])
                    if result['accepted']:
                        D=alignment.rotation.T@old_rotation;dt=alignment.rotation.T@(old_translation-alignment.translation)
                        points=(np.asarray(points)@D.T+dt).tolist()
                        correction_sequence+=1
                        correction_pub.publish(String(data=json.dumps(dict(revision=revision,sequence=correction_sequence,rotation=D.tolist(),translation=dt.tolist()))))
                        event('bag_frame_corrected',sim_time_s=state['stamp'],revision=revision,sequence=correction_sequence,**result)
                # A periodic path replacement while moving resets the follower
                # and can command already passed vertices. Keep its ordered
                # progress intact; only re-anchor a genuinely stopped recovery.
                if (stalled and state['cloud']is not None and
                    abs(state['stamp']-state['cloud_stamp'])<.5 and
                    m.get('path_progress_stamp_sim')is not None and
                    abs(state['stamp']-m['path_progress_stamp_sim'])<.5 and now-alignment_attempt>1.):
                    alignment_attempt=now
                    result=alignment.register(state['cloud'],state['position'])
                    drain_feedback()
                    fresh=state['mission']
                    if (fresh.get('phase')!='RUNNING' or fresh.get('route_revision')!=revision or
                        fresh.get('path_progress_stamp_sim')is None or
                        not abs(state['stamp']-fresh['path_progress_stamp_sim'])<.5):continue
                    m=fresh;progress=m['path_progress_m']
                    event('bag_map_alignment_recovery',chunk=chunk+1,sim_time_s=state['stamp'],**result)
                    if result['accepted']:
                        segment=int(m['reached_waypoints'])
                        if not 0<=segment<len(active_global_points)-1:continue
                        active_global_points,active_global_yaws=alignment.remaining_route(
                            active_global_points,active_global_yaws,segment,state['position'])
                        points,yaws=alignment.route_in_live_frame(active_global_points,active_global_yaws,state['position'])
                        awaiting_revision=revision+1;last_publish=0.;settled=None
                        alignment_recoveries+=1
                        alignment_watch_stamp=state['stamp']
                        event('aligned_remaining_path_republished',chunk=chunk+1,old_revision=revision,
                              remaining_vertices=len(points),recovery=alignment_recoveries,
                              source_segment=segment,source_position=state['position'],
                              progress_feedback_age_s=state['stamp']-m['path_progress_stamp_sim'],
                              next_vertex=points[1])
            if (mission.get('allow_blocked_loop_skip',False) and m.get('phase')=='RUNNING' and
                revision>=awaiting_revision and progress is not None and state['stamp']is not None):
                if revision!=watch_revision or progress>watch_progress+.15:
                    watch_revision=revision;watch_progress=progress;watch_stamp=state['stamp']
                elif state['stamp']-watch_stamp>8. and state['speed']<.3:
                    join=find_return_join(points,progress,state['position'])
                    if join is not None:
                        index,removed=join
                        points=[state['position'],*points[index:]];yaws=[yaws[index],*yaws[index:]]
                        awaiting_revision=revision+1;last_publish=0.;settled=None;watch_stamp=state['stamp']
                        event('blocked_corridor_loop_skipped',chunk=chunk+1,old_revision=revision,
                              old_progress_m=progress,omitted_arc_m=removed,join=points[1])
            if args.replace_after_meters>0 and not replaced and m.get('path_progress_m',0.) is not None and m.get('path_progress_m',0.)>=args.replace_after_meters:
                progress=m['path_progress_m'];index=next((i for i,s in enumerate(arc)if s>progress+.5),len(original)-1)
                points=[state['position'],*original[index:]];replaced=True
                yaws=[original_yaws[index],*original_yaws[index:]]
                event('replace_route',old_progress_m=progress,vertices=len(points));last_publish=0.
            if not published or last_publish==0.:
                msg=NavPath();msg.header.frame_id='world';msg.header.stamp=node.get_clock().now().to_msg()
                for point,yaw in zip(points,yaws):
                    pose=PoseStamped();pose.header=msg.header
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
                    if yaw is not None:
                        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
                    msg.poses.append(pose)
                publisher.publish(msg);last_publish=now
                if not published:event('initial_route_published');published=True
            if m.get('phase')=='WAITING_PATH' and state['speed']<.3 and m.get('route_revision',0)>=awaiting_revision:
                if settled is None:settled=now;event('route_complete_waiting_for_next',chunk=chunk+1,chunks=len(boundaries)-1,mission=m)
                if chunk<len(boundaries)-2 and now-settled>args.wait_at_viewpoint_seconds:
                    chunk+=1;awaiting_revision=m['route_revision']+1
                    points=original[boundaries[chunk]:boundaries[chunk+1]+1]
                    yaws=original_yaws[boundaries[chunk]:boundaries[chunk+1]+1]
                    active_global_points=list(points);active_global_yaws=list(yaws)
                    alignment_pending=alignment is not None;alignment_wait=None;alignment_attempt=0.
                    alignment_recoveries=0
                    settled=None;last_publish=0.
                    event('next_viewpoint_chunk',chunk=chunk+1,chunks=len(boundaries)-1,start_vertex=boundaries[chunk],end_vertex=boundaries[chunk+1])
                    continue
                if args.land_at_end and now-settled>args.wait_at_end_seconds and landing.service_is_ready():
                    future=landing.call_async(Trigger.Request());event('explicit_test_driver_land_request')
            else:settled=None
    except KeyboardInterrupt:event('interrupted');return 130
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
