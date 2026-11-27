#!/usr/bin/env python3
"""Real SUPER + OMMPC supervisor regression with scripted odometry, not flight dynamics.
Run in a device-free --network none container, ROS_DOMAIN_ID=218.
"""
import json, os, socket, struct, subprocess, tempfile, time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(name=='lo' for _,name in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from zav_super.srv import GoForward
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly


def run():
    rclpy.init();node=rclpy.create_node('terminal_settling_regression')
    odom=node.create_publisher(Odometry,'/argo/super/odom',10)
    cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
    bridge=node.create_publisher(String,'/zav_real/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    status=[];plans=[];beats=[];goals=[];planner_events=[];packet_requests={};retagged=[]
    node.create_subscription(String,"/zav/super/planner_status",lambda m:planner_events.append(json.loads(m.data)),10)
    node.create_subscription(PoseStamped,'/zav/super/goal',goals.append,10)
    goal_pub=node.create_publisher(PoseStamped,'/zav/super/goal',10)
    cancel=node.create_publisher(String,'/zav/super/planner_cancel',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(String,'/zav/super/controller_status',lambda m:status.append(json.loads(m.data)),10)
    def on_plan(m):
        now=time.monotonic()
        if m.type & Poly.POSITION_TRAJ:
            key=(m.start_wt_pos,m.start_wt_yaw,tuple(m.time_pos),tuple(m.time_yaw),tuple(m.coef_pos_x),tuple(m.coef_pos_y),tuple(m.coef_pos_z),tuple(m.coef_yaw))
            request=json.loads(m.debug_info)['goal_request_stamp_ns']
            if key in packet_requests and packet_requests[key]!=request:retagged.append((packet_requests[key],request))
            packet_requests[key]=request;plans.append((now,m))
        if m.type & Poly.HEART_BEAT: beats.append(now)
    node.create_subscription(Poly,'/planning_cmd/poly_traj',on_plan,qos_profile_sensor_data)
    forward=node.create_client(GoForward,'/zav/super/go_forward')
    rth=node.create_client(Trigger,'/zav/super/return_home')
    state={'p':np.array([0.,0.,2.]),'v':np.zeros(3),'armed':False,'landed':True,'mode':[3,0]}
    points=[]
    for x in np.arange(-2.,12.1,.25):
        for y in np.arange(-4.,4.1,.25): points.extend([(x,y,0.,1.),(x,y,5.,1.)])
    for x in np.arange(-2.,12.1,.25):
        for z in np.arange(0.,5.1,.25): points.extend([(x,-4.,z,1.),(x,4.,z,1.)])
    for y in np.arange(-4.,4.1,.25):
        for z in np.arange(0.,5.1,.25): points.extend([(-2.,y,z,1.),(12.,y,z,1.)])
    raw=b''.join(struct.pack('<ffff',*p) for p in points)
    scan=PointCloud2(height=1,width=len(points),point_step=16,row_step=16*len(points),is_dense=True,data=raw)
    scan.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z','intensity'))]
    scan.header.frame_id='world';last_cloud=0.
    def publish():
        nonlocal last_cloud
        stamp=node.get_clock().now().to_msg();m=Odometry();m.header.frame_id='world';m.child_frame_id='argo_body';m.header.stamp=stamp
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,state['p'])
        m.pose.pose.orientation.w=1.
        m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=map(float,state['v']);odom.publish(m)
        if time.monotonic()-last_cloud>.1:
            scan.header.stamp=stamp;cloud.publish(scan);last_cloud=time.monotonic()
        bridge.publish(String(data=json.dumps({'ready':True,'latched':False,'armed':state['armed'],
            'landed':state['landed'],'landed_fresh':True,'offboard':{'source':'super','stopped':False,'fc_mode':state['mode'],'auto_land_disarm_s':2.}})))
    node.create_timer(.02,publish)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def wait(predicate,seconds=8.):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.01)
        assert predicate(),('Timeout',status[-1] if status else None)
    def signature(m):return (m.start_wt_pos,tuple(m.time_pos))
    def last_plan_after(index,requested_after):
        # The transport may still deliver an old committed trajectory while a
        # new goal is queued. Apply the controller's epoch/freshness gates;
        # following that old packet would teleport scripted odometry.
        def valid(m):
            stamp=json.loads(m.debug_info)['goal_request_stamp_ns']
            return stamp>=requested_after and m.start_wt_pos>=stamp/1e9-.1 and m.start_wt_pos+sum(m.time_pos)>node.get_clock().now().nanoseconds/1e9
        wait(lambda:any(valid(m) for _,m in plans[index:]))
        return next(m for _,m in reversed(plans[index:]) if valid(m))
    def follow(m):
        end=m.start_wt_pos+sum(m.time_pos)
        while node.get_clock().now().nanoseconds/1e9<end:
            # Follow only the committed polynomial; no output feeds a real plant.
            t=max(0.,node.get_clock().now().nanoseconds/1e9-m.start_wt_pos)
            for i,duration in enumerate(m.time_pos):
                if t<=duration:break
                t-=duration
            coeff=[np.asarray(c[i*8:(i+1)*8]) for c in (m.coef_pos_x,m.coef_pos_y,m.coef_pos_z)]
            state['p']=np.array([np.polyval(c,t) for c in coeff])
            state['v']=np.array([np.polyval(np.polyder(c),t) for c in coeff])
            # Approach the endpoint with a smooth 15cm residual during the last second.
            progress=np.clip(1.-(end-node.get_clock().now().nanoseconds/1e9),0.,1.)
            state['p'][0]+=.15*float(progress)
            spin(.015)
        goal=np.array(json.loads(m.debug_info)['accepted_goal'])
        state['p']=goal+np.array([.15,0.,0.]);state['v']=np.array([.2,0.,0.])
        return goal,end
    share=Path(get_package_share_directory('zav_super'))
    config=yaml.safe_load((share/'config/common.yaml').read_text());config['super_planner']['visualization_en']=False
    config['fsm']['click_goal_topic']='/zav/super/goal'
    # Synthetic room is 5m high; isolate fixture bounds from the 1.5m vehicle ceiling.
    config['rog_map']['virtual_ceil_height']=6.
    config['rog_map']['ros_callback'].update(cloud_topic='/argo/super/cloud_world',odom_topic='/argo/super/odom')
    controller=dict(config['argo_ommpc_px4']['ros__parameters'])
    for key in ('use_px4_landing','allow_terminal_hold'):controller.pop(key,None)
    for key in ('goal_position_tolerance','goal_speed_tolerance','goal_snap_max_distance'):controller[key]=config['fsm'][key]
    controller['execute']=True
    processes=[];handles=[]
    with tempfile.TemporaryDirectory(prefix='terminal-regression-') as temp:
        temp=Path(temp);(temp/'common.yaml').write_text(yaml.safe_dump(config));(temp/'controller.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':controller}}))
        try:
            for package,executable,args in [('super_planner','fsm_node',['-p',f'config_name:={temp}/common.yaml']),('zav_super','super_ommpc',['--params-file',str(temp/'controller.yaml')])]:
                log=(temp/f'{executable}.log').open('w+');handles.append(log)
                binary=Path(get_package_prefix(package))/'lib'/package/executable
                processes.append(subprocess.Popen([str(binary),'--ros-args',*args],stdout=log,stderr=subprocess.STDOUT))
            wait(lambda:status and status[-1]['phase']=='PRESTREAM' and forward.service_is_ready());spin(3.)
            state.update(armed=True,landed=False,mode=[6,0]);wait(lambda:status[-1]['phase']=='HOVER' and status[-1]['mission']['home_valid'])
            index=len(plans);requested_after=node.get_clock().now().nanoseconds;future=forward.call_async(GoForward.Request(distance_m=2.));wait(future.done);assert future.result().success
            m=last_plan_after(index,requested_after);goal,end=follow(m);key=signature(m);index=len(plans);b0=len(beats)
            spin(1.2)
            assert status[-1]['phase']=='ENDPOINT_SETTLING',status[-1]
            assert not status[-1]['mission']['complete']
            assert all(signature(x)==key for _,x in plans[index:]),'Unexpected early recovery trajectory'
            assert len(beats)>b0+10,'Heartbeat stopped during convergence'
            state['p']=goal+np.array([.08,0.,0.]);state['v']=np.array([.12,0.,0.]);spin(.35)
            assert status[-1]['phase']=='ENDPOINT_SETTLING' and not status[-1]['mission']['complete'],status[-1]
            state['v']=np.array([.08,0.,0.]);wait(lambda:status[-1]['mission']['complete'],3.)
            assert status[-1]['phase']=='ENDPOINT_HOLD'
            print('PASS: real planner waits at 15cm/0.2m/s, heartbeat continues; 8cm/0.12m/s is not arrival; 8cm/0.08m/s +1s completes',flush=True)
            # A second leg remains 15cm short: wait must expire and allow recovery.
            state['v']=np.zeros(3);index=len(plans);requested_after=node.get_clock().now().nanoseconds;future=forward.call_async(GoForward.Request(distance_m=1.));wait(future.done);assert future.result().success
            m=last_plan_after(index,requested_after);goal,end=follow(m);key=signature(m);index=len(plans)
            state['v']=np.zeros(3);spin(2.4)
            assert status[-1]['phase']=='ENDPOINT_SETTLING',status[-1]
            assert all(signature(x)==key for _,x in plans[index:]),'Recovery happened before the 3s deadline'
            wait(lambda:any(signature(x)!=key for _,x in plans[index:]),3.)
            recovery=next(x for _,x in plans[index:] if signature(x)!=key)
            assert recovery.start_wt_pos>=end+2.9,(recovery.start_wt_pos,end)
            print('PASS: no correction during 3s convergence window; timeout generates recovery from measured state',flush=True)
            # Follow that recovery to another residual, then preempt convergence.
            goal,end=follow(recovery);spin(.3);assert status[-1]['phase']=='ENDPOINT_SETTLING',status[-1]
            index=len(plans);future=rth.call_async(Trigger.Request());wait(future.done);assert future.result().success,future.result().message
            wait(lambda:any(x.start_wt_pos>end and json.loads(x.debug_info)['goal_request_stamp_ns']!=json.loads(recovery.debug_info)['goal_request_stamp_ns'] for _,x in plans[index:]),5.)
            assert status[-1]['mission']['kind']=='rth' and not status[-1]['mission']['complete'],status[-1]
            print('PASS: new RTH goal preempts terminal convergence and receives a newly correlated trajectory',flush=True)
            assert not any(s['phase'] in ('SOLVER_FAILURE','SUPER_EMERGENCY','ODOMETRY_JUMP','PLANNER_TIMEOUT') for s in status)
            # Stop the synthetic controller, then exercise the actual planner's
            # cancellation epoch independently of its heartbeat consumer.
            processes[1].terminate();processes[1].wait(timeout=5.)
            stale=goals[-1];stamp=node.get_clock().now().nanoseconds
            cancel.publish(String(data=json.dumps({'stamp_ns':stamp,'reason':'OFFLINE_CANCEL_TEST'})))
            spin(.5);count=len(plans);b0=len(beats);spin(1.2)
            assert len(plans)==count and len(beats)==b0,'cancelled planner continued old-goal output'
            assert any(e['event']=='CANCELLED' for e in planner_events),'missing planner cancellation acknowledgement'
            goal_pub.publish(stale);spin(.4)
            assert len(plans)==count,'stale goal revived cancelled mission'
            fresh=PoseStamped();fresh.header.frame_id='world';fresh.header.stamp=node.get_clock().now().to_msg()
            fresh.pose.position.x=float(state['p'][0]-.8);fresh.pose.position.y=float(state['p'][1]);fresh.pose.position.z=float(state['p'][2]);fresh.pose.orientation.w=1.
            goal_pub.publish(fresh)
            expected=fresh.header.stamp.sec*10**9+fresh.header.stamp.nanosec
            wait(lambda:any(json.loads(x.debug_info)['goal_request_stamp_ns']==expected for _,x in plans[count:]),5.)
            assert not retagged,('cached trajectory relabelled as a new goal response',retagged)
            print('PASS: actual SUPER stops old goal/replanning/heartbeat after cancel; stale goal rejected; newer mission resumes; cached packets keep their request identity',flush=True)

        finally:
            for p in processes:
                p.terminate()
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            for log in handles:
                log.seek(0);data=log.read();print('\nNODE LOG TAIL\n'+data[-6000:]);log.close()
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':run()
