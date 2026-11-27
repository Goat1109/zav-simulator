#!/usr/bin/env python3
"""Device-free ROS contract test; synthetic EPIC/FC/odometry, no flight dynamics.
Run in a network-disabled container with no device mounts, domain 218.
"""
import json,math,os,socket,struct,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(name=='lo' for _,name in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import rclpy,yaml
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField,Joy
from std_msgs.msg import String,Bool
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly

frame_mode=os.environ.get('EPIC_FRAME_TEST')=='1'
from zav_real_control.epic_frame import ProfileFrame
profile_frame=ProfileFrame([0.,0.,.2],math.pi/2,[0.,0.,.8],0.)
rclpy.init();node=rclpy.create_node('offline_epic_contract_test')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
odom=node.create_publisher(Odometry,'/argo/super/odom',10)
cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
bridge=node.create_publisher(String,'/zav_real/status',qos)
marker=node.create_publisher(Marker,'/epic/state',10)
guide=node.create_publisher(NavPath,'/epic/profile_path' if frame_mode else '/mission/exploration_path',1)
poly=node.create_publisher(Poly,'/planning_cmd/poly_traj',10)
done=node.create_publisher(Bool,'/mission/exploration_complete',1)
status=[];epic=[];goals=[];triggers=[];cancels=[]
feedback=[]
node.create_subscription(String,'/zav/super/trajectory_feedback',lambda m:feedback.append(json.loads(m.data)),10)
node.create_subscription(String,"/zav/super/planner_cancel",lambda m:cancels.append(json.loads(m.data)),qos)
node.create_subscription(String,'/zav/super/controller_status',lambda m:status.append(json.loads(m.data)),10)
node.create_subscription(String,'/zav/epic/status',lambda m:epic.append(json.loads(m.data)),qos)
node.create_subscription(PoseStamped,'/zav/super/goal',goals.append,10)
node.create_subscription(NavPath,'/epic/trigger',triggers.append,10)
entries=[]
node.create_subscription(NavPath,'/zav/super/entry_goal',entries.append,10)
patrol=node.create_client(Trigger,'/zav/super/patrol')
s={'p':[0.,0.,.8],'mode':[3,0],'rc':1500,'rc_fresh':True,'battery':90,'fsm':'WAIT_TRIGGER','emit_fsm':True,'armed':True,'hover_ready':True,'landed':False,'v':0.}

sticks=node.create_publisher(Joy,'/zav_real/pilot_sticks',1)

def publish():
    stamp=node.get_clock().now().to_msg()
    if s.get('axes') is not None:
        stick=Joy(axes=s['axes']);stick.header.stamp=stamp;sticks.publish(stick)
    o=Odometry();o.header.frame_id='world';o.child_frame_id='argo_body';o.header.stamp=stamp
    o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=s['p'];o.pose.pose.orientation.w=1.;o.twist.twist.linear.x=s['v'];odom.publish(o)
    c=PointCloud2();c.header=o.header;c.width=c.height=1;c.point_step=c.row_step=12;c.data=struct.pack('<fff',0.,0.,0.)
    c.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')];cloud.publish(c)
    bridge.publish(String(data=json.dumps({'ready':True,'latched':False,'armed':s['armed'],'landed':s['landed'],'landed_fresh':True,'checks':{'heartbeat':True},
        'telemetry':{'rc7':{'pwm':s['rc'],'age_s':0.,'fresh':s['rc_fresh']},'battery_fresh':True,
                     'battery':{'remaining_pct':s['battery'],'charge_state':1}},
        'offboard':{'rc_ground_stage':s.get('ground_stage','IDLE'),'source':'super','fc_mode':s['mode'],'hover_ready':s['hover_ready'],'stopped':False,'auto_land_disarm_s':2.}})))
    if s['emit_fsm']:
        m=Marker();m.header=o.header;m.text=s['fsm'];marker.publish(m)
    beat=Poly();beat.header=o.header;beat.type=Poly.HEART_BEAT;poly.publish(beat)
node.create_timer(.02,publish)

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def wait(predicate,seconds=5.):
    end=time.monotonic()+seconds
    while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.01)
    assert predicate(),('timeout',status[-1:] ,epic[-1:])
def start():
    s['fsm']='WAIT_TRIGGER';s['rc']=1000;spin(.5);s['rc']=1500
    wait(lambda:epic[-1]['active']);s['fsm']='PLAN_TRAJ';spin(.1)
def send_path(points,frame='world',stale=False):
    m=NavPath();m.header.frame_id=frame;m.header.stamp=node.get_clock().now().to_msg()
    if stale:m.header.stamp.sec-=5
    for p in points:
        if frame_mode:p=profile_frame.forward(p)
        pose=PoseStamped();pose.header=m.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p
        pose.pose.orientation.w=1.
        if frame_mode:
            q=profile_frame.quaternion((1.,0.,0.,0.));pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z=q
        m.poses.append(pose)
    guide.publish(m);spin(.15)
def plan(p,excursion=False,request=None,duration=.2,linear=False):
    m=Poly();m.header.frame_id='world';m.header.stamp=node.get_clock().now().to_msg()
    m.type=Poly.POSITION_TRAJ|Poly.YAW_TRAJ|Poly.HEART_BEAT;m.order_pos=m.order_yaw=7
    m.piece_num_pos=m.piece_num_yaw=1;m.time_pos=m.time_yaw=[duration]
    m.start_wt_pos=m.start_wt_yaw=node.get_clock().now().nanoseconds/1e9
    m.coef_pos_x=[0.]*7+[p[0]];m.coef_pos_y=[0.]*7+[p[1]];m.coef_pos_z=[0.]*7+[p[2]];m.coef_yaw=[0.]*8
    if linear:m.coef_pos_x=[0.]*6+[p[0]/duration,0.]
    if excursion:m.coef_pos_x=[0.]*5+[-2000.,400.,p[0]] # inside endpoints, outside midpoint
    stamp=(request or goals[-1]).header.stamp
    m.debug_info=json.dumps({'goal_request_stamp_ns':stamp.sec*10**9+stamp.nanosec,'accepted_goal':p});poly.publish(m)
def arrive(p):
    original=s['p'][:]
    for k in range(1,41):s['p']=[a+(b-a)*k/40 for a,b in zip(original,p)];spin(.02)
    spin(1.3)

binary=str(Path(get_package_prefix('zav_super'))/'lib/zav_super/super_ommpc')
with tempfile.TemporaryDirectory() as temp:
    px4=os.environ.get('TRACKING_BACKEND')=='px4'
    params={'tracking_backend':'px4' if px4 else 'ommpc','rc_ground_hover_thrust':.31,'planner_response_timeout_s':1.0,'execute':True,'epic_enabled':True,'epic_profile':'gym','epic_bounds_lower':[-10.,-10.,0.],
            'epic_bounds_upper':[10.,10.,2.],'epic_flight_min':.3,'epic_flight_max':1.5}
    if frame_mode:
        params['epic_frame_required']=True
        params['epic_flight_min']=.1  # synthetic ground anchor maps WORLD .8 to PROFILE .2
    cfg=Path(temp)/'params.yaml';cfg.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
    with open(Path(temp)/'controller.log','w+') as log:
        adapter=None
        if frame_mode:
            adapter_binary=str(Path(get_package_prefix('zav_real_control'))/'lib/zav_real_control/epic_frame_adapter')
            adapter=subprocess.Popen([adapter_binary,'--ros-args','-p','reference_position:=[0.0,0.0,0.2]','-p','reference_yaw_deg:=90.0'],stdout=log,stderr=subprocess.STDOUT)
        proc=subprocess.Popen([binary,'--ros-args','--params-file',str(cfg)],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:status and epic and status[-1]['phase']=='PRESTREAM');spin(.4)
            assert not triggers and not epic[-1]['active']
            if frame_mode:
                assert epic[-1]['blocked_by']=='GROUND_ANCHOR_REQUIRED','airborne launch unexpectedly acquired anchor'
                s['armed']=False;s['landed']=True;wait(lambda:epic[-1]['blocked_by']!='GROUND_ANCHOR_REQUIRED')
                s['armed']=True;s['landed']=False;spin(.3)
            if not frame_mode:
                s['armed']=False;s['landed']=True;s['p'][2]=.4
                wait(lambda:status[-1]['mission'].get('rc_ground_seed')==.31)
                s['armed']=True;spin(.2);assert status[-1]['phase']=='PRESTREAM'
                assert status[-1]['mission']['kind']=='none','Position arm started OMMPC takeoff'
                s['armed']=False;spin(.3);s['ground_stage']='PILOT_ARM'
                wait(lambda:status[-1]['phase']=='PRESTREAM')
                assert status[-1]['mission']['kind']=='none','RC8 selected without arm started climb'
                s['armed']=True;s['ground_stage']='PRESTREAM';spin(1.3)
                assert status[-1]['mission']['kind']=='none'
                s['mode']=[6,0];s['ground_stage']='PX4_HOVER';s['landed']=False
                arrive([0.,0.,.8]);assert status[-1]['phase']=='PRESTREAM'
                s['ground_stage']='HANDOFF';wait(lambda:status[-1]['phase']=='HOVER')
                assert status[-1]['mission']['px4_hover_acquired'] and abs(status[-1]['hold_target'][2]-.8)<1e-5
                s['ground_stage']='COMPLETE'
                print('PASS: PX4 owns ascent and the selected tracker acquires the measured 0.4 m hover',flush=True)
            s['mode']=[6,0];wait(lambda:status[-1]['phase']=='HOVER');spin(.5)
            assert not triggers,'middle at startup must not trigger on mode readiness'
            if not frame_mode:
                # Sticks must be centered once in hover. Unexpectedly held sticks never start motion.
                s['axes']=[1.,0.,0.,0.];spin(.3)
                assert not status[-1]['pilot_ready'] and status[-1]['phase']=='HOVER'
                s['axes']=[0.]*4;wait(lambda:status[-1]['pilot_ready'])
                s['rc']=1000;s['axes']=[1.,-.5,.5,-.5];spin(.45)
                x,y,z,yaw=status[-1]['hold_target']
                assert status[-1]['phase']=='PILOT_STICKS' and x>0 and y<0 and z>.8 and yaw<0
                s['axes']=None;spin(.35)
                assert status[-1]['phase']=='HOVER' and not status[-1]['pilot_ready']
                s['axes']=[0.]*4;wait(lambda:status[-1]['pilot_ready'])
                s['axes']=[1.,0.,0.,0.];s['hover_ready']=False;s['v']=.4;s['rc']=1500
                wait(lambda:epic[-1]['reason']=='BRAKING_FOR_EPIC')
                wait(lambda:status[-1]['phase']=='HOVER')
                spin(.25);assert not triggers and not status[-1]['pilot_ready']
                # Releasing RC7 during braking must discard the pending request.
                s['rc']=1000;wait(lambda:not epic[-1]['active'])
                s['v']=0.;s['hover_ready']=True;spin(.4);assert not triggers
                s['axes']=[0.]*4;wait(lambda:status[-1]['pilot_ready'])
                s['axes']=[1.,0.,0.,0.];s['v']=.4;s['hover_ready']=False;s['rc']=1500
                wait(lambda:epic[-1]['reason']=='BRAKING_FOR_EPIC')
                s['v']=0.;s['hover_ready']=True;wait(lambda:len(triggers)==1)
                s['fsm']='PLAN_TRAJ';spin(.3)
                assert status[-1]['phase']=='HOVER' and len(triggers)==1 and not status[-1]['pilot_ready']
                s['rc']=1000;wait(lambda:not epic[-1]['active']);spin(.3)
                assert status[-1]['phase']=='HOVER' and not status[-1]['pilot_ready']
                s['axes']=[0.]*4;wait(lambda:status[-1]['pilot_ready'])
                s['axes']=[1.,0.,0.,0.];wait(lambda:status[-1]['phase']=='PILOT_STICKS')
                s['mode']=[3,0];wait(lambda:status[-1]['phase']=='PRESTREAM')
                assert not status[-1]['pilot_ready']
                s['mode']=[6,0];spin(.4);assert status[-1]['phase']=='HOVER' and not status[-1]['pilot_ready']
                s['axes']=None;s['fsm']='WAIT_TRIGGER';triggers.clear()
                print('PASS: stick axes, neutral interlock, sample loss, RC7 brake and trigger, neutral re-entry, Position takeover',flush=True)
                s['rc']=1000;arrive([0.,0.,-.39]);s['rc']=1500;spin(.45)
                assert not entries and not triggers and epic[-1]['blocked_by']=='OUTSIDE_ENTRY_ENVELOPE'
                s['rc']=1000;arrive([0.,0.,.2]);s['rc']=1500
                wait(lambda:len(entries)==1)
                assert not triggers and not goals
                initial,target=entries[0].poses
                assert abs(initial.pose.position.z-.2)<1e-6 and .33<target.pose.position.z<1.47
                p=[target.pose.position.x,target.pose.position.y,target.pose.position.z]
                plan(p,request=target);spin(.35);assert not triggers
                arrive(p);wait(lambda:len(triggers)==1)
                s['rc']=1000;wait(lambda:not epic[-1]['active'])
                triggers.clear();feedback.clear();s['p'][2]=.8;spin(.4)
                print('PASS: hard floor exclusion, single RC7 edge for entry and automatic EPIC trigger after settlement',flush=True)

            s['hover_ready']=False;s['rc']=1000;spin(.5);s['rc']=1500;spin(.5)
            assert not triggers,'FC Offboard alone is insufficient without bridge hover readiness'
            s['hover_ready']=True;spin(.5);assert not triggers,'rejected switch edge must be repeated'
            start();assert len(triggers)==1
            points=[[float(x),0.,.8] for x in range(7)]
            send_path(points)
            wait(lambda:len(goals)==1);assert status[-1]['mission']['kind']=='epic'
            assert abs(goals[-1].pose.position.x-4.)<1e-6,'did not preview across collinear waypoints'
            assert abs(goals[-1].pose.orientation.z)<1.e-8,'guide yaw failed inverse transform'
            future=patrol.call_async(Trigger.Request());wait(future.done);assert not future.result().success
            first=goals[-1];plan([4.,0.,.8],excursion=True);spin(.12)
            assert status[-1]['phase']=='PLANNING','curve crossing bounds was accepted'
            assert feedback and feedback[-1]['event']=='REJECTED'
            assert feedback[-1]['goal_request_stamp_ns']==first.header.stamp.sec*10**9+first.header.stamp.nanosec
            plan([4.,0.,.8],duration=10.,linear=px4);wait(lambda:not epic[-1]['pending_plan'])
            if px4:
                wait(lambda:status[-1].get('px4_reference',[0]*7)[4]==.4)
                ref=status[-1]['px4_reference']
                assert 0.<ref[0]<1. and ref[2]==.8 and status[-1]['solver_ms']==0.
                print('PASS: PX4 receives an intermediate polynomial sample and velocity feedforward',flush=True)
            first_commit=epic[-1]['committed_request_stamp_ns'];assert first_commit>0
            s['fsm']='EXEC_TRAJ';s['v']=.5
            for k in range(1,7):s['p']=[k*.1,0.,.8];spin(.025)
            wait(lambda:len(goals)>=2)
            second=goals[-1];assert second.pose.position.x>4.3,'target did not advance while moving'
            # A guide arriving during optimization changes only the desired
            # target. No request storm/deadline reset, and no reference erasure.
            revised=[s['p'][:],[1.,0.,.8],[2.,0.,.8],[3.,0.,.8],[4.,0.,.8],[5.,0.,.8],[6.,0.,.8]]
            count=len(goals);send_path(revised);send_path(revised)
            assert len(goals)==count and epic[-1]['pending_plan']
            assert epic[-1]['committed_request_stamp_ns']==first_commit
            plan([4.,0.,.8],request=first,duration=10.);spin(.08)
            assert epic[-1]['pending_plan'] and epic[-1]['committed_request_stamp_ns']==first_commit,'old response replaced committed reference'
            p=second.pose.position;plan([p.x,p.y,p.z],request=second,duration=10.)
            wait(lambda:epic[-1]['committed_request_stamp_ns']!=first_commit)
            wait(lambda:len(goals)>count)
            third=goals[-1];p=third.pose.position;plan([p.x,p.y,p.z],duration=10.)
            wait(lambda:not epic[-1]['pending_plan'])
            revision=epic[-1]['revision'];progress=epic[-1]['progress_m']
            # The same path may now start >1m behind the vehicle: it must retain
            # progress rather than rejecting it or resetting to its first point.
            for k in range(7,15):
                s['p']=[k*.1,0.,.8];spin(.025)
                if len(goals)>count+1:
                    p=goals[-1].pose.position;plan([p.x,p.y,p.z],duration=10.);count=len(goals)-1
            send_path(revised)
            assert epic[-1]['active'] and epic[-1]['revision']==revision and epic[-1]['progress_m']>=progress
            before_cancel=len(cancels);s['rc']=2000;wait(lambda:not epic[-1]['active']);s['v']=0.;spin(.1)
            assert len(cancels)>before_cancel,'EPIC stop failed to cancel planner goal'
            assert status[-1]['mission']['kind']=='none' and status[-1]['phase']=='HOVER'
            # Stale retained guides never start a new mission.
            start();before=len(goals);send_path([s['p'],[1.,1.,.8]],stale=True);assert len(goals)==before
            s['rc_fresh']=False;wait(lambda:not epic[-1]['active']);s['rc_fresh']=True;spin(.6)
            assert not epic[-1]['active'],'RC recovery while middle must not restart'
            start();s['battery']=20;wait(lambda:not epic[-1]['active']);assert epic[-1]['reason']=='BATTERY_HOLD';s['battery']=90
            start();send_path([s['p'],[14.,0.,.8]]);wait(lambda:not epic[-1]['active']);assert epic[-1]['reason']=='INVALID_GUIDE_HOLD'
            if not frame_mode:
                start();send_path([s['p'],[1.,1.,.8]],frame='odom');wait(lambda:not epic[-1]['active']);assert epic[-1]['reason']=='INVALID_GUIDE_HOLD'
            start();s['emit_fsm']=False;wait(lambda:not epic[-1]['active'],3.);assert epic[-1]['reason']=='EPIC_TIMEOUT_HOLD';s['emit_fsm']=True
            start();done.publish(Bool(data=True));wait(lambda:not epic[-1]['active']);assert epic[-1]['reason']=='COMPLETE_HOLD'
            start();s['mode']=[3,0];wait(lambda:not epic[-1]['active']);spin(.3);s['mode']=[6,0];spin(.6);assert not epic[-1]['active']
            assert not any(x['phase'] in ('SOLVER_FAILURE','ODOMETRY_JUMP') for x in status)
            if frame_mode:
                start();adapter.kill();wait(lambda:adapter.poll() is not None);wait(lambda:not epic[-1]['active']);assert epic[-1]['reason']=='PROFILE_FRAME_LOST_HOLD',epic[-1]
                print('PASS: ground-only fixed anchor, yaw/translation guide round trip, frame heartbeat loss')
            if not frame_mode:
                start();count=len(goals)
                send_path([s['p'][:],[2.,0.,.8],[3.,0.,.8],[4.,0.,.8],[5.,0.,.8],[6.,0.,.8],[7.,0.,.8]])
                wait(lambda:len(goals)>count);p=goals[-1].pose.position;plan([p.x,p.y,p.z],duration=10.)
                wait(lambda:not epic[-1]['pending_plan']);count=len(goals);s['v']=.5
                original=s['p'][0]
                for k in range(1,6):s['p']=[original+k*.1,0.,.8];spin(.03)
                wait(lambda:len(goals)>count);count=len(goals)
                # Repeated desired updates and fake heartbeats cannot extend a
                # pending request indefinitely or hide its original deadline.
                spin(1.3)
                assert len(goals)==count and not epic[-1]['active']
                assert status[-1]['phase']=='REPLACEMENT_PLAN_TIMEOUT',status[-1]
                print('PASS: missing replacement times out without request starvation; fault reason persists')
            print('PASS: startup edge, rolling preview, moving replacement, single outstanding request, stale response rejection, duplicate progress, planner cancellation, mission exclusion, RC exit/loss/recovery, battery, boundary, EPIC death/completion, mode takeover')
        finally:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            if adapter is not None and adapter.poll() is None:
                adapter.terminate()
                try:adapter.wait(timeout=5)
                except subprocess.TimeoutExpired:adapter.kill();adapter.wait()
            log.seek(0);print(log.read()[-5000:])
node.destroy_node();rclpy.shutdown()
