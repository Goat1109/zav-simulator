#!/usr/bin/env python3
"""Actual SUPER + OMMPC rolling-guide handoff, ideal scripted reference follower.
No vehicle dynamics/devices/network. This validates protocol, not flight tuning.
"""
import json,os,socket,struct,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(n=='lo' for _,n in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import numpy as np,rclpy,yaml
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly
rclpy.init();node=rclpy.create_node('offline_continuous_super')
retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
odom=node.create_publisher(Odometry,'/argo/super/odom',10);cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
bridge=node.create_publisher(String,'/zav_real/status',retained);marker=node.create_publisher(Marker,'/epic/state',10)
guide=node.create_publisher(NavPath,'/mission/exploration_path',1)
frame=node.create_publisher(PoseStamped,'/zav/epic/profile_transform',retained)
reject_test=os.environ.get('SUPER_REJECT_TEST')=='1'
feedback=node.create_publisher(String,'/zav/super/trajectory_feedback',10)
rejected=None
status=[];epic=[];goals=[];plans=[];events=[]
node.create_subscription(String,'/zav/super/controller_status',lambda m:status.append(json.loads(m.data)),10)
node.create_subscription(String,'/zav/epic/status',lambda m:epic.append(json.loads(m.data)),retained)
node.create_subscription(String,'/zav/super/planner_status',lambda m:events.append(json.loads(m.data)),retained)
node.create_subscription(PoseStamped,'/zav/super/goal',lambda m:goals.append((m,float(np.linalg.norm(state['v'])))),10)
def receive_plan(m):
 global rejected
 if not m.type & Poly.POSITION_TRAJ:return
 plans.append(m)
 if reject_test and rejected is None:
  rejected=(json.loads(m.debug_info)['goal_request_stamp_ns'],m.start_wt_pos)
  feedback.publish(String(data=json.dumps({'event':'REJECTED','stamp_ns':node.get_clock().now().nanoseconds,
   'goal_request_stamp_ns':rejected[0],'start_wt_pos':rejected[1]})))
node.create_subscription(Poly,'/planning_cmd/poly_traj',receive_plan,qos_profile_sensor_data)
state={'p':np.array([0.,0.,.8]),'v':np.zeros(3),'rc':1000,'mode':[3,0],'fsm':'WAIT_TRIGGER','follow':False,'armed':False}
# Compact static room: raw points in hardware world coordinates, like registered LIO.
points=[]
for x in np.arange(-2,9.01,.4):
 for y in np.arange(-3,3.01,.4):points.extend([(x,y,0,1),(x,y,3,1)])
for x in np.arange(-2,9.01,.4):
 for z in np.arange(0,3.01,.4):points.extend([(x,-3,z,1),(x,3,z,1)])
for y in np.arange(-3,3.01,.4):
 for z in np.arange(0,3.01,.4):points.extend([(-2,y,z,1),(9,y,z,1)])
raw=np.array(points,dtype='<f4').tobytes();last_cloud=0.;last_reference=None

def publish():
 global last_cloud,last_reference
 now=node.get_clock().now();t=now.nanoseconds/1e9
 if state['follow'] and epic:
  committed=epic[-1].get('committed_request_stamp_ns',0)
  for candidate in reversed(plans):
   if json.loads(candidate.debug_info)['goal_request_stamp_ns']==committed:
    last_reference=candidate;break
  if last_reference is not None:
   m=last_reference;elapsed=max(0.,min(t-m.start_wt_pos,sum(m.time_pos)))
   for i,duration in enumerate(m.time_pos):
    if elapsed<=duration:break
    elapsed-=duration
   c=[np.array(a[8*i:8*(i+1)]) for a in (m.coef_pos_x,m.coef_pos_y,m.coef_pos_z)]
   state['p']=np.array([np.polyval(a,elapsed) for a in c]);state['v']=np.array([np.polyval(np.polyder(a),elapsed) for a in c])
 stamp=now.to_msg();f=PoseStamped();f.header.stamp=stamp;f.header.frame_id='epic_profile';f.pose.orientation.w=1.;frame.publish(f)
 stamp=now.to_msg();o=Odometry();o.header.frame_id='world';o.child_frame_id='argo_body';o.header.stamp=stamp;o.pose.pose.orientation.w=1.
 o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,state['p'])
 o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=map(float,state['v']);odom.publish(o)
 if time.monotonic()-last_cloud>=.2:
  scan=PointCloud2(height=1,width=len(points),point_step=16,row_step=16*len(points),is_dense=True,data=raw);scan.header=o.header
  scan.fields=[PointField(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate(('x','y','z','intensity'))];cloud.publish(scan);last_cloud=time.monotonic()
 bridge.publish(String(data=json.dumps({'ready':True,'latched':False,'armed':state['armed'],'landed':not state['armed'],'landed_fresh':True,
  'telemetry':{'rc7':{'pwm':state['rc'],'age_s':0.,'fresh':True},'battery_fresh':True,'battery':{'remaining_pct':90,'charge_state':1}},
  'offboard':{'source':'super','fc_mode':state['mode'],'hover_ready':True,'stopped':False,'auto_land_disarm_s':2.}})))
 m=Marker();m.header=o.header;m.text=state['fsm'];marker.publish(m)
node.create_timer(.02,publish)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
def wait(predicate,seconds=10.):
 end=time.monotonic()+seconds
 while time.monotonic()<end and not predicate():
  rclpy.spin_once(node,timeout_sec=.005)
  assert not status or status[-1]['valid'] or status[-1]['phase'] in ('WAITING','PRESTREAM','HOVER','FEEDBACK_WAIT'),status[-1]
 assert predicate(),('timeout',status[-1:] ,epic[-1:],events[-4:])
def path(points):
 m=NavPath();m.header.frame_id='world';m.header.stamp=node.get_clock().now().to_msg()
 for p in points:
  v=PoseStamped();v.header=m.header;v.pose.position.x,v.pose.position.y,v.pose.position.z=map(float,p);v.pose.orientation.w=1.;m.poses.append(v)
 guide.publish(m)
config=yaml.safe_load((Path(get_package_share_directory('zav_super'))/'config/common.yaml').read_text())
config['fsm']['click_goal_topic']='/zav/super/goal';config['super_planner']['visualization_en']=False
config['rog_map']['ros_callback'].update(cloud_topic='/argo/super/cloud_world',odom_topic='/argo/super/odom')
params={k:v for k,v in config['argo_ommpc_px4']['ros__parameters'].items() if k not in ('allow_terminal_hold','use_px4_landing')}
params.update(tracking_backend="ommpc",execute=True,epic_enabled=True,epic_frame_required=True,trajectory_max_vel=1.,trajectory_max_acc=1.5,trajectory_max_jerk=3.,epic_profile='gym',epic_floor=-.4,epic_ceiling=1.,epic_flight_min=.3,epic_flight_max=1.,epic_bounds_lower=[-2.,-3.,0.],epic_bounds_upper=[9.,3.,1.])
processes=[];logs=[]
with tempfile.TemporaryDirectory() as temp:
 root=Path(temp);(root/'common.yaml').write_text(yaml.safe_dump(config));(root/'controller.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
 (root/'planner.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(mission_envelope_enabled=True,mission_flight_min=.3,mission_flight_max=1.,mission_bounds_lower=[-2.,-3.,0.],mission_bounds_upper=[9.,3.,1.])}}))
 try:
  for package,binary,args in [('super_planner','fsm_node',['-p',f'config_name:={root}/common.yaml',
    '-p','flight_height_guard_enabled:=true','-p','flight_floor:=-0.37','-p','flight_ceiling:=0.97','--params-file',str(root/'planner.yaml')]),('zav_super','super_ommpc',['--params-file',str(root/'controller.yaml')])]:
   log=(root/(binary+'.log')).open('w+');logs.append(log);exe=Path(get_package_prefix(package))/'lib'/package/binary
   processes.append(subprocess.Popen([str(exe),'--ros-args',*args],stdout=log,stderr=subprocess.STDOUT))
  wait(lambda:epic and epic[-1]['blocked_by']!='GROUND_ANCHOR_REQUIRED');state['armed']=True
  wait(lambda:epic and status and status[-1]['phase']=='PRESTREAM');spin(2.)
  wait(lambda:node.count_subscribers('/zav/super/goal')>=2,10.);spin(.5)
  state['mode']=[6,0];wait(lambda:status[-1]['phase']=='HOVER');spin(.4);state['rc']=1500
  wait(lambda:epic[-1]['active']);state['fsm']='EXEC_TRAJ'
  path([[x,0.,.8] for x in range(7)]);state['follow']=True
  wait(lambda:len(goals)>=3 and sum(speed>.1 for _,speed in goals)>=2,8.)
  if reject_test:
   assert any(e['event']=='TRAJECTORY_REJECTED_REPLAN' for e in events)
   assert any(json.loads(m.debug_info)['goal_request_stamp_ns']==rejected[0] and m.start_wt_pos!=rejected[1] for m in plans),'no fresh replacement for rejected packet'
   assert sum(m.start_wt_pos==rejected[1] for m in plans)==1,'rejected cache was republished'
  assert sum(speed>.1 for _,speed in goals)>=2,'new goals waited for stopping'
  p=state['p'].copy();points=[p];x=p[0]+1.
  while x<6.:
   points.append([x,.2,.8]);x+=1.
  points.append([6.,.2,.8]);revision=epic[-1]['revision'];path(points)
  wait(lambda:epic[-1]['revision']>revision)
  wait(lambda:status[-1]['mission']['complete'],14.)
  final_requests=[m for m,_ in goals if np.linalg.norm(np.array([
   m.pose.position.x,m.pose.position.y,m.pose.position.z])-np.array([6.,.2,.8]))<1e-8]
  assert len(final_requests)==1,'identical final goal was retried during braking/arrival'
  assert np.linalg.norm(state['p']-np.array(status[-1]['goal']))<.1 and np.linalg.norm(state['v'])<.1
  state['rc']=2000;wait(lambda:not epic[-1]['active']);wait(lambda:any(e['event']=='CANCELLED' for e in events),3.)
  spin(.3);count=len(plans);spin(.5);assert len(plans)==count,'old-goal plans continued after cancellation'
  for m in plans:
   for i,T in enumerate(m.time_pos):
    c=np.array(m.coef_pos_z[8*i:8*(i+1)]);roots=np.roots(np.polyder(c))
    times=[0.,T]+[float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<r.real<T]
    heights=np.polyval(c,times)
    assert min(heights)>=-.4 and max(heights)<=1.,'world height envelope exceeded'
  print('PASS: actual SUPER + OMMPC, rolling goals while moving, revised EPIC path, atomic accepted-reference handoff, final 10cm/0.1m/s, planner cancel',json.dumps({'goals':len(goals),'plans':len(plans),'moving_goals':sum(v>.1 for _,v in goals),'final_position':state['p'].tolist()}))
 finally:
  for proc in processes:
   proc.terminate()
   try:proc.wait(timeout=8)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
  for log in logs:log.seek(0);print(log.read()[-2800:]);log.close()
node.destroy_node();rclpy.shutdown()
