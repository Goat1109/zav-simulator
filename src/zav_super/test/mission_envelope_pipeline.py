#!/usr/bin/env python3
"""Recorded final request + native planner, fresh synthetic map and poses.
No FC/devices, no dynamics replay. Uses the actual second-flight transform.
"""
import json,os,socket,struct,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(n=='lo' for _,n in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import numpy as np,rclpy,yaml
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly
rclpy.init();node=rclpy.create_node('recorded_mission_envelope_regression')
retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
odom=node.create_publisher(Odometry,'/argo/super/odom',10);cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
frame=node.create_publisher(PoseStamped,'/zav/epic/profile_transform',retained);goal=node.create_publisher(PoseStamped,'/test/goal',1)
entry=node.create_publisher(NavPath,'/zav/super/entry_goal',1)
plans=[];events=[]
node.create_subscription(Poly,'/planning_cmd/poly_traj',lambda m:plans.append(m) if m.type&Poly.POSITION_TRAJ else None,qos_profile_sensor_data)
node.create_subscription(String,'/zav/super/planner_status',lambda m:events.append(json.loads(m.data)),retained)
tr=np.array([-3.3085676191682145,-.0859663940202335,.34432277840586933]);w,z=.06477022639804543,-.9979002043152141
R=np.array([[1-2*z*z,-2*w*z,0],[2*w*z,1-2*z*z,0],[0,0,1.]])
lo=np.array([[-11.2,-11.2,-.3],[-9.6,-2.,-.3]]);hi=np.array([[-6.5,47.2,2.4],[-2.6,1.65,2.4]])
requested=np.array([4.533537999554331,-25.97212448524284,1.0009707434752342]);old=np.array([4.575,-26.025,.975]);body=np.array([4.3,-24.8,1.04])
def valid(p):
 p=R@np.asarray(p)+tr
 return 1.2-1e-5<=p[2]<=1.6+1e-5 and any(np.all(p>=a-1e-5) and np.all(p<=b+1e-5) for a,b in zip(lo,hi))
assert valid(requested) and not valid(old)
case=os.environ.get('MISSION_ENVELOPE_CASE','recorded')
if case=='box_join':
 body=R.T@(np.array([-3.3,0.,1.4])-tr)
 requested=R.T@(np.array([-10.,4.,1.4])-tr)
if case in ('entry_low','entry_xy'):
 start_profile=[-3.3,0.,1.02] if case=='entry_low' else [-2.25,0.,1.4]
 body=R.T@(np.array(start_profile)-tr)
 requested=R.T@(np.array([-3.3,0.,1.4])-tr)
points=[]
for x in np.arange(-1,10.1,.5):
 for y in np.arange(-35,5.1,.5):points.extend([(x,y,-.25,1),(x,y,2.35,1)])
raw=np.asarray(points,dtype='<f4').tobytes();last_cloud=0.
def publish():
 global last_cloud
 stamp=node.get_clock().now().to_msg();o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.child_frame_id='argo_body';o.pose.pose.orientation.w=1.
 o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=map(float,body);odom.publish(o)
 f=PoseStamped();f.header.stamp=stamp;f.header.frame_id='epic_profile';f.pose.position.x,f.pose.position.y,f.pose.position.z=map(float,tr);f.pose.orientation.w=w;f.pose.orientation.z=z;frame.publish(f)
 if time.monotonic()-last_cloud>.2:
  c=PointCloud2(height=1,width=len(points),point_step=16,row_step=len(raw),data=raw);c.header=o.header
  c.fields=[PointField(name=k,offset=4*i,datatype=7,count=1)for i,k in enumerate(('x','y','z','intensity'))];cloud.publish(c);last_cloud=time.monotonic()
node.create_timer(.02,publish)
def spin(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
config=yaml.safe_load((Path(get_package_share_directory('zav_super'))/'config/common.yaml').read_text())
config['fsm']['click_goal_topic']='/test/goal';config['super_planner']['visualization_en']=False
config['rog_map']['ros_callback'].update(cloud_topic='/argo/super/cloud_world',odom_topic='/argo/super/odom')
with tempfile.TemporaryDirectory() as temp:
 root=Path(temp);(root/'common.yaml').write_text(yaml.safe_dump(config))
 params=dict(config_name=str(root/'common.yaml'),flight_height_guard_enabled=True,flight_floor=-.37,flight_ceiling=1.37,
  mission_envelope_enabled=True,mission_flight_min=1.2,mission_flight_max=1.6,mission_bounds_lower=lo.ravel().tolist(),mission_bounds_upper=hi.ravel().tolist())
 (root/'planner.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
 with (root/'planner.log').open('w+') as log:
  exe=Path(get_package_prefix('super_planner'))/'lib/super_planner/fsm_node'
  proc=subprocess.Popen([str(exe),'--ros-args','--params-file',str(root/'planner.yaml')],stdout=log,stderr=subprocess.STDOUT)
  try:
   spin(3.);assert proc.poll() is None
   assert any(e['event']=='MISSION_ENVELOPE_READY'for e in events),events
   g=PoseStamped();g.header.frame_id='world';g.header.stamp=node.get_clock().now().to_msg();g.pose.orientation.w=1.;g.pose.position.x,g.pose.position.y,g.pose.position.z=map(float,requested)
   if case.startswith('entry_'):
    initial=PoseStamped();initial.header=g.header;initial.pose.position.x,initial.pose.position.y,initial.pose.position.z=map(float,body)
    entry.publish(NavPath(header=g.header,poses=[initial,g]))
   else:goal.publish(g)
   end=time.monotonic()+8.
   while not plans and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
   assert plans,'no feasible replacement for the recorded goal'
   msg=plans[0];accepted=json.loads(msg.debug_info)['accepted_goal'];assert valid(accepted),accepted
   assert np.linalg.norm(np.array(accepted)-requested)<=.3
   max_v=0.
   for i,t in enumerate(msg.time_pos):
    c=np.array([getattr(msg,'coef_pos_'+k)[8*i:8*i+8]for k in 'xyz']);ts=np.linspace(0,t,301)
    positions=np.array([np.polyval(v,ts)for v in c]).T
    if case.startswith('entry_'):
     delta=requested-body;yaw=np.arctan2(delta[1],delta[0]);cs,sn=np.cos(yaw),np.sin(yaw)
     axes=np.array([[cs,sn,0],[-sn,cs,0],[0,0,1]])
     local=(positions-body)@axes.T;end=axes@delta
     assert np.all(local>=np.minimum(end,0)-.35001) and np.all(local<=np.maximum(end,0)+.35001)
     assert np.all((positions[:,2]>=-.37001)&(positions[:,2]<=1.37001))
     if i==0:assert np.linalg.norm(positions[0]-body)<.05
     if i+1==len(msg.time_pos):assert valid(positions[-1])
    else:assert all(valid(p)for p in positions),'curve exits mission bounds'
    vel=np.array([np.polyval(np.polyder(v),ts)for v in c]).T;max_v=max(max_v,float(np.linalg.norm(vel,axis=1).max()))
   assert max_v<=1.00001
   print(json.dumps({'result':'PASS','case':case,'requested_goal':requested.tolist(),'old_invalid_goal':old.tolist(),'new_accepted_goal':accepted,'max_sampled_speed':max_v,'flight_commands':False}))
  finally:
   proc.terminate()
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
   log.seek(0);print(log.read()[-3500:])
node.destroy_node();rclpy.shutdown()
