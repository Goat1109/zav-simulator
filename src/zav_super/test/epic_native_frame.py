#!/usr/bin/env python3
"""Actual EPIC + hardware frame adapter, synthetic room, no FC/controller/device."""
import json,math,os,socket,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(n=='lo' for _,n in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField,Imu
from visualization_msgs.msg import Marker
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('offline_epic_native_frame')
odom=node.create_publisher(Odometry,'/argo/super/odom',qos_profile_sensor_data)
cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',qos_profile_sensor_data)
imu=node.create_publisher(Imu,'/livox/imu',qos_profile_sensor_data)
bridge=node.create_publisher(String,'/zav_real/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
trigger=node.create_publisher(NavPath,'/epic/trigger',1)
received=[];states=[]
node.create_subscription(NavPath,'/mission/exploration_path',received.append,1)
node.create_subscription(Marker,'/epic/state',lambda m:states.append(m.text),10)
points=[]
for x in np.arange(-8,8.01,.4):
 for z in np.arange(0,5.01,.4):points.extend([(x,-8,z),(x,8,z)])
for y in np.arange(-8,8.01,.4):
 for z in np.arange(0,5.01,.4):points.append((-8,y,z))
for x in np.arange(-8,8.01,.4):
 for y in np.arange(-8,8.01,.4):points.extend([(x,y,0),(x,y,5)])
raw=np.array(points,dtype='<f4').tobytes();begin=time.monotonic();sent=False;last_cloud=0.;height=0.
def publish():
 global last_cloud,height,sent
 elapsed=time.monotonic()-begin;height=min(1.6,max(0.,elapsed-2.))
 stamp=node.get_clock().now().to_msg();o=Odometry();o.header.frame_id='world';o.child_frame_id='argo_body';o.header.stamp=stamp
 o.pose.pose.position.z=height;o.pose.pose.orientation.w=1.;odom.publish(o)
 i=Imu();i.header.stamp=stamp;imu.publish(i)
 bridge.publish(String(data=json.dumps({'armed':elapsed>=2.,'landed':elapsed<2.,'landed_fresh':True,'checks':{'heartbeat':True}})))
 if time.monotonic()-last_cloud>=.1:
  c=PointCloud2(height=1,width=len(points),point_step=12,row_step=12*len(points),is_dense=True,data=raw);c.header=o.header
  c.fields=[PointField(name=k,offset=4*n,datatype=7,count=1)for n,k in enumerate('xyz')];cloud.publish(c);last_cloud=time.monotonic()
 if not sent and elapsed>4.5 and states and states[-1]=='WAIT_TRIGGER':
  p=PoseStamped();p.header=o.header;p.pose.position.z=height;p.pose.orientation.w=1.
  m=NavPath();m.header=o.header;m.poses=[p];trigger.publish(m);sent=True
node.create_timer(.02,publish)
with tempfile.TemporaryFile(mode='w+') as log:
 processes=[]
 try:
  binary=str(Path(get_package_prefix('zav_real_control'))/'lib/zav_real_control/epic_frame_adapter')
  processes.append(subprocess.Popen([binary,'--ros-args','-p','reference_position:=[1.0,-2.0,0.2]','-p','reference_yaw_deg:=90.0'],stdout=log,stderr=subprocess.STDOUT))
  config=str(Path(get_package_share_directory('epic_ros2'))/'config/frontier_bounds.yaml')
  processes.append(subprocess.Popen(['ros2','launch','epic_ros2','hardware.launch.py',f'config:={config}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
  deadline=time.monotonic()+35.
  while not received and time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.02)
   assert all(p.poll() is None for p in processes),'native process exited'
  assert received,('No native guide',list(dict.fromkeys(states)))
  m=received[-1];assert m.header.frame_id=='world' and len(m.poses)>=2
  p=m.poses[0].pose.position
  assert math.dist((p.x,p.y,p.z),(0.,0.,height))<.3,(p,height)
  assert all(math.isfinite(v) for p in m.poses for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z))
  print(json.dumps({'result':'PASS','actual_nodes':['epic_profile_frame','epic_exploration'],
     'reference_position':[1.,-2.,.2],'reference_yaw_deg':90.,'native_fsm':list(dict.fromkeys(states)),
     'guide_vertices':len(m.poses),'world_departure':[p.x,p.y,p.z],'physical_devices':False}))
 finally:
  import signal
  for n,p in enumerate(processes):
   if p.poll() is None:
    if n==1:os.killpg(p.pid,signal.SIGINT)
    else:p.terminate()
  for p in processes:
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  log.seek(0);print(log.read()[-3500:])
node.destroy_node();rclpy.shutdown()
