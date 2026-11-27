#!/usr/bin/env python3
"""Recorded polynomial, fresh synthetic telemetry. No FC or flight dynamics.

The 2026-11-25 polynomial reached profile Z=1.201330278, was rejected by
the former mixed-coordinate ceiling=1.2, and preceded the rate-stream loss.
"""
import json,os,socket,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(n=='lo' for _,n in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import rclpy,yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from mars_quadrotor_msgs.msg import PolynomialTrajectory as Poly
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
import struct

fixture=json.loads((Path(__file__).parent/'fixtures/frontier5_rejected_poly.json').read_text())
rclpy.init();node=rclpy.create_node('offline_incident_boundary')
retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
odom=node.create_publisher(Odometry,'/argo/super/odom',10)
cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
bridge=node.create_publisher(String,'/zav_real/status',retained)
frame=node.create_publisher(PoseStamped,'/zav/epic/profile_transform',retained)
marker=node.create_publisher(Marker,'/epic/state',10)
guide=node.create_publisher(NavPath,'/mission/exploration_path',1)
poly=node.create_publisher(Poly,'/planning_cmd/poly_traj',10)
status=[];epic=[];goals=[];feedback=[]
node.create_subscription(String,'/zav/super/controller_status',lambda m:status.append(json.loads(m.data)),10)
node.create_subscription(String,'/zav/epic/status',lambda m:epic.append(json.loads(m.data)),retained)
node.create_subscription(String,'/zav/super/trajectory_feedback',lambda m:feedback.append(json.loads(m.data)),10)
node.create_subscription(PoseStamped,'/zav/super/goal',goals.append,10)
s={'armed':False,'mode':[3,0],'rc':1000}
p=[fixture['polynomial'][k][7] for k in ('coef_pos_x','coef_pos_y','coef_pos_z')]

def publish():
    stamp=node.get_clock().now().to_msg()
    o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.child_frame_id='argo_body'
    o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=p;o.pose.pose.orientation.w=1.;odom.publish(o)
    c=PointCloud2(height=1,width=1,point_step=12,row_step=12,data=struct.pack('<fff',0.,0.,0.));c.header=o.header
    c.fields=[PointField(name=k,offset=4*i,datatype=7,count=1) for i,k in enumerate('xyz')];cloud.publish(c)
    bridge.publish(String(data=json.dumps({'ready':True,'latched':False,'armed':s['armed'],'landed':not s['armed'],
      'landed_fresh':True,'checks':{'heartbeat':True},'telemetry':{'rc7':{'pwm':s['rc'],'age_s':0.,'fresh':True},
      'battery_fresh':True,'battery':{'remaining_pct':90,'charge_state':1}},
      'offboard':{'source':'super','fc_mode':s['mode'],'hover_ready':True,'stopped':False,'auto_land_disarm_s':2.}})))
    f=PoseStamped();f.header.stamp=stamp;f.header.frame_id='epic_profile'
    f.pose.position.x,f.pose.position.y,f.pose.position.z=fixture['translation']
    f.pose.orientation.w,f.pose.orientation.x,f.pose.orientation.y,f.pose.orientation.z=fixture['quaternion_wxyz'];frame.publish(f)
    m=Marker();m.header=o.header;m.text='WAIT_TRIGGER';marker.publish(m)
node.create_timer(.02,publish)
def wait(predicate,limit=5.):
    until=time.monotonic()+limit
    while time.monotonic()<until and not predicate():rclpy.spin_once(node,timeout_sec=.01)
    assert predicate(),(status[-1:],epic[-1:])
def spin(duration):
    until=time.monotonic()+duration
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)

with tempfile.TemporaryDirectory() as temp:
    # Historical fixture peaks at 1.0096m/s. This test isolates coordinate/boundary
    # acceptance; independent kinematic tests enforce the production 1.0m/s cap.
    config={'trajectory_max_vel':1.1,'execute':True,'epic_enabled':True,'epic_profile':'frontier_single_floor','epic_frame_required':True,
      'epic_floor':-.4,'epic_ceiling':1.2,'epic_flight_min':.8,'epic_flight_max':1.7,
      'epic_bounds_lower':[-11.2,-11.2,-.3,-9.6,-2.,-.3],
      'epic_bounds_upper':[-6.5,47.2,2.4,-2.6,1.65,2.4]}
    cfg=Path(temp)/'params.yaml';cfg.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
    with (Path(temp)/'controller.log').open('w+') as log:
        binary=Path(get_package_prefix('zav_super'))/'lib/zav_super/super_ommpc'
        proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(cfg)],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:epic and status and epic[-1]['profile_height_bounds'][1]>1.5)
            s['armed']=True;wait(lambda:status[-1]['phase']=='PRESTREAM');spin(.4)
            s['mode']=[6,0];wait(lambda:status[-1]['phase']=='HOVER');spin(.5)
            s['rc']=1500;wait(lambda:epic[-1]['active'])
            path=NavPath();path.header.frame_id='world';path.header.stamp=node.get_clock().now().to_msg()
            for xyz in (p,fixture['accepted_goal']):
                pose=PoseStamped();pose.header=path.header;pose.pose.orientation.w=1.
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=xyz;path.poses.append(pose)
            guide.publish(path);wait(lambda:goals)
            m=Poly()
            for key,value in fixture['polynomial'].items():setattr(m,key,value)
            m.header.frame_id='world';m.header.stamp=node.get_clock().now().to_msg()
            m.start_wt_pos=m.start_wt_yaw=node.get_clock().now().nanoseconds/1e9+.02
            stamp=goals[-1].header.stamp;request=stamp.sec*10**9+stamp.nanosec
            m.debug_info=json.dumps({'goal_request_stamp_ns':request,'accepted_goal':fixture['accepted_goal']})
            poly.publish(m);wait(lambda:epic[-1]['committed_request_stamp_ns']==request,.5)
            assert not feedback,'recorded safe polynomial still rejected'
            # A boundary-compatible constant-velocity packet must still be
            # rejected by the independent kinematic guard before commitment.
            import copy
            fast=copy.deepcopy(m);fast.piece_num_pos=1;fast.time_pos=[2.]
            fast.coef_pos_x=[0.]*6+[1.2,p[0]]
            fast.coef_pos_y=[0.]*7+[p[1]];fast.coef_pos_z=[0.]*7+[p[2]]
            fast.header.stamp=node.get_clock().now().to_msg()
            count=len(feedback);poly.publish(fast);wait(lambda:len(feedback)>count,.5)
            assert feedback[-1]['goal_request_stamp_ns']==request

            # A genuine WORLD height violation must still be rejected.
            z=list(m.coef_pos_z)
            for i in range(7,len(z),8):z[i]+=1.
            m.coef_pos_z=z;m.header.stamp=node.get_clock().now().to_msg();count=len(feedback);poly.publish(m)
            wait(lambda:len(feedback)>count,.5)
            assert feedback[-1]['goal_request_stamp_ns']==request
            print(json.dumps({'result':'PASS','recorded_boundary_compatible_polynomial_accepted':True,
              'real_world_height_violation_rejected':True,'overspeed_rejected':True,'profile_height_bounds':epic[-1]['profile_height_bounds'],
              'aligned_flight_band':epic[-1]['aligned_flight_band'],'flight_commands':False}))
        finally:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            log.seek(0);print(log.read()[-1500:])
node.destroy_node();rclpy.shutdown()
