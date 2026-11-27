#!/usr/bin/env python3
"""Native EPIC anchor/height admission, isolated ROS only, no vehicle nodes."""
import os,socket,subprocess,tempfile,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='218' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert all(n=='lo' for _,n in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*'))+list(Path('/dev').glob('ttyUSB*'))
import rclpy
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
rclpy.init();node=rclpy.create_node('offline_native_height_guard')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
frame=node.create_publisher(PoseStamped,'/zav/epic/profile_transform',qos)
trigger=node.create_publisher(NavPath,'/waypoint_generator/waypoints',1)
def send_trigger():
    m=NavPath();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='world'
    p=PoseStamped();p.pose.position.z=.8;m.poses=[p];trigger.publish(m)
with tempfile.TemporaryDirectory() as temp:
    binary=Path(get_package_prefix('epic_ros2'))/'lib/epic_ros2/exploration_node'
    config=Path(get_package_share_directory('epic_ros2'))/'config/frontier-first-floor-sim.yaml'
    with (Path(temp)/'native.log').open('w+') as log:
        args=[str(binary),'--ros-args','--params-file',str(config)]
        for key,value in {'fsm.external_local_planner':'true','fsm.hardware_height_guard':'true',
            'fsm.world_floor':'-0.4','fsm.world_ceiling':'1.2','fsm.height_margin':'0.03',
            'exploration.tsp_dir':temp,'view_graph':'false'}.items():args+=['-p',f'{key}:={value}']
        proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        def wait(text,emit,limit=12.):
            end=time.monotonic()+limit
            while time.monotonic()<end:
                assert proc.poll() is None,'native node exited'
                emit();rclpy.spin_once(node,timeout_sec=.05)
                if text in (Path(temp)/'native.log').read_text():return
            raise AssertionError((Path(temp)/'native.log').read_text()[-4000:])
        try:
            wait('aligned height bounds not ready',send_trigger)
            def anchor(z):
                m=PoseStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='epic_profile'
                m.pose.position.z=z;m.pose.orientation.w=1.;frame.publish(m)
            wait('height band has no overlap',lambda:anchor(-2.))
            wait('EPIC aligned PROFILE flight Z 1.200000..1.515305',lambda:anchor(.3453047100395165))
            print('PASS: native EPIC blocks missing anchor and empty intersection; recorded anchor yields profile 1.2..1.515305 / world 0.854695..1.17')
        finally:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            log.seek(0);print(log.read()[-1400:])
node.destroy_node();rclpy.shutdown()
