#!/usr/bin/env python3
"""Isolated controller test: synthetic feedback only, execute=false, no FC bridge.

Run in a separate container with no network or hardware devices; this also
isolates the controller singleton lock from the production process:

docker run --rm --network none --pull never \
  -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 \
  -v /home/radxa/argo_ws:/root/argo_ws:ro --entrypoint bash argo_humble:base -c \
  'source /opt/ros/humble/setup.bash; source /root/argo_ws/install/setup.bash; \
   python3 /root/argo_ws/src/zav_super/test/hover_transfer.py'
"""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='218' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires isolated ROS domain 218 / localhost only')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_prefix
    rclpy.init();node=rclpy.create_node('hover_transfer_offline_fixture')
    samples=[]
    odom=node.create_publisher(Odometry,'/argo/super/odom',qos_profile_sensor_data)
    cloud=node.create_publisher(PointCloud2,'/argo/super/cloud_world',qos_profile_sensor_data)
    bridge=node.create_publisher(String,'/zav_real/status',QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/zav/super/controller_status',lambda m:samples.append(json.loads(m.data)),10)
    binary=Path(get_package_prefix('zav_super'))/'lib/zav_super/super_ommpc'
    with tempfile.TemporaryDirectory(prefix='hover-transfer-test-') as folder:
        log_path=Path(folder)/'controller.log'
        with log_path.open('w') as output:
            process=subprocess.Popen([str(binary),'--ros-args','-p','execute:=false'],stdout=output,
                                     stderr=subprocess.STDOUT,start_new_session=True)
        def feed(height,mode,duration,cloud_enabled=True):
            begin=time.monotonic();first=len(samples)
            while time.monotonic()-begin<duration:
                if process.poll() is not None:raise AssertionError(log_path.read_text())
                stamp=node.get_clock().now().to_msg()
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.child_frame_id='argo_body'
                o.pose.pose.position.z=height;o.pose.pose.orientation.w=1.;odom.publish(o)
                if cloud_enabled:
                    c=PointCloud2();c.header.stamp=stamp;c.header.frame_id='world';c.height=c.width=1
                    c.point_step=c.row_step=12;c.data=struct.pack('<fff',0.,0.,0.)
                    c.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
                    cloud.publish(c)
                bridge.publish(String(data=json.dumps({'ready':True,'latched':False,
                    'offboard':{'source':'super','stopped':False,'fc_mode':[mode,0]}})))
                rclpy.spin_once(node,timeout_sec=.01)
                time.sleep(.01)
            return samples[first:]
        try:
            initial=feed(2.,3,1.5)
            assert any(p['valid'] and p['phase']=='PRESTREAM' for p in initial),'No valid prestream'
            entry=feed(2.,6,.4)
            assert any(p['valid'] and p['phase']=='HOVER' for p in entry),'No valid hover'
            displaced=feed(2.10,6,.5)
            valid=[p for p in displaced[-10:] if p['valid']]
            assert valid and all(p['phase']=='HOVER' and p['acceleration']<9.8 for p in valid),'Hover height was recaptured'
            recovered=feed(2.,6,.4)
            assert abs(recovered[-1]['acceleration']-9.81)<.02,'Hover reference did not remain at original height'
            feed(2.5,3,.4)
            second=feed(2.5,6,.4)
            assert second[-1]['valid'] and abs(second[-1]['acceleration']-9.81)<.02,'Position return did not reset hold'
            stale=feed(2.5,6,.8,cloud_enabled=False)
            assert any(not p['valid'] and p['phase']=='FEEDBACK_WAIT' for p in stale),'Feedback guard was bypassed'
            print(json.dumps({'execute':False,'no_fc_bridge':True,'position_prestream':True,
                'offboard_hold':True,'height_reference_preserved':True,'next_entry_recaptured':True,
                'feedback_timeout_rejected':True,'samples':len(samples)},indent=2))
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
