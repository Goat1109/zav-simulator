"""Rigid yaw/translation alignment between hardware world and EPIC profile world.

Anchor once while disarmed and landed. This node has no FC command interface.
"""
from copy import deepcopy
import math
import time
from .geometry import mul


class ProfileFrame:
    def __init__(self,reference,reference_yaw,position,heading):
        if len(reference)!=3 or len(position)!=3 or not all(math.isfinite(v) for v in (*reference,*position,reference_yaw,heading)):
            raise ValueError('Invalid EPIC frame anchor')
        self.yaw=reference_yaw-heading
        self.c,self.s=math.cos(self.yaw),math.sin(self.yaw)
        self.translation=[reference[0]-self.c*position[0]+self.s*position[1],
                          reference[1]-self.s*position[0]-self.c*position[1],reference[2]-position[2]]
    def forward(self,p):
        x,y,z=p;t=self.translation
        return (self.c*x-self.s*y+t[0],self.s*x+self.c*y+t[1],z+t[2])
    def inverse(self,p):
        x,y,z=(v-t for v,t in zip(p,self.translation))
        return (self.c*x+self.s*y,-self.s*x+self.c*y,z)
    def quaternion(self,q,inverse=False):
        a=(-1. if inverse else 1.)*self.yaw/2
        return mul((math.cos(a),0.,0.,math.sin(a)),q)
    def odometry(self,msg):
        out=deepcopy(msg);p=out.pose.pose.position;q=out.pose.pose.orientation;v=out.twist.twist.linear
        p.x,p.y,p.z=self.forward((p.x,p.y,p.z))
        q.w,q.x,q.y,q.z=self.quaternion((q.w,q.x,q.y,q.z))
        v.x,v.y=self.c*v.x-self.s*v.y,self.s*v.x+self.c*v.y
        # Angular velocity remains in the unchanged body-FLU frame.
        out.pose.covariance=[0.]*36;out.twist.covariance=[0.]*36
        return out
    def path(self,msg):
        out=deepcopy(msg)
        for pose in out.poses:
            p=pose.pose.position;q=pose.pose.orientation
            p.x,p.y,p.z=self.inverse((p.x,p.y,p.z))
            q.w,q.x,q.y,q.z=self.quaternion((q.w,q.x,q.y,q.z),inverse=True)
        return out
    def cloud(self,msg):
        import numpy as np
        fields={f.name:f for f in msg.fields}
        if not msg.width*msg.height or msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:
            raise ValueError('Invalid cloud layout')
        if any(k not in fields or fields[k].datatype!=7 or fields[k].count!=1 or fields[k].offset+4>msg.point_step for k in ('x','y','z')):
            raise ValueError('Cloud requires float32 XYZ')
        buffer=bytearray(msg.data);dtype=np.dtype('>f4' if msg.is_bigendian else '<f4')
        values=[np.ndarray((msg.height,msg.width),dtype=dtype,buffer=buffer,offset=fields[k].offset,
                           strides=(msg.row_step,msg.point_step)) for k in ('x','y','z')]
        x,y,z=values;old_x=x.copy();old_y=y.copy();t=self.translation
        x[:]=self.c*old_x-self.s*old_y+t[0];y[:]=self.s*old_x+self.c*old_y+t[1];z[:]+=t[2]
        out=deepcopy(msg);out.data=buffer;return out


def main(args=None):
    import json
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import Odometry,Path
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    class Adapter(Node):
        def __init__(self):
            super().__init__('epic_profile_frame')
            if self.get_parameter('use_sim_time').value:raise ValueError('Hardware frame requires wall time')
            self.reference=self.declare_parameter('reference_position',[0.,0.,0.]).value
            self.reference_yaw=math.radians(self.declare_parameter('reference_yaw_deg',0.).value)
            self.frame=None;self.bridge_rx=None;self.ground=False;self.ground_since=None;self.ground_samples=0;self.odom_rx=None
            self.last_odom_stamp=0
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.odom_pub=self.create_publisher(Odometry,'/epic/profile_odom',qos_profile_sensor_data)
            self.cloud_pub=self.create_publisher(PointCloud2,'/epic/profile_cloud',qos_profile_sensor_data)
            self.path_pub=self.create_publisher(Path,'/mission/exploration_path',1)
            self.frame_pub=self.create_publisher(PoseStamped,'/zav/epic/profile_transform',qos)
            self.create_subscription(String,'/zav_real/status',self.bridge,qos)
            self.create_subscription(Odometry,'/argo/super/odom',self.odometry,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,'/argo/super/cloud_world',self.cloud,qos_profile_sensor_data)
            self.create_subscription(Path,'/epic/profile_path',self.path,1)
            self.create_timer(.1,self.publish_frame)
        def fresh(self,msg,limit):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            return -.02<=(self.get_clock().now().nanoseconds-stamp)/1e9<limit
        def bridge(self,msg):
            try:
                b=json.loads(msg.data)
                ground=b.get('armed') is False and b.get('landed_fresh') is True and b.get('landed') is True and b.get('checks',{}).get('heartbeat') is True
                observed=time.monotonic()
                self.ground_since=(self.ground_since if self.ground else observed) if ground else None
                self.ground_samples=self.ground_samples+1 if ground and self.ground else 1 if ground else 0
                self.ground=ground;self.bridge_rx=observed
            except (ValueError,TypeError,AttributeError):self.ground=False
        def odometry(self,msg):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if msg.header.frame_id!='world' or msg.child_frame_id!='argo_body' or not self.fresh(msg,.25) or stamp<=self.last_odom_stamp:return
            if self.count_publishers('/argo/super/odom')!=1:return
            p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist.linear
            if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.w,q.x,q.y,q.z,v.x,v.y,v.z)):return
            if not .95<q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z<1.05:return
            if self.frame is None:
                if not self.ground or self.ground_samples<3 or self.ground_since is None or time.monotonic()-self.ground_since<.5 or self.bridge_rx is None or time.monotonic()-self.bridge_rx>.6 or math.hypot(v.x,v.y,v.z)>=.15:return
                heading=math.atan2(2*(q.w*q.z+q.x*q.y),1.-2*(q.y*q.y+q.z*q.z))
                self.frame=ProfileFrame(self.reference,self.reference_yaw,(p.x,p.y,p.z),heading)
                self.get_logger().info(f'EPIC profile aligned: ground world={[p.x,p.y,p.z]}, heading={heading:.9f}rad, profile reference={self.reference}, profile yaw={self.reference_yaw:.9f}rad; anchor fixed until relaunch')
            self.odom_rx=time.monotonic();self.last_odom_stamp=stamp;self.odom_pub.publish(self.frame.odometry(msg))
        def cloud(self,msg):
            if self.frame is None or self.odom_rx is None or time.monotonic()-self.odom_rx>.25 or not self.fresh(msg,.5) or msg.header.frame_id!='world':return
            try:self.cloud_pub.publish(self.frame.cloud(msg))
            except ValueError as error:self.get_logger().error(str(error))
        def path(self,msg):
            if self.frame is None or self.odom_rx is None or time.monotonic()-self.odom_rx>.25 or not self.fresh(msg,1.) or msg.header.frame_id!='world':return
            if self.count_publishers('/epic/profile_path')!=1:return
            self.path_pub.publish(self.frame.path(msg))
        def publish_frame(self):
            if self.frame is None or self.odom_rx is None or time.monotonic()-self.odom_rx>.25:return
            m=PoseStamped();m.header.frame_id='epic_profile';m.header.stamp=self.get_clock().now().to_msg()
            m.pose.position.x,m.pose.position.y,m.pose.position.z=self.frame.translation
            m.pose.orientation.w=math.cos(self.frame.yaw/2);m.pose.orientation.z=math.sin(self.frame.yaw/2)
            self.frame_pub.publish(m)
    rclpy.init(args=args);node=Adapter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RCLError:
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
