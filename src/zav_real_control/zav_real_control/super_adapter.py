"""Hardware IMU odometry -> SUPER body-FLU/world contract; world cloud passthrough.

No spawn, ground truth, /clock, flight command, or second mounting correction of
already world-registered points. T_BI is shared with the existing EV bridge.
"""
from copy import deepcopy
import math
from .geometry import body_odometry, frd, mul, conj


def body_message(msg, config):
    if (msg.header.frame_id,msg.child_frame_id)!=(config['odom_parent'],config['odom_child']):
        raise ValueError('Unexpected IMU odometry frame')
    p,q,v,w=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
    pos,quat,vel,omega=body_odometry((p.x,p.y,p.z),(q.w,q.x,q.y,q.z),
        (v.x,v.y,v.z),(w.x,w.y,w.z),config['body_from_imu_q_wxyz'],config['imu_position_body_flu_m'])
    flip=(0.,1.,0.,0.)
    quat=mul(mul(flip,quat),conj(flip))
    out=deepcopy(msg)
    out.header.frame_id='world';out.child_frame_id='argo_body'
    out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=frd(pos)
    out.pose.pose.orientation.w,out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z=quat
    out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=frd(vel)
    out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=frd(omega)
    # Source covariance is in IMU coordinates. Do not advertise it as body/world covariance.
    out.pose.covariance=[0.]*36;out.twist.covariance=[0.]*36
    return out


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,PointField
    from .config import load_config

    class Adapter(Node):
        def __init__(self):
            super().__init__('zav_super_adapter')
            if self.get_parameter('use_sim_time').value:raise ValueError('Hardware requires use_sim_time=false')
            self.c=load_config(self.declare_parameter('config_file','').value)
            self.input=self.declare_parameter('odom_topic',self.c['odom_topic']).value
            self.cloud_frame=self.declare_parameter('cloud_frame','world').value
            self.max_age=.25
            self.odom=self.create_publisher(Odometry,'/argo/super/odom',10)
            self.cloud=self.create_publisher(PointCloud2,'/argo/super/cloud_world',1)
            self.last_stamp=0;self.last_odom_rx=None
            self.create_subscription(Odometry,self.input,self.on_odom,qos_profile_sensor_data)
            self.create_subscription(PointCloud2,self.declare_parameter('cloud_topic','/super_lio/cloud_world').value,self.on_cloud,qos_profile_sensor_data)
            self.get_logger().info('SUPER hardware adapter: body mount shared with EV; registered world points retain XYZ')
        def fresh(self,msg,age):
            dt=self.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            return -.02<=dt<age
        def on_odom(self,msg):
            stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if stamp<=self.last_stamp or not self.fresh(msg,.25) or self.count_publishers(self.input)!=1:return
            try:out=body_message(msg,self.c)
            except ValueError as error:
                self.get_logger().error(str(error));return
            self.last_stamp=stamp;self.last_odom_rx=self.get_clock().now().nanoseconds
            self.odom.publish(out)
        def on_cloud(self,msg):
            if not self.fresh(msg,.5) or msg.header.frame_id!=self.cloud_frame or not msg.width*msg.height:return
            if self.last_odom_rx is None or self.get_clock().now().nanoseconds-self.last_odom_rx>250000000:return
            fields={f.name:f for f in msg.fields}
            if any(k not in fields or fields[k].datatype!=PointField.FLOAT32 or fields[k].offset+4>msg.point_step for k in ('x','y','z')):return
            if msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:return
            out=deepcopy(msg);out.header.frame_id='world';self.cloud.publish(out)
    rclpy.init(args=args);node=Adapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
