"""Convert Super-LIO world velocity to the existing IMU-child EV contract."""
from copy import deepcopy
import math
import time

from .geometry import conj, rotate
from .motion import unit_quaternion


def adapt(msg):
    if msg.header.frame_id != 'world' or msg.child_frame_id not in ('', 'imu'):
        raise ValueError('unexpected Super-LIO frames')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    if not all(math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z)):
        raise ValueError('nonfinite Super-LIO sample')
    quat = unit_quaternion((q.w, q.x, q.y, q.z))
    velocity = rotate(conj(quat), (v.x, v.y, v.z))
    out = deepcopy(msg)
    out.header.frame_id = 'camera_init'
    out.child_frame_id = 'mid360_imu'
    out.pose.pose.orientation.w, out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z = quat
    out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class CorrectionGate:
    def __init__(self, max_age):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max correction age must be positive')
        self.max_age = max_age
        self.correction = None
        self.received = None
        self.last_output = None

    def correct(self, measured, now):
        if self.correction is not None and measured <= self.correction:
            return
        self.correction, self.received = measured, now

    def ready(self, measured, now):
        return (self.correction is not None and measured > 0
                and 0 <= measured - self.correction <= self.max_age
                and 0 <= now - self.received <= self.max_age
                and (self.last_output is None or measured > self.last_output))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    class Adapter(Node):
        def __init__(self):
            super().__init__('super_lio_ev_adapter')
            self.gate = CorrectionGate(self.declare_parameter('max_correction_age_s', 0.25).value)
            self.reason = None
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub = self.create_publisher(Odometry, '/zav/lio/odometry', qos)
            self.create_subscription(Odometry, '/super_lio/odom', self.correction, qos)
            self.create_subscription(Odometry, '/super_lio/imu/odom', self.odometry, qos)
            self.get_logger().info('Super-LIO -> /zav/lio/odometry; measurement stamps preserved; no FC commands')

        def report(self, reason):
            if reason != self.reason:
                self.get_logger().info(reason)
                self.reason = reason

        def correction(self, msg):
            try:
                adapt(msg)
            except ValueError:
                return
            self.gate.correct(stamp(msg), time.monotonic())

        def odometry(self, msg):
            if (self.count_publishers('/zav/lio/odometry') != 1
                    or self.count_publishers('/super_lio/imu/odom') != 1
                    or self.count_publishers('/super_lio/odom') != 1):
                self.report('BLOCKED: duplicate or undiscovered odometry publisher; run only one estimator')
                return
            measured = stamp(msg)
            if not self.gate.ready(measured, time.monotonic()):
                self.report('BLOCKED: waiting for recent LiDAR correction or monotonic IMU sample')
                return
            try:
                out = adapt(msg)
            except ValueError as error:
                self.report('BLOCKED: ' + str(error))
                return
            self.gate.last_output = measured
            self.pub.publish(out)
            self.report('READY: forwarding Super-LIO IMU odometry with recent LiDAR correction')

    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
