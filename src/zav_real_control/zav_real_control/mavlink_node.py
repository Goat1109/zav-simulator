"""Single ROS odometry subscriber + one USB-TTL MAVLink owner; no DDS or MAVROS process."""
import fcntl
import json
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .config import load_config


class MavlinkBridge(Node):
    def __init__(self):
        super().__init__('zav_real_mavlink_bridge')
        self.c=load_config(self.declare_parameter('config_file','').value)
        if self.c.get('transport')!='mavlink':raise ValueError('This launch requires transport: mavlink')
        if self.get_parameter('use_sim_time').value:raise ValueError('Hardware requires use_sim_time=false')
        live=self.declare_parameter('publish_enabled',self.c['publish_enabled']).value
        # Explicitly select MAVLink 2: ODOMETRY has message id 331 and needs v2 framing.
        os.environ['MAVLINK20']='1'
        from pymavlink import mavutil
        from .serial_link import CheckedSerial, configure_ftdi_latency
        from .single_link import SingleLink
        mavutil.set_dialect('common')
        self.connection=CheckedSerial(self.c['mavlink']['connection'],
            baud=self.c['mavlink']['baud'],source_system=self.c['source_system'],
            source_component=self.c['source_component'])
        try:
            if hasattr(self.connection,'port') and hasattr(self.connection.port,'exclusive'):
                self.connection.port.exclusive=True
            latency=configure_ftdi_latency(self.c['mavlink']['connection'],
                                          self.c['mavlink']['ftdi_latency_timer_ms'])
            if latency is not None:
                self.get_logger().info(f'FTDI {latency["device"]}: receive latency '
                    f'{latency["actual_ms"]} ms (previous {latency["previous_ms"]} ms)')
            self.core=SingleLink(self.connection,self.c,bool(live))
            source=self.declare_parameter('offboard_source','legacy').value
            if source not in ('legacy','super'):raise ValueError('Unknown offboard_source')
            if source=='super':
                from .super_gate import SuperRateGate
                self.core.offboard=SuperRateGate(automatic_takeoff=bool(self.declare_parameter('automatic_takeoff',False).value),
                    hover_thrust=float(self.declare_parameter('takeoff_hover_thrust',0.).value),
                    takeoff_height=float(self.declare_parameter('takeoff_height',.4).value))
            self.offboard_source=source
            self.core.offboard.enabled=bool(self.declare_parameter('offboard_enabled',False).value)
            if self.core.offboard.enabled:
                self.core.offboard.reason='waiting_for_armed_airborne_healthy_position'
            self.last_report=-float('inf');self.last_state=None
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
            reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub=self.create_publisher(String,self.c['status_prefix']+'/status',reliable)
            self.create_subscription(Odometry,self.c['odom_topic'],self.odometry,qos)
            self.super_sub=self.create_subscription(String,'/zav/super/rates',self.super_request,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
            self.offboard_sub=self.create_subscription(String,self.c['status_prefix']+'/offboard/request',self.offboard_request,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
            # PX4 rejects timesync RTT >=10 ms. A 10 ms receive poll adds up
            # to that entire budget before UART/FC scheduling is accounted for.
            self.create_timer(.001,self.tick)
        except Exception:
            self.connection.close();raise
        self.get_logger().info(f'Single MAVLink link: {self.c["mavlink"]["connection"]}, '
                               f'{self.c["mavlink"]["baud"]} baud; live={live}')
        self.get_logger().info(f'TIMESYNC: {self.c["mavlink"]["timesync_initial_rate_hz"]} Hz '
            f'until FC convergence, then {self.c["mavlink"]["timesync_steady_rate_hz"]} Hz; '
            'FC convergence checks unchanged')

    def offboard_request(self,msg):
        if not self.core.offboard.enabled or self.offboard_source!='legacy':
            return
        try:
            if self.count_publishers(self.c['status_prefix']+'/offboard/request')!=1:
                self.core.offboard.abort('multiple_test_publishers')
                return
            self.core.offboard.accept(json.loads(msg.data),time.monotonic(),time.time_ns())
        except (ValueError,TypeError,KeyError) as error:
            if self.core.offboard.session is not None:
                self.core.offboard.abort('invalid_test_request')
            self.get_logger().error(f'Offboard request rejected: {error}')

    def super_request(self,msg):
        if not self.core.offboard.enabled or self.offboard_source!='super':return
        try:
            if self.count_publishers('/zav/super/rates')!=1:
                self.core.offboard.abort('SUPER_MULTIPLE_CONTROLLERS');return
            self.core.offboard.accept(json.loads(msg.data),time.monotonic(),time.time_ns())
        except (ValueError,TypeError,KeyError) as error:
            if self.core.offboard.prestream_since is not None:self.core.offboard.abort('SUPER_INVALID_REQUEST')
            self.get_logger().error(f'SUPER request rejected: {error}')

    def odometry(self,msg):
        p,q,v,w=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        try:
            self.core.odometry(msg.header.stamp.sec*1000000+msg.header.stamp.nanosec//1000,
                (p.x,p.y,p.z),(q.w,q.x,q.y,q.z),(v.x,v.y,v.z),(w.x,w.y,w.z),
                time.monotonic(),time.time_ns()//1000,(msg.header.frame_id,msg.child_frame_id))
        except (OSError,ValueError) as error:
            self.core.fail_io(error)

    def tick(self):
        now=time.monotonic()
        data=self.core.update(now,time.time_ns())
        if now-self.last_report>=.2:
            self.pub.publish(String(data=json.dumps(data,allow_nan=False)))
            state=(data['mode'],data['phase'],tuple(data['blocked_by']))
            if state!=self.last_state:
                self.get_logger().warning(f'{state}; jump={data["reasons"]}; error={data["error"]}')
                self.last_state=state
            self.last_report=now

    def destroy_node(self):
        self.connection.close()
        return super().destroy_node()


def main(args=None):
    # Stop between executor calls, before destroying the context/subscriptions.
    # Humble's asynchronous shutdown can otherwise race take_message().
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    stopping=False
    def stop(signum,frame):
        nonlocal stopping
        stopping=True
    previous={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    node=None
    lock=open(f'/tmp/zav-real-EV-{os.getuid()}.lock','w')
    try:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another mavlink_bridge is running. Stop the existing '
                             'rc_flight launch with Ctrl+C before restarting with publish:=true. '
                             'Do not delete the lock file while that process is running.')
        node=MavlinkBridge()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        try:
            if node:node.destroy_node()
        finally:
            lock.close()
            if rclpy.ok():rclpy.shutdown()
            for sig,handler in previous.items():signal.signal(sig,handler)
