"""Six ROS 2 test clients. Preview by default; no UART, arm or mode commands."""
from dataclasses import asdict, fields
import fcntl
import json
import os
import time
import uuid

from .offboard import Options, segments


def main(pattern, args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init(args=args)

    class TestNode(Node):
        def __init__(self):
            super().__init__('offboard_test_'+pattern)
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Real FC test clients require use_sim_time=false')
            self.execute = self.declare_parameter('execute', False).value
            defaults = Options()
            self.options = Options(**{f.name:self.declare_parameter(f.name,getattr(defaults,f.name)).value for f in fields(defaults)})
            self.prefix = self.declare_parameter('status_prefix','/zav_real').value
            self.session, self.sequence = uuid.uuid4().hex, 0
            self.last_status = None
            self.lock = None
            self.pub = None
            if self.execute:
                self.lock = open(f'/tmp/zav-offboard-test-{os.getuid()}.lock','w')
                try:
                    fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    raise ValueError('Another Offboard test is running')
                self.pub = self.create_publisher(String,self.prefix+'/offboard/request',
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
                self.create_timer(.05,self.request)
                self.create_subscription(String,self.prefix+'/status',self.status,10)
            self.get_logger().info(json.dumps(dict(pattern=pattern,execute=self.execute,
                segments=segments(pattern,self.options),options=asdict(self.options))))
            self.get_logger().info('LIVE REQUESTS enabled; enter OFFBOARD_HOVER_READY first, then start this command. Requests started in Position are rejected; restart this test client after hover is ready.'
                                   if self.execute else 'PREVIEW ONLY: no Offboard requests or flight commands. execute:=true is required for live use.')

        def request(self):
            self.sequence += 1
            self.pub.publish(String(data=json.dumps(dict(session=self.session,sequence=self.sequence,
                stamp_ns=time.time_ns(),pattern=pattern,options=asdict(self.options)),allow_nan=False)))

        def status(self,msg):
            try:
                data=json.loads(msg.data).get('offboard',{})
                phase=(data.get('session'),data.get('phase'),data.get('command_state'),
                       (data.get('last_rejection') or {}).get('session'))
                if phase!=self.last_status:
                    self.get_logger().info('BRIDGE '+json.dumps(data))
                    self.last_status=phase
            except (ValueError,AttributeError):
                pass

    node=None
    try:
        node=TestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if node.lock:node.lock.close()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def go_front(): main('front')
def go_right(): main('right')
def go_back(): main('back')
def go_left(): main('left')
def waypoint_square(): main('square')
def waypoint_circle(): main('circle')
