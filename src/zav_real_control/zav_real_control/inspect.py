"""Read-only compact status viewer. No extra camera/odometry/serial subscriptions."""
import json
import sys
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from .status_view import render


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('zav_real_inspect')
    prefix = node.declare_parameter('status_prefix', '/zav_real').value
    detailed = node.declare_parameter('detailed', False).value
    state = {'data': None, 'rx': None}

    def receive(msg):
        try:
            state.update(data=json.loads(msg.data), rx=time.monotonic())
        except (ValueError, TypeError):
            return

    def display():
        age = time.monotonic()-state['rx'] if state['rx'] is not None else None
        if age is None or age > 2:
            text = '브리지 상태 수신 대기/끊김 — 현재 기체·배터리 상태 미확인'
        else:
            text = (json.dumps(state['data'], ensure_ascii=False, indent=2)
                    if detailed else render(state['data']))
        if sys.stdout.isatty():
            print('\033[2J\033[H' + text, end='\n', flush=True)
        else:
            print(text, flush=True)

    node.create_subscription(String, prefix+'/status', receive,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(1., display)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
