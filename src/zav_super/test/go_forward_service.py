#!/usr/bin/env python3
"""Synthetic service test; requires a device-free container with --network none."""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '218'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert all(name == 'lo' for _, name in socket.if_nameindex())
assert not list(Path('/dev').glob('ttyACM*')) + list(Path('/dev').glob('ttyUSB*'))

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from zav_super.srv import GoForward


def run():
    rclpy.init()
    node = rclpy.create_node('offline_goal_service_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    odom = node.create_publisher(Odometry, '/argo/super/odom', 10)
    cloud = node.create_publisher(PointCloud2, '/argo/super/cloud_world', 1)
    bridge = node.create_publisher(String, '/zav_real/status', qos)
    goals, status = [], []
    node.create_subscription(PoseStamped, '/zav/super/goal', goals.append, 10)
    node.create_subscription(String, '/zav/super/controller_status',
                             lambda msg: status.append(json.loads(msg.data)), 10)
    client = node.create_client(GoForward, '/zav/super/go_forward')
    legacy = node.create_client(Trigger, '/zav/super/go_front')
    mode, speed = [3, 0], 0.

    def publish():
        stamp = node.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'world', 'argo_body'
        msg.header.stamp = stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1., 2., 1.2
        msg.pose.pose.orientation.w = msg.pose.pose.orientation.z = math.sqrt(.5)
        msg.twist.twist.linear.x = speed
        odom.publish(msg)
        scan = PointCloud2()
        scan.header.frame_id, scan.header.stamp = 'world', stamp
        scan.height = scan.width = 1
        cloud.publish(scan)
        bridge.publish(String(data=json.dumps({'ready': True, 'latched': False,
            'offboard': {'source': 'super', 'stopped': False, 'fc_mode': mode}})))

    node.create_timer(.02, publish)

    def until(predicate, timeout=8.):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert predicate(), 'Timed out waiting for synthetic controller state'

    def request(distance=None):
        if distance is None:
            future = legacy.call_async(Trigger.Request())
        else:
            future = client.call_async(GoForward.Request(distance_m=distance))
        until(future.done)
        return future.result()

    def hover():
        nonlocal mode
        mode = [3, 0]
        until(lambda: status and status[-1]['phase'] == 'PRESTREAM')
        mode = [6, 0]
        until(lambda: status[-1]['phase'] == 'HOVER')

    binary = Path(get_package_prefix('zav_super')) / 'lib/zav_super/super_ommpc'
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '-p', 'execute:=true'],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            until(lambda: client.service_is_ready() and legacy.service_is_ready())
            until(lambda: status and status[-1]['phase'] == 'PRESTREAM')
            assert not request(3.5).success
            hover()
            for distance in (0., -1., 20.01, float('nan'), float('inf')):
                response = request(distance)
                assert not response.success and 'distance_m' in response.message
            assert not goals
            speed = .3
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.02)
            assert not request(3.5).success
            speed = 0.
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.02)
            response = request(3.5)
            assert response.success, response.message
            until(lambda: len(goals) == 1)
            point = goals[-1].pose.position
            assert abs(point.x-1.) < 1e-6 and abs(point.y-5.5) < 1e-6 and abs(point.z-1.2) < 1e-6
            assert not request(2.).success
            assert not request().success
            hover()
            assert request().success
            until(lambda: len(goals) == 2)
            assert abs(goals[-1].pose.position.y-12.) < 1e-6
            hover()
            assert request(20.).success
            until(lambda: len(goals) == 3)
            assert abs(goals[-1].pose.position.y-22.) < 1e-6
            print('PASS: distance, heading/height, bounds, hover/speed guards, one goal per entry, legacy default')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
