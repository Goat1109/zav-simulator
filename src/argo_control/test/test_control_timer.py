"""A slow simulation clock must not throttle PX4 heartbeat scheduling."""
import time
import uuid
from types import SimpleNamespace

import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
from rosgraph_msgs.msg import Clock
from rclpy.executors import SingleThreadedExecutor
from argo_control import argo_flight_manager as module


def test_control_remains_50hz_with_20hz_ros_clock(monkeypatch):
    prefix = '/test_control_' + uuid.uuid4().hex
    args = ['--ros-args', '-p', 'use_sim_time:=true']
    for topic in ('/clock', '/fmu/in/vehicle_command', '/fmu/in/offboard_control_mode',
                  '/fmu/in/trajectory_setpoint', '/fmu/out/vehicle_status',
                  '/fmu/out/vehicle_local_position', '/argo/lio_safety/land_requested',
                  '/argo/lio_safety/ready', '/argo/goal_pose', '/argo/cmd_vel', '/argo_super/execution_enabled'):
        args.extend(['-r', f'{topic}:={prefix}{topic}'])
    rclpy.init(args=args)
    calls = []
    monkeypatch.setattr(module.ARGOFlightManager, '_control_loop', lambda self: calls.append(time.monotonic()))
    monkeypatch.setattr(module.ARGOFlightManager, '_print_guide', lambda self: None)
    monkeypatch.setattr(module, 'Win32MultiKeyReader', lambda: SimpleNamespace(stop=lambda: None))
    manager = module.ARGOFlightManager()
    driver = rclpy.create_node('slow_clock_driver')
    pub = driver.create_publisher(Clock, '/clock', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(driver)
    start = time.monotonic()
    next_clock = start
    try:
        while time.monotonic()-start < 2.5:
            now = time.monotonic()
            if now >= next_clock:
                clock = Clock()
                sim = 100.0 + (now-start)
                clock.clock.sec = int(sim)
                clock.clock.nanosec = int((sim-int(sim))*1e9)
                pub.publish(clock)
                next_clock += .05
            executor.spin_once(timeout_sec=.002)
        settled = [t for t in calls if t>=start+.5]
        assert len(settled)>=85, f'control timer was throttled: {len(settled)/2:.1f}Hz'
        assert manager.get_clock().now().nanoseconds > 100_000_000_000
        assert manager.get_parameter('use_sim_time').value is True
    finally:
        executor.shutdown()
        manager.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
