import math

import pytest
from nav_msgs.msg import Odometry
from zav_real_control.super_lio_adapter import adapt, CorrectionGate
from zav_real_control.geometry import rotate, body_odometry


def sample():
    msg = Odometry()
    msg.header.frame_id = 'world'
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.pose.pose.orientation.w = math.sqrt(.5)
    msg.pose.pose.orientation.z = math.sqrt(.5)
    msg.twist.twist.linear.x = 1.
    msg.twist.twist.angular.z = .2
    return msg


def test_yaw_velocity_and_stamp():
    msg = sample()
    out = adapt(msg)
    assert out.header.stamp == msg.header.stamp
    assert (out.header.frame_id, out.child_frame_id) == ('camera_init', 'mid360_imu')
    v = out.twist.twist.linear
    assert (v.x, v.y, v.z) == pytest.approx((0., -1., 0.), abs=1e-12)
    assert out.twist.twist.angular.z == .2
    assert msg.twist.twist.linear.x == 1.
    assert msg.header.frame_id == 'world'


def test_real_mount_lever_arm_removed_once():
    msg = sample()
    # Real mount: 22 degree pitch. Stationary body, IMU lever rotating at 1 rad/s.
    q = (math.cos(math.radians(11)), 0., math.sin(math.radians(11)), 0.)
    lever = (.03888, .02329, .15182)
    msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = q
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = lever
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = (-lever[1], lever[0], 0.)
    from zav_real_control.geometry import conj
    w = rotate(conj(q), (0., 0., 1.))
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = w
    out = adapt(msg)
    v = out.twist.twist.linear
    p, orientation, velocity, angular = body_odometry(lever, q, (v.x,v.y,v.z), w, q, lever)
    assert p == pytest.approx((0.,0.,0.), abs=1e-12)
    assert velocity == pytest.approx((0.,0.,0.), abs=1e-12)


def test_lidar_loss_and_out_of_order():
    gate = CorrectionGate(.25)
    assert not gate.ready(100., 1.)
    gate.correct(100., 1.)
    assert gate.ready(100.1, 1.1)
    assert not gate.ready(100.3, 1.1)
    assert not gate.ready(100.1, 1.3)
    assert not gate.ready(99.9, 1.1)
    gate.last_output = 100.1
    assert not gate.ready(100.1, 1.1)
    gate.correct(99., 1.2)
    assert gate.correction == 100.


def test_bad_source_and_values():
    msg = sample()
    msg.header.frame_id = 'unexpected'
    with pytest.raises(ValueError): adapt(msg)
    msg.header.frame_id = 'world'
    msg.twist.twist.linear.x = math.nan
    with pytest.raises(ValueError): adapt(msg)
