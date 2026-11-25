"""ROS callback smoke test with a fake sole serial owner; never opens a device."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('pymavlink')
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav
from zav_real_control.mavlink_node import MavlinkBridge


class Connection:
    def __init__(self):self.calls=[];self.mav=self
    def recv_match(self,blocking=False):return None
    def __getattr__(self,name):return lambda *a,**kw:self.calls.append((name,a,kw))


def test_ros_subscriber_uses_exactly_one_mavlink_connection(tmp_path,monkeypatch):
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())
    c.update(firmware_verified=True,publish_enabled=True,status_prefix='/isolated_single_link_test',
             expected_firmware=[1,14,3])
    c['mavlink']['rc_mode_channel']=7
    c['mavlink']['connection']='/offline/not-a-device'
    path=tmp_path/'hardware.yaml';path.write_text(yaml.safe_dump(c))
    connection=Connection();opens=[]
    def connect(*a,**kw):opens.append((a,kw));return connection
    from zav_real_control import serial_link
    import serial
    def forbid_device(*a,**kw):
        raise AssertionError('Offline ROS test must never open a serial device')
    monkeypatch.setattr(serial,'Serial',forbid_device)
    monkeypatch.setattr(serial_link,'CheckedSerial',connect)
    rclpy.init(args=['--ros-args','-p',f'config_file:={path}'])
    node=None
    try:
        node=MavlinkBridge();core=node.core
        now=time.monotonic()
        def receive(kind,**kw):
            core.receive(SimpleNamespace(get_type=lambda:kind,get_srcSystem=lambda:1,
                get_srcComponent=lambda:1,**kw),now,time.time_ns())
        receive('HEARTBEAT',autopilot=mav.MAV_AUTOPILOT_PX4,base_mode=128,custom_mode=3<<16)
        receive('AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(14<<16)|(3<<8))
        core.parameters.poll(now)
        receive('NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1000,value=1.)
        receive('RC_CHANNELS',chan7_raw=1000)
        msg=Odometry();epoch=time.time_ns()
        msg.header.stamp.sec=epoch//10**9;msg.header.stamp.nanosec=epoch%10**9
        msg.header.frame_id='camera_init';msg.child_frame_id='mid360_imu'
        msg.pose.pose.orientation.w=1.
        node.odometry(msg)
        assert core.counts['out']==1
        assert len(opens)==1 and opens[0][0]==(c['mavlink']['connection'],)
        assert opens[0][1]['baud']==921600
        node.tick()
        assert json.loads(json.dumps(core.snapshot(time.monotonic()),allow_nan=False))['transport']=='single_mavlink'
        assert not any(a[0]=='command_long_send' and a[1][2]==400 for a in connection.calls)
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
    assert any(name=='close' for name,a,kw in connection.calls)
