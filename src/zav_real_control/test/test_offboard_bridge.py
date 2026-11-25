"""Bridge callback tests. CheckedSerial is replaced BEFORE node creation."""
import json
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
rclpy=pytest.importorskip('rclpy')
from std_msgs.msg import String
from pymavlink.dialects.v20 import common as mav
from zav_real_control.offboard import Options
from dataclasses import asdict


class FakeConnection:
    def __init__(self):
        self.calls=[];self.mav=self
    def recv_match(self,blocking=False): return None
    def __getattr__(self,name):
        return lambda *a,**kw:self.calls.append((name,a,kw))


@pytest.mark.parametrize('enabled',[False,True])
def test_requests_require_explicit_bridge_enable_without_serial_access(tmp_path,monkeypatch,enabled):
    import zav_real_control.serial_link as serial_link
    from zav_real_control.mavlink_node import MavlinkBridge
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())
    c['mavlink']['connection']='/NEVER_OPEN_A_PHYSICAL_DEVICE'
    path=tmp_path/'hardware.yaml';path.write_text(yaml.safe_dump(c))
    connection=FakeConnection();opens=[]
    def fake(*args,**kw):opens.append((args,kw));return connection
    monkeypatch.setattr(serial_link,'CheckedSerial',fake)
    rclpy.init(args=['--ros-args','-p',f'config_file:={path}','-p',f'offboard_enabled:={str(enabled).lower()}'])
    node=None
    try:
        node=MavlinkBridge();node.count_publishers=lambda topic:1
        request=dict(session='a'*32,sequence=1,stamp_ns=time.time_ns(),pattern='square',options=asdict(Options()))
        node.offboard_request(String(data=json.dumps(request)))
        assert node.core.offboard.session is None
        assert bool(node.core.offboard.last_rejection)==enabled
        assert len(opens)==1 and opens[0][0][0]=='/NEVER_OPEN_A_PHYSICAL_DEVICE'
        # Node construction/request alone emits no flight commands or setpoints.
        assert connection.calls==[]
        if enabled:
            node.count_publishers=lambda topic:2
            node.offboard_request(String(data=json.dumps(request)))
            assert node.core.offboard.terminal
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


@pytest.mark.parametrize('source,enabled',[('super',True),('super',False),('legacy',True)])
def test_super_source_is_exclusive_without_opening_serial(tmp_path,monkeypatch,source,enabled):
    import zav_real_control.serial_link as serial_link
    from zav_real_control.mavlink_node import MavlinkBridge
    from zav_real_control.super_gate import SuperRateGate
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())
    c['mavlink']['connection']='/NEVER_OPEN_A_PHYSICAL_DEVICE'
    path=tmp_path/'hardware.yaml';path.write_text(yaml.safe_dump(c))
    connection=FakeConnection()
    monkeypatch.setattr(serial_link,'CheckedSerial',lambda *a,**kw:connection)
    rclpy.init(args=['--ros-args','-p',f'config_file:={path}',
        '-p',f'offboard_enabled:={str(enabled).lower()}','-p',f'offboard_source:={source}'])
    node=None
    try:
        node=MavlinkBridge();node.count_publishers=lambda topic:1
        packet=dict(session='super-callback-isolated',sequence=1,stamp_ns=time.time_ns(),
            valid=True,rates_flu=[0.,0.,0.],acceleration=9.81,phase='HOVER',
            solver_ms=1.,position=[0.,0.,2.],goal=[7.,0.,2.])
        node.super_request(String(data=json.dumps(packet)))
        assert isinstance(node.core.offboard,SuperRateGate)==(source=='super')
        assert (node.core.offboard.session is not None)==(source=='super' and enabled)
        if source=='super' and enabled:
            # A legacy command cannot replace the selected SUPER control source.
            node.offboard_request(String(data='{}'))
            assert node.core.offboard.session==packet['session']
            node.count_publishers=lambda topic:2
            node.super_request(String(data=json.dumps(packet)))
            assert node.core.offboard.terminal
        assert connection.calls==[]
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
