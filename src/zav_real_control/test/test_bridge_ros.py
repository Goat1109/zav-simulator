"""Isolated host ROS callback integration; never connects to an Agent, FC or simulator."""
import json
import time
import uuid
from pathlib import Path

import pytest
import yaml
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs')
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus,VehicleLocalPosition,TimesyncStatus,EstimatorAidSource1d,VehicleCommand,VehicleOdometry
from zav_real_control.bridge import RealBridge


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def test_ev_cut_baro_handoff_land_and_no_arming(tmp_path):
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/dds_legacy.yaml').read_text())
    token='/isolated_test_'+uuid.uuid4().hex
    c.update(px4_prefix=token+'/fmu',status_prefix=token+'/state',odom_topic=token+'/odom',
             publish_enabled=True,firmware_abi_verified=True)
    path=tmp_path/'hardware.yaml';path.write_text(yaml.safe_dump(c))
    rclpy.init(args=['--ros-args','-p',f'config_file:={path}'])
    node=None
    try:
        node=RealBridge()
        node.ev=Capture();node.command=Capture();node.baro_pub=Capture()
        node.count_publishers=lambda topic:1
        def status():
            msg=VehicleStatus(timestamp=time.time_ns()//1000,system_id=1,component_id=1,
                              arming_state=VehicleStatus.ARMING_STATE_ARMED,nav_state=2,
                              nav_state_user_intention=2)
            node.on_status(msg)
            node.on_sync(TimesyncStatus(timestamp=msg.timestamp,source_protocol=2,round_trip_time=100))
        def parameter(value):
            node.on_parameter(String(data=json.dumps(dict(parameter='EKF2_BARO_CTRL',target=[1,1],
                   firmware_supported=True,confirmed=True,value=value))))
        def odom(x):
            msg=Odometry();stamp=time.time_ns()
            msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
            msg.header.frame_id='camera_init';msg.child_frame_id='mid360_imu'
            msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=float(x)
            node.on_odom(msg)
            return stamp//1000
        status();parameter(0)
        stamp=odom(0)
        assert len(node.ev.messages)==1
        assert node.ev.messages[0].timestamp_sample==stamp
        assert node.ev.messages[0].pose_frame==VehicleOdometry.POSE_FRAME_FRD  # FRD, arbitrary heading
        odom(2)
        assert node.motion.latched and len(node.ev.messages)==1
        assert not node.command.messages
        node.last_report=-float('inf');node.tick()
        assert node.baro_pub.messages[-1].data
        parameter(1);node.tick()
        assert not node.command.messages  # parameter ACK is insufficient
        now=time.time_ns()//1000
        node.on_aid(EstimatorAidSource1d(timestamp=now,timestamp_sample=now,time_last_fuse=now,
                      fused=True,fusion_enabled=True,test_ratio=.1))
        node.tick();assert not node.command.messages
        node.on_local(VehicleLocalPosition(timestamp=now,z_valid=True,v_z_valid=True,z=-1.,vz=0.))
        node.tick()
        assert node.handoff.confirmed
        assert [msg.command for msg in node.command.messages]==[VehicleCommand.VEHICLE_CMD_NAV_LAND]
        assert node.command.messages[0].target_system==1
        assert node.command.messages[0].from_external
        odom(0)
        assert len(node.ev.messages)==1  # latch does not silently resume EV
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
