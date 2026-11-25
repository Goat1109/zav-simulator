"""Real RC flight gateway. Publishes EV and failsafe LAND only: no arm or offboard commands."""
import fcntl
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from px4_msgs.msg import (VehicleOdometry,VehicleStatus,VehicleLocalPosition,VehicleLandDetected,
                          VehicleCommand,VehicleCommandAck,TimesyncStatus,EstimatorAidSource1d)
from .config import load_config
from .geometry import body_odometry
from .clock import validate_sample,validate_dds_clock
from .motion import LioSafety,SafetyConfig
from .handoff import Handoff


class RealBridge(Node):
    def __init__(self):
        super().__init__('zav_real_lio_bridge')
        self.c=load_config(self.declare_parameter('config_file','').value)
        self.live=bool(self.declare_parameter('publish_enabled',self.c['publish_enabled']).value)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Real bridge requires use_sim_time=false')
        motion=dict(self.c['motion'])
        motion['angle_slack']=math.radians(motion.pop('angle_slack_deg'))
        self.motion=LioSafety(SafetyConfig(**motion))
        self.handoff=Handoff(self.c['baro_activation_timeout_s'],self.c['respect_rc_takeover'])
        self.status=self.local=self.landed=self.aid=self.parameter=self.sync=None
        self.status_rx=self.local_rx=self.landed_rx=self.aid_rx=self.parameter_rx=self.sync_rx=None
        self.good_baro_rx=None
        self.runtime_firmware_ok=False
        self.last_source_us=0
        self.last_input_rx=None
        self.last_out=-math.inf
        self.last_report=-math.inf
        self.last_state=None
        self.ack=None
        self.counts=dict(rx=0,out=0,dropped=0,land_requests=0)
        self.input_reason='waiting_for_odometry'
        self.last_geometry=None
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        prefix=self.c['px4_prefix']
        self.ev_topic=prefix+'/in/vehicle_visual_odometry'
        self.ev=self.create_publisher(VehicleOdometry,self.ev_topic,qos) if self.live else None
        self.command=self.create_publisher(VehicleCommand,prefix+'/in/vehicle_command',qos) if self.live else None
        self.report_pub=self.create_publisher(String,self.c['status_prefix']+'/status',latched)
        self.baro_pub=self.create_publisher(Bool,self.c['status_prefix']+'/baro_enable',latched)
        self.create_subscription(Odometry,self.c['odom_topic'],self.on_odom,qos)
        for typ,topic,callback in [
            (VehicleStatus,'vehicle_status',self.on_status),
            (VehicleLocalPosition,'vehicle_local_position',self.on_local),
            (VehicleLandDetected,'vehicle_land_detected',self.on_landed),
            (TimesyncStatus,'timesync_status',self.on_sync),
            (EstimatorAidSource1d,'estimator_aid_src_baro_hgt',self.on_aid),
            (VehicleCommandAck,'vehicle_command_ack',self.on_ack)]:
            self.create_subscription(typ,prefix+'/out/'+topic,callback,qos)
        self.create_subscription(String,self.c['status_prefix']+'/baro_parameter',self.on_parameter,latched)
        self.create_timer(.05,self.tick)
        self.get_logger().info(f'RC flight bridge; live={self.live}; no arm/takeoff/offboard setpoints; '
                               f'input={self.c["odom_topic"]}')

    def on_status(self,msg):
        if self.status and msg.timestamp<=self.status.timestamp:
            return
        self.status,self.status_rx=msg,time.monotonic()
        self.motion.set_armed(msg.arming_state==VehicleStatus.ARMING_STATE_ARMED)

    def on_local(self,msg):
        if self.local is None or msg.timestamp>self.local.timestamp:
            self.local,self.local_rx=msg,time.monotonic()

    def on_landed(self,msg):
        if self.landed is None or msg.timestamp>self.landed.timestamp:
            self.landed,self.landed_rx=msg,time.monotonic()

    def on_sync(self,msg):
        if self.sync is None or msg.timestamp>self.sync['timestamp']:
            self.sync=dict(timestamp=msg.timestamp,source_protocol=msg.source_protocol,
                           round_trip_time=msg.round_trip_time)
            self.sync_rx=time.monotonic()

    def on_aid(self,msg):
        if msg.estimator_instance!=0 or (self.aid and msg.timestamp_sample<=self.aid.timestamp_sample):
            return
        self.aid,self.aid_rx=msg,time.monotonic()
        if (msg.fused and msg.fusion_enabled and not msg.innovation_rejected
                and msg.time_last_fuse>0 and math.isfinite(msg.test_ratio) and 0<=msg.test_ratio<=1):
            self.good_baro_rx=self.aid_rx

    def on_parameter(self,msg):
        try:
            data=json.loads(msg.data)
            if (not isinstance(data,dict) or data.get('parameter')!='EKF2_BARO_CTRL'
                    or self.status is None
                    or data.get('target')!=[self.status.system_id,self.status.component_id]):
                return
            self.parameter,self.parameter_rx=data,time.monotonic()
            self.runtime_firmware_ok=data.get('firmware_supported') is True
        except (ValueError,TypeError):
            pass

    def on_ack(self,msg):
        if (self.counts['land_requests'] and msg.command==VehicleCommand.VEHICLE_CMD_NAV_LAND
                and msg.target_system==self.c['source_system']
                and msg.target_component==self.c['source_component']):
            self.ack=int(msg.result)
            self.get_logger().info(f'LAND ACK={self.ack}; check actual nav_state, not ACK alone')

    def clock_reason(self,now):
        return validate_dds_clock(time.time_ns()//1000,self.status.timestamp if self.status else 0,
                                  self.status_rx,self.sync,self.sync_rx,now,self.c)

    def verified(self):
        return (self.c['calibration_verified'] and self.c['firmware_abi_verified']
                and self.runtime_firmware_ok)

    def drop(self,reason):
        self.input_reason=reason
        self.counts['dropped']+=1

    def on_odom(self,msg):
        now=time.monotonic()
        self.counts['rx']+=1
        self.last_input_rx=now
        stamp=msg.header.stamp.sec*1_000_000+msg.header.stamp.nanosec//1000
        if (msg.header.frame_id!=self.c['odom_parent'] or msg.child_frame_id!=self.c['odom_child']):
            self.drop('unexpected_odometry_frames');return
        reason=validate_sample(stamp,time.time_ns()//1000,self.last_source_us,self.c['max_odom_age_s'])
        if reason:
            self.drop(reason);return
        p,q,v,w=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        try:
            transformed=body_odometry((p.x,p.y,p.z),(q.w,q.x,q.y,q.z),(v.x,v.y,v.z),(w.x,w.y,w.z),
                        self.c['body_from_imu_q_wxyz'],self.c['imu_position_body_flu_m'])
        except (ValueError,TypeError,OverflowError):
            self.drop('invalid_odometry');return
        self.last_source_us=stamp
        self.last_geometry=transformed
        # Do not turn lack of clock/status feedback into a jump landing trigger.
        reason=self.clock_reason(now)
        if reason:
            self.drop(reason);return
        accepted=self.motion.odometry(stamp/1e6,(p.x,p.y,p.z),(q.w,q.x,q.y,q.z),(v.x,v.y,v.z))
        self.tick()
        if not accepted or self.motion.latched:
            self.drop('jump_latched' if self.motion.latched else self.motion.comparison);return
        self.input_reason=None
        if not self.live or not self.verified() or now-self.last_out<1/self.c['rate_hz']:
            return
        if self.count_publishers(self.ev_topic)>1:
            self.drop('duplicate_EV_publisher');return
        position,orientation,velocity,angular=transformed
        out=VehicleOdometry()
        out.timestamp=time.time_ns()//1000
        out.timestamp_sample=stamp  # DDS subtracts its offset exactly once inside PX4.
        out.pose_frame=VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame=VehicleOdometry.VELOCITY_FRAME_FRD
        out.position=list(position);out.q=list(orientation)
        out.velocity=list(velocity);out.angular_velocity=list(angular)
        out.position_variance=self.c['position_variance_m2']
        out.orientation_variance=self.c['orientation_variance_rad2']
        out.velocity_variance=self.c['velocity_variance_m2_s2']
        out.quality=0  # Unknown quality, not a fabricated 100% score. EKF2_EV_QMIN=0.
        out.reset_counter=0  # No in-flight reset/relocalization is silently approved.
        self.ev.publish(out)
        self.last_out=now;self.counts['out']+=1

    def tick(self):
        now=time.monotonic()
        fresh=lambda rx,limit: rx is not None and now-rx<limit
        status_ok=fresh(self.status_rx,self.c['max_feedback_age_s'])
        parameter_ok=(fresh(self.parameter_rx,2.) and self.parameter
                      and self.parameter.get('confirmed') is True)
        parameter_off=bool(parameter_ok and self.parameter.get('value')==0)
        aid_ok=fresh(self.aid_rx,1.)
        baro_off=parameter_off and aid_ok and not self.aid.fusion_enabled
        started=self.handoff.started
        parameter_on=bool(parameter_ok and self.parameter.get('value')==1
                          and started is not None and self.parameter_rx>=started)
        baro_good=(aid_ok and self.aid.fusion_enabled and fresh(self.good_baro_rx,1.)
                   and started is not None and self.good_baro_rx>=started)
        vertical_good=(fresh(self.local_rx,1.) and started is not None and self.local_rx>=started
                       and self.local.z_valid and self.local.v_z_valid
                       and math.isfinite(self.local.z) and math.isfinite(self.local.vz))
        result=self.handoff.update(now,self.motion.latched,self.motion.armed,bool(status_ok),
                    self.status.nav_state if self.status else None,
                    self.status.nav_state_user_intention if self.status else None,
                    parameter_on, bool(baro_good), bool(vertical_good))
        writable=self.live and self.verified()
        if result['send_land'] and writable:
            self.send_land()
        clock=self.clock_reason(now)
        checks=dict(calibration=self.c['calibration_verified'],firmware_abi=self.c['firmware_abi_verified'],
                    runtime_firmware=self.runtime_firmware_ok,clock=clock is None,
                    recent_odometry=fresh(self.last_input_rx,self.c['max_odom_age_s']) and self.input_reason is None,
                    baro_off_confirmed=bool(baro_off),single_ev_publisher=(not self.live or self.count_publishers(self.ev_topic)<=1))
        ready=all(checks.values()) and not self.motion.latched
        complete=bool(self.motion.latched and status_ok and not self.motion.armed
                      and fresh(self.landed_rx,2.) and self.landed.landed)
        data=dict(mode='LIVE' if self.live else 'MONITOR_ONLY',ready=ready,checks=checks,
                  blocked_by=[key for key,value in checks.items() if not value],clock_reason=clock,
                  input_reason=self.input_reason,latched=self.motion.latched,reasons=self.motion.latch_reasons,
                  comparison=self.motion.comparison,metrics=self.motion.metrics,
                  armed=self.motion.armed,nav_state=self.status.nav_state if self.status else None,
                  landing_complete=complete,land_ack=self.ack,counts=dict(self.counts),
                  baro_parameter_value=self.parameter.get('value') if parameter_ok else None,
                  baro_fusion_enabled=bool(self.aid.fusion_enabled) if aid_ok else None,**result)
        if now-self.last_report>=.2:
            if writable:
                self.baro_pub.publish(Bool(data=self.motion.latched))
            # Replace nonfinite diagnostics, never serialize NaN into a status document.
            data['metrics']={k:v if math.isfinite(v) else None for k,v in data['metrics'].items()}
            self.report_pub.publish(String(data=json.dumps(data,allow_nan=False)))
            state=(data['mode'],data['phase'],tuple(data['blocked_by']))
            if state!=self.last_state:
                self.get_logger().warning(f'{state}; jump={data["reasons"]}')
                self.last_state=state
            self.last_report=now
        self.snapshot=data

    def send_land(self):
        msg=VehicleCommand()
        msg.timestamp=time.time_ns()//1000
        msg.command=VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param4=msg.param5=msg.param6=msg.param7=math.nan
        msg.target_system=self.status.system_id
        msg.target_component=self.status.component_id
        msg.source_system=self.c['source_system'];msg.source_component=self.c['source_component']
        msg.from_external=True
        self.command.publish(msg)
        self.counts['land_requests']+=1
        self.get_logger().error(f'Latched LIO jump: LAND #{self.counts["land_requests"]}; '
                                f'{self.motion.latch_reasons}; phase={self.handoff.confirmed}')


def main(args=None):
    rclpy.init(args=args)
    node=None
    lock=open(f'/tmp/zav-real-EV-{os.getuid()}.lock','w')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        node=RealBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        lock.close()
        if rclpy.ok():
            rclpy.shutdown()
