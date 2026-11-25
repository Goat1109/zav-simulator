"""One MAVLink connection owns every read/write. No ROS, UART creation or arm command here."""
from collections import deque
import math
import time
from pymavlink.dialects.v20 import common as mav
from .telemetry import FlightTelemetry
from .clock import validate_sample
from .geometry import body_odometry, rotate, conj
from .handoff import Handoff
from .mavlink_parameters import ParameterLink, decode, INT32
from .motion import LioSafety, SafetyConfig
from .offboard import OffboardControl


def covariance(diagonal):
    result=[0.]*21
    for i,value in zip((0,6,11,15,18,20),diagonal):result[i]=value
    return result


class BufferedLink:
    def __init__(self, connection):
        self.connection=connection
        self.queue=deque()
    @property
    def mav(self):return self.connection.mav
    def recv_match(self,blocking=False):return self.queue.popleft() if self.queue else None


class SingleLink:
    def __init__(self, connection, config, live=False):
        self.connection,self.c,self.live=connection,config,live
        self.buffer=BufferedLink(connection)
        self.parameters=ParameterLink(self.buffer,config['expected_firmware'])
        self.parameters.target=(config['mavlink']['target_system'],config['mavlink']['target_component'])
        motion=dict(config['motion'])
        motion['angle_slack']=math.radians(motion.pop('angle_slack_deg'))
        self.motion=LioSafety(SafetyConfig(**motion))
        self.handoff=Handoff(config['baro_activation_timeout_s'],False)
        self.hb_rx=self.sync_rx=self.baro_rx=self.local_rx=self.est_rx=self.landed_rx=None
        self.sync_ok=False
        self.baro_bits=None
        self.local=None
        self.estimator_flags=0
        self.landed=None
        self.mode=None
        self.armed=True  # Unknown is never treated as disarmed for baro-OFF writes.
        self.last_stamp=0
        self.input_rx=None
        self.usable_input_rx=None
        self.usable_input_stamp=None
        self.input_reason='waiting_for_odometry'
        self.last_out=self.last_heartbeat=self.last_request=-math.inf
        self.requests=deque()
        self.timesync_interval_us=None
        self.last_timesync_request=-math.inf
        self.rc_value=None
        self.rc_at_latch=None
        self.rc_rx=None
        self.rc_mapping=0
        self.rc_mapping_rx=None
        self.rc_mapping_changed=False
        self.last_rc_request=-math.inf
        self.diag_stamps={}
        self.boot_ms=None
        self.reboot_detected=False
        self.error=None
        self.clock_step_detected=False
        self.last_clock=None
        self.ack=None
        self.counts=dict(rx=0,out=0,dropped=0,land_requests=0)
        self.telemetry=FlightTelemetry()
        self.offboard=OffboardControl()

    def fresh(self,rx,now,age=None):
        return rx is not None and 0<=now-rx<(age or self.c['max_feedback_age_s'])

    def writable(self):
        return (self.live and self.c['firmware_verified'] and self.c['calibration_verified']
                and self.parameters.supported and not self.reboot_detected and not self.clock_step_detected
                and not self.rc_mapping_changed)

    def mode_channel(self,now):
        configured=self.c['mavlink']['rc_mode_channel']
        if configured:return configured
        return self.rc_mapping if self.fresh(self.rc_mapping_rx,now,5.) else 0

    def receive(self,msg,now,epoch_ns):
        if (msg.get_srcSystem(),msg.get_srcComponent())!=self.parameters.target:return
        kind=msg.get_type()
        if self.offboard.enabled:
            self.offboard.receive(msg,now)
        self.telemetry.receive(msg,now)
        if kind=='HEARTBEAT':
            if msg.autopilot!=mav.MAV_AUTOPILOT_PX4:return
            self.buffer.queue.append(msg)
            self.hb_rx=now
            self.armed=bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            self.motion.set_armed(self.armed)
            self.mode=((int(msg.custom_mode)>>16)&255,(int(msg.custom_mode)>>24)&255)
        elif kind in ('AUTOPILOT_VERSION','PARAM_VALUE'):
            self.buffer.queue.append(msg)
            if kind=='PARAM_VALUE' and msg.param_type==INT32:
                name=msg.param_id.decode() if isinstance(msg.param_id,bytes) else msg.param_id
                if name.rstrip('\0')=='RC_MAP_FLTMODE':
                    channel=decode(msg.param_value)
                    if 0<=channel<=18:
                        if self.rc_mapping and channel!=self.rc_mapping and self.armed:
                            self.rc_mapping_changed=True
                        self.rc_mapping,self.rc_mapping_rx=channel,now
        elif kind=='TIMESYNC' and msg.tc1==0:
            # Respond in the SAME epoch clock used by ODOMETRY. PX4 owns offset conversion.
            self.connection.mav.timesync_send(epoch_ns,msg.ts1)
        elif kind=='NAMED_VALUE_FLOAT':
            name=msg.name.decode() if isinstance(msg.name,bytes) else msg.name
            name=name.rstrip('\0')
            if name not in ('zav_sync','zav_baro') or not math.isfinite(msg.value):return
            previous=self.diag_stamps.get(name)
            if previous is not None and msg.time_boot_ms<=previous:return
            self.diag_stamps[name]=msg.time_boot_ms
            if name=='zav_sync':
                self.sync_rx=now;self.sync_ok=msg.value==1.
            elif msg.value in range(8):
                self.baro_rx=now;self.baro_bits=int(msg.value)
        elif kind=='SYSTEM_TIME':
            if self.boot_ms is not None and msg.time_boot_ms+1000<self.boot_ms:
                self.reboot_detected=True  # Ground restart required; never silently reuse an FC session.
                self.sync_ok=False
            self.boot_ms=msg.time_boot_ms
        elif kind=='LOCAL_POSITION_NED':
            if all(math.isfinite(v) for v in (msg.z,msg.vz)):
                self.local_rx,self.local=now,(msg.z,msg.vz)
        elif kind=='ESTIMATOR_STATUS':
            self.est_rx,self.estimator_flags=now,msg.flags
        elif kind=='EXTENDED_SYS_STATE':
            self.landed_rx,self.landed=now,msg.landed_state==mav.MAV_LANDED_STATE_ON_GROUND
        elif kind=='RC_CHANNELS':
            channel=self.mode_channel(now)
            value=getattr(msg,f'chan{channel}_raw',65535) if channel else 65535
            if 800<=value<=2200:
                self.rc_rx,self.rc_value=now,value
        elif kind=='COMMAND_ACK' and msg.command==mav.MAV_CMD_NAV_LAND:
            if (self.counts['land_requests'] and msg.target_system in (0,self.c['source_system'])
                    and msg.target_component in (0,self.c['source_component'])):
                self.ack=msg.result

    def clock_good(self,now):
        return self.sync_ok and self.fresh(self.sync_rx,now,self.c['max_timesync_age_s'])

    def begin_handoff(self,now):
        if self.handoff.started is None:
            self.rc_at_latch=self.rc_value if self.fresh(self.rc_rx,now) else None
            self.handoff.update(now,True,self.armed,True,None,None,False,False,False)

    def check_ev_input(self,now,epoch_us):
        # Only an armed live session that has emitted EV can initiate this fallback.
        if not (self.live and self.armed and self.counts['out'] and not self.motion.latched):
            return
        limit=self.c['max_odom_age_s']
        reason=None
        if self.usable_input_rx is not None and now-self.usable_input_rx>limit:
            reason='ev_input_timeout'
        elif self.usable_input_stamp is not None and (epoch_us-self.usable_input_stamp)/1e6>limit:
            reason='ev_measurement_stale'
        if reason:
            self.motion.latched=True
            self.motion.latch_reasons=[reason]
            self.begin_handoff(now)

    def odometry(self,stamp_us,position,q,velocity,angular,now,epoch_us,frames):
        # Check before accepting a resumed stream: a gap must not erase a failure.
        self.check_ev_input(now,epoch_us)
        self.counts['rx']+=1;self.input_rx=now
        self.telemetry.odometry(now,stamp_us,epoch_us)
        reason=validate_sample(stamp_us,epoch_us,self.last_stamp,self.c['max_odom_age_s'])
        if frames!=(self.c['odom_parent'],self.c['odom_child']):reason='unexpected_odometry_frames'
        try:
            transformed=body_odometry(position,q,velocity,angular,
                       self.c['body_from_imu_q_wxyz'],self.c['imu_position_body_flu_m'])
        except (ValueError,TypeError,OverflowError):reason='invalid_odometry'
        if not reason:
            self.last_stamp=stamp_us
            if not self.fresh(self.hb_rx,now,self.c['mavlink']['heartbeat_timeout_s']):reason='heartbeat_missing_or_stale'
            elif not self.clock_good(now):reason='fc_timesync_missing_or_not_converged'
            elif self.reboot_detected:reason='fc_reboot_ground_restart_required'
            elif self.clock_step_detected:reason='host_clock_step_ground_restart_required'
            elif self.rc_mapping_changed:reason='rc_mapping_changed_ground_restart_required'
        if not reason:
            accepted=self.motion.odometry(stamp_us/1e6,position,q,velocity)
            if self.motion.latched and self.handoff.started is None:
                self.begin_handoff(now)
            if not accepted or self.motion.latched:
                reason='jump_latched' if self.motion.latched else self.motion.comparison
        self.input_reason=reason
        if reason:
            self.counts['dropped']+=1;return False
        self.usable_input_rx,self.usable_input_stamp=now,stamp_us
        if not self.writable() or now-self.last_out<1/self.c['rate_hz']:return False
        # Respect pilot takeover setup before emitting EV on a live vehicle.
        if self.c['respect_rc_takeover'] and not self.mode_channel(now):return False
        p,orientation,world_velocity,body_angular=transformed
        body_velocity=rotate(conj(orientation),world_velocity)
        pose_cov=covariance([*self.c['position_variance_m2'],*self.c['orientation_variance_rad2']])
        velocity_cov=covariance([*self.c['velocity_variance_m2_s2'],math.nan,math.nan,math.nan])
        # PX4 v1.14 rejects MAV_ESTIMATOR_TYPE_LIDAR; UNKNOWN is accepted for external EV.
        self.connection.mav.odometry_send(stamp_us,mav.MAV_FRAME_LOCAL_FRD,mav.MAV_FRAME_BODY_FRD,
            *p,list(orientation),*body_velocity,*body_angular,pose_cov,velocity_cov,
            reset_counter=0,estimator_type=mav.MAV_ESTIMATOR_TYPE_UNKNOWN,quality=0)
        self.last_out=now;self.counts['out']+=1
        return True

    def update(self,now,epoch_ns):
        if self.last_clock is not None:
            previous_now,previous_ns=self.last_clock
            if abs((epoch_ns-previous_ns)/1e9-(now-previous_now))>.05:
                self.clock_step_detected=True
        self.last_clock=(now,epoch_ns)
        try:
            receive_deadline=time.monotonic()+.002
            for _ in range(64):
                msg=self.connection.recv_match(blocking=False)
                if msg is None:break
                self.receive(msg,now,epoch_ns)
                if time.monotonic()>=receive_deadline:break
            heartbeat_ok=self.fresh(self.hb_rx,now,self.c['mavlink']['heartbeat_timeout_s'])
            if now-self.last_heartbeat>=1:
                self.connection.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER,
                   mav.MAV_AUTOPILOT_INVALID,0,0,mav.MAV_STATE_ACTIVE)
                self.last_heartbeat=now
            if heartbeat_ok and not self.requests and now-self.last_request>=10:
                # One request per tick, not a burst that can clog the UART.
                self.requests.extend(((mav.MAVLINK_MSG_ID_ESTIMATOR_STATUS,100000),
                    (mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED,100000),
                    (mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,200000),
                    (mav.MAVLINK_MSG_ID_RC_CHANNELS,100000),
                    (mav.MAVLINK_MSG_ID_SYSTEM_TIME,1000000),
                    (mav.MAVLINK_MSG_ID_BATTERY_STATUS,1000000)))
                self.last_request=now
                if self.offboard.enabled:
                    self.requests.append((mav.MAVLINK_MSG_ID_ATTITUDE,100000))
                    if getattr(self.offboard,'requires_hover_thrust',False):
                        self.requests.append((mav.MAVLINK_MSG_ID_ATTITUDE_TARGET,50000))
            # React immediately to the FC's actual convergence diagnostic, not
            # a fixed five-second timer. Stale/lost sync returns to the fast rate.
            rate_key='timesync_steady_rate_hz' if self.clock_good(now) else 'timesync_initial_rate_hz'
            sync_rate=self.c['mavlink'][rate_key]
            sync_interval=round(1e6/sync_rate)
            sync_requested=False
            if heartbeat_ok and (sync_interval!=self.timesync_interval_us
                                 or now-self.last_timesync_request>=10):
                self.connection.mav.command_long_send(*self.parameters.target,
                    mav.MAV_CMD_SET_MESSAGE_INTERVAL,0,mav.MAVLINK_MSG_ID_TIMESYNC,
                    sync_interval,0,0,0,0,0)
                self.timesync_interval_us=sync_interval
                self.last_timesync_request=now
                sync_requested=True
            if heartbeat_ok and self.requests and not sync_requested:
                message,interval=self.requests.popleft()
                self.connection.mav.command_long_send(*self.parameters.target,
                     mav.MAV_CMD_SET_MESSAGE_INTERVAL,0,message,interval,0,0,0,0,0)
            if heartbeat_ok and now-self.last_rc_request>=2:
                self.connection.mav.param_request_read_send(*self.parameters.target,b'RC_MAP_FLTMODE',-1)
                self.last_rc_request=now
            self.check_ev_input(now,epoch_ns//1000)
            self.parameters.tick(int(self.motion.latched),self.armed,now,
                                 self.writable() and heartbeat_ok)
            self.error=None
            self.land_tick(now)
            self.offboard.tick(self,now)
        except (OSError,ValueError) as error:
            self.fail_io(error)
        return self.snapshot(now)

    def fail_io(self,error):
        if self.offboard.enabled:
            self.offboard.abort('serial_io_error')
        self.error=str(error)
        self.hb_rx=self.sync_rx=self.baro_rx=None
        self.parameters.value_rx=self.parameters.heartbeat_rx=None
        self.sync_ok=False

    def land_tick(self,now):
        p=self.parameters.snapshot(1,now)
        started=self.handoff.started
        after=lambda rx:started is not None and rx is not None and rx>=started
        parameter_on=p['confirmed'] and after(self.parameters.value_rx)
        baro_good=(self.fresh(self.baro_rx,now) and after(self.baro_rx) and self.baro_bits==3)
        # Flags explicitly indicate vertical velocity and absolute vertical position validity.
        mask=mav.ESTIMATOR_VELOCITY_VERT | mav.ESTIMATOR_POS_VERT_ABS
        vertical=(self.fresh(self.local_rx,now) and self.fresh(self.est_rx,now)
                  and after(self.local_rx) and after(self.est_rx) and self.estimator_flags&mask==mask)
        if (self.motion.latched and self.c['respect_rc_takeover'] and self.rc_at_latch is not None
                and self.fresh(self.rc_rx,now) and self.rc_value is not None
                and abs(self.rc_value-self.rc_at_latch)>self.c['mavlink']['rc_switch_delta_us']
                and self.mode and self.mode[0] in (1,2,7,8)):
            self.handoff.pilot_takeover=True
        nav_state=18 if self.mode==(4,6) else None  # PX4 AUTO.LAND
        result=self.handoff.update(now,self.motion.latched,self.armed,self.fresh(self.hb_rx,now,self.c['mavlink']['heartbeat_timeout_s']),
                   nav_state,None,parameter_on,bool(baro_good),bool(vertical))
        if result['send_land'] and self.writable():
            self.connection.mav.command_long_send(*self.parameters.target,mav.MAV_CMD_NAV_LAND,0,
                0,0,0,math.nan,math.nan,math.nan,math.nan)
            self.counts['land_requests']+=1
        return result

    def snapshot(self,now):
        p=self.parameters.snapshot(int(self.motion.latched),now)
        checks=dict(calibration=self.c['calibration_verified'],firmware=self.c['firmware_verified'],
            runtime_firmware=self.parameters.supported,heartbeat=self.fresh(self.hb_rx,now,self.c['mavlink']['heartbeat_timeout_s']),
            clock=self.clock_good(now),fc_session=not self.reboot_detected,host_clock=not self.clock_step_detected,
            recent_odometry=self.fresh(self.input_rx,now,self.c['max_odom_age_s']) and self.input_reason is None,
            baro_off_confirmed=p['confirmed'] and p['value']==0 and self.fresh(self.baro_rx,now) and self.baro_bits==0,
            rc_takeover=(not self.c['respect_rc_takeover'] or
                         bool(self.mode_channel(now)) and self.fresh(self.rc_rx,now) and not self.rc_mapping_changed),
            serial_io=self.error is None)
        metrics={k:v if math.isfinite(v) else None for k,v in self.motion.metrics.items()}
        return dict(mode='LIVE' if self.live else 'MONITOR_ONLY',transport='single_mavlink',
            offboard=self.offboard.snapshot(now),
            telemetry=self.telemetry.snapshot(now),odom_age_limit_ms=self.c['max_odom_age_s']*1000,
            ready=all(checks.values()) and not self.motion.latched,checks=checks,
            blocked_by=[k for k,v in checks.items() if not v],latched=self.motion.latched,
            reasons=list(self.motion.latch_reasons),metrics=metrics,input_reason=self.input_reason,
            armed=self.armed,px4_mode=self.mode,counts=dict(self.counts),baro_parameter=p,
            landed=self.landed,landed_fresh=self.fresh(self.landed_rx,now,.5),
            rc_mode_switch=self.c['mavlink'].get('rc_mode_switch'),rc_mode_channel=self.mode_channel(now),
            baro_fusion_bits=self.baro_bits if self.fresh(self.baro_rx,now) else None,
            baro_confirmed=self.handoff.confirmed,baro_timeout=self.handoff.timed_out,
            pilot_takeover=self.handoff.pilot_takeover,land_ack=self.ack,error=self.error,
            phase=('PILOT_TAKEOVER' if self.handoff.pilot_takeover else 'BARO_ACTIVE' if self.handoff.confirmed else
                   'BARO_TIMEOUT' if self.handoff.timed_out else 'ENABLING_BARO' if self.motion.latched else 'NORMAL_EV_ONLY'),
            landing_complete=bool(self.motion.latched and self.fresh(self.hb_rx,now,self.c['mavlink']['heartbeat_timeout_s']) and not self.armed
                                  and self.fresh(self.landed_rx,now) and self.landed))
