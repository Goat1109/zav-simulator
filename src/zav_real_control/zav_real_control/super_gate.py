"""Single-UART OMMPC gate. No connection creation or parameter writes.

The normal profile never arms. The separate, explicitly enabled GroundTakeoff
profile can request normal mode/arm through the existing UART. RTH may request
LAND after its final approach; actual AUTO.LAND owns descent/disarm.

The existing EV/FC land handoff has priority. Position/Offboard ownership is
exclusive. A latched controller fault replaces rates with PX4 position hold
while requesting Position; the EV-loss landing handoff always takes priority.
"""
import math
from .geometry import frd
from .offboard import OffboardControl
from .mavlink_parameters import decode
from .hover_handoff import HoverHandoff
from .super_auto import GroundTakeoff
from .thrust_model import ThrustModel

PARAMS = ('MPC_THR_HOVER', 'THR_MDL_FAC', 'RC_MAP_OFFB_SW', 'COM_OF_LOSS_T',
          'COM_OBL_RC_ACT', 'ZAV_EV_GUARD', 'MC_ROLLRATE_MAX', 'MC_PITCHRATE_MAX',
          'MC_YAWRATE_MAX', 'MPC_XY_VEL_MAX', 'MPC_VEL_MANUAL')
QUERY_PARAMS = PARAMS + ('COM_DISARM_LAND','RC_MAP_THROTTLE')


class SuperRateGate(OffboardControl):
    requires_hover_thrust = True

    def __init__(self, enabled=False, automatic_takeoff=False, hover_thrust=0., takeoff_height=.4, fresh_tracking_output=False, adaptive_thrust=False):
        super().__init__(enabled)
        self.fresh_tracking_output = fresh_tracking_output
        self.thrust_model = ThrustModel() if adaptive_thrust else None
        self.next_rate_slot = None
        self.last_sent_command = None
        self.cache = {}
        self.last_query = -math.inf
        self.query_index = 0
        self.command = None
        self.last_received = None
        self.rate_limit = (1.5, 1.5, .8)
        self.hover_thrust = None
        self.normalized_thrust = None
        self.hover_handoff = HoverHandoff()
        self.attitude_target = None
        self.roll_pitch = None
        self.reason = 'SUPER_WAITING' if enabled else 'disabled'
        self.tracking_backend="ommpc"
        self.px4_frame=None
        self.last_px4_target=None
        self.rc_ground = None
        self.automatic = GroundTakeoff(automatic_takeoff,hover_thrust,takeoff_height)
        self.landing_requested = self.landing_last_request = None
        self.landing_requests = 0
        self.rc_channels={}
        self.pilot_axes = None
        self.recovery = None

    def tick(self, core, now):
        self._tick(core, now)
        if self.enabled and self.terminal:
            self.recover_position(core, now)

    def recover_position(self, core, now):
        """Single owner, latched fault: replace rates before waiting for mode ACK.

        Position setpoints enable PX4's position/attitude loops immediately in
        Offboard. Never repeat the failed controller's last angular rates.
        A mode heartbeat, rather than COMMAND_ACK alone, confirms handoff.
        """
        from pymavlink.dialects.v20 import common as mav
        if self.recovery is None:
            if (not self.in_offboard and self.prestream_since is None) or core.mode != (6, 0):
                return
            self.recovery = dict(stage='POSITION_REQUEST', started=now, last_send=None,
                                 last_request=None, target=None, sent=0)
        r = self.recovery
        if r['stage'] in ('CONFIRMED', 'PILOT_MODE', 'EV_HANDOFF'):
            return
        if getattr(core.handoff,'pilot_takeover',False):
            r['stage'] = 'PILOT_MODE'
            return
        # Preserve the existing higher-priority EV-loss LAND owner. Once it
        # takes control this recovery cannot resume on a transient healthy sample.
        flags = (mav.ESTIMATOR_ATTITUDE | mav.ESTIMATOR_VELOCITY_HORIZ |
                 mav.ESTIMATOR_VELOCITY_VERT | mav.ESTIMATOR_POS_HORIZ_REL |
                 mav.ESTIMATOR_POS_VERT_ABS)
        healthy = (self.reason != 'fc_local_position_reset_or_jump' and
                   core.writable() and core.snapshot(now)['ready'] and
                   core.handoff.started is None and not core.motion.latched and
                   all(v is not None and 0 <= now-v < .5 for v in
                       (self.local_rx, self.att_rx, self.rc_rx, self.air_rx)) and
                   core.fresh(core.est_rx, now, .5) and core.estimator_flags & flags == flags)
        if not healthy:
            r['stage'] = 'EV_HANDOFF'
            return
        if core.mode != (6, 0) or not core.armed or not self.in_air:
            r['stage'] = 'CONFIRMED' if core.mode in ((3, 0), (4, 6)) else 'PILOT_MODE'
            return
        if r['target'] is None:
            target = (*self.position, self.yaw) if self.position is not None else ()
            if len(target) != 4 or not all(math.isfinite(v) for v in target):
                r['stage'] = 'EV_HANDOFF'
                return
            r['target'] = target
        if r['last_send'] is None or now-r['last_send'] >= .02-1e-9:
            self.send_position(core,now,r['target'])
            r['last_send'] = now
            r['sent'] += 1
        # If Position is denied for two seconds, request PX4 LAND while still
        # stabilizing. Stop our stream only after the FC actually leaves Offboard.
        mode = (3, 0) if now-r['started'] < 2. else (4, 6)
        r['stage'] = 'POSITION_REQUEST' if mode == (3, 0) else 'LAND_REQUEST'
        if r['last_request'] is None or now-r['last_request'] >= .5:
            core.connection.mav.command_long_send(*core.parameters.target,
                mav.MAV_CMD_DO_SET_MODE, 0, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                *mode, 0., 0., 0., 0.)
            r['last_request'] = now

    def send_position(self,core,now,target,velocity=None):
        from pymavlink.dialects.v20 import common as mav
        mask = (mav.POSITION_TARGET_TYPEMASK_VX_IGNORE | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_AY_IGNORE | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        if velocity is not None:
            mask &= ~(mav.POSITION_TARGET_TYPEMASK_VX_IGNORE | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE)
        else:velocity=(0.,0.,0.)
        core.connection.mav.set_position_target_local_ned_send(
            int(now*1000)&0xffffffff, *core.parameters.target, mav.MAV_FRAME_LOCAL_NED,
            mask, *target[:3], *velocity, 0., 0., 0., target[3], 0.)

    def receive(self, msg, now):
        super().receive(msg, now)
        self.automatic.receive(msg, now)
        if self.rc_ground is not None:self.rc_ground.receive(msg, now)
        if self.thrust_model is not None:
            if msg.get_type() == 'TIMESYNC' and msg.tc1 == 0:
                self.thrust_model.sync(msg.ts1, now)
            elif msg.get_type() == 'HIGHRES_IMU' and self.in_offboard and not self.terminal:
                self.thrust_model.sample(msg, now)
        if msg.get_type() == 'MANUAL_CONTROL':
            # PX4 transmits calibrated pitch/roll/yaw in [-1000,1000] and throttle in [0,1000].
            self.pilot_axes = None
            if all(-1000 <= value <= 1000 for value in (msg.x,msg.y,msg.r)) and 0 <= msg.z <= 1000:
                self.pilot_axes = (now, [msg.x/1000., -msg.y/1000., msg.z/500.-1., -msg.r/1000.])
        if msg.get_type() == 'RC_CHANNELS':
            for channel in range(1,19):
                value=getattr(msg,f'chan{channel}_raw',65535)
                if 800<=value<=2200:self.rc_channels[channel]=(value,now)
        elif msg.get_type() == 'ATTITUDE':
            rp = (getattr(msg,'roll',math.nan),getattr(msg,'pitch',math.nan))
            self.roll_pitch = rp if all(math.isfinite(v) for v in rp) else None
        elif msg.get_type() == 'ATTITUDE_TARGET':
            thrust, stamp = getattr(msg,'thrust',math.nan),getattr(msg,'time_boot_ms',None)
            if math.isfinite(thrust) and isinstance(stamp,int) and stamp > 0:
                self.attitude_target = (now,stamp,thrust)
        if msg.get_type() == 'PARAM_VALUE':
            name = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
            name = name.rstrip('\0')
            if name in QUERY_PARAMS+self.throttle_parameters() and msg.param_type in (6, 9):
                value = decode(msg.param_value) if msg.param_type == 6 else msg.param_value
                if math.isfinite(value):
                    self.cache[name] = (value, now)

    def throttle_parameters(self):
        channel=self.cache.get('RC_MAP_THROTTLE',(0,None))[0]
        return tuple(f'RC{int(channel)}_{suffix}' for suffix in ('MIN','MAX','REV')) if 1<=channel<=18 else ()

    def throttle_low(self,now):
        names=('RC_MAP_THROTTLE',)+self.throttle_parameters()
        if len(names)!=4 or any(n not in self.cache or not 0<=now-self.cache[n][1]<20 for n in names):return False
        channel=int(self.cache[names[0]][0]);raw=self.rc_channels.get(channel)
        if raw is None or not 0<=now-raw[1]<.5:return False
        minimum,maximum,reverse=[self.cache[n][0] for n in names[1:]]
        if not 800<=minimum<maximum<=2200 or reverse not in (-1.,1.):return False
        fraction=(raw[0]-minimum)/(maximum-minimum)
        return (fraction if reverse>0 else 1-fraction)<=.1

    def accept(self, p, now, epoch_ns):
        self.observed_now = now
        if not self.enabled:
            return
        keys = {'session', 'sequence', 'stamp_ns', 'valid', 'rates_flu', 'acceleration',
                'phase', 'solver_ms', 'position', 'goal'}
        if not isinstance(p, dict) or not keys <= set(p) or set(p)-keys-{'mission','intent','tracking_backend','px4_reference','measured_yaw'}:
            raise ValueError('invalid SUPER schema')
        if p.get('intent','none') not in ('none','land'):
            raise ValueError('invalid SUPER intent')
        if 'mission' in p and (not isinstance(p['mission'],dict) or p['mission'].get('kind') not in ('none','forward','patrol','rth','takeoff','epic')):
            raise ValueError('invalid SUPER mission')
        if (not isinstance(p['session'], str) or not 16 <= len(p['session']) <= 96
                or type(p['sequence']) is not int or p['sequence'] < 0
                or type(p['stamp_ns']) is not int or not 0 <= (epoch_ns-p['stamp_ns'])/1e9 < .15
                or type(p['valid']) is not bool or not isinstance(p['phase'], str)):
            raise ValueError('invalid SUPER identity/time')
        for name in ('rates_flu', 'position', 'goal'):
            if not isinstance(p[name], list) or len(p[name]) != 3:
                raise ValueError('invalid SUPER vector')
            if not all(type(v) in (float, int) and math.isfinite(v) for v in p[name]):
                raise ValueError('nonfinite SUPER vector')
        if any(type(p[n]) not in (float, int) or not math.isfinite(p[n]) for n in ('acceleration','solver_ms')):
            raise ValueError('invalid SUPER scalar')
        backend=p.get('tracking_backend','ommpc')
        if backend not in ('px4','ommpc'):
            raise ValueError('invalid tracking backend')
        if backend=='px4':
            ref=p.get('px4_reference')
            if (not isinstance(ref,list) or len(ref)!=7 or
                not all(type(v) in (int,float) and math.isfinite(v) for v in [*ref,p.get('measured_yaw')])):
                raise ValueError('invalid PX4 reference')
        elif 'px4_reference' in p or 'measured_yaw' in p:
            raise ValueError('inconsistent tracking backend')
        if self.session==p['session'] and self.tracking_backend!=backend:
            self.abort('SUPER_BACKEND_CHANGED')
            return
        if self.session is not None and (p['session'] != self.session or p['sequence'] <= self.sequence):
            if self.prestream_since is not None:
                self.abort('SUPER_SESSION_CHANGED')
                return
        self.tracking_backend=backend
        self.session, self.sequence = p['session'], p['sequence']
        self.command, self.last_received = p, now

    def audit(self, now):
        missing = [n for n in PARAMS if n not in self.cache or not 0 <= now-self.cache[n][1] < 20]
        if missing:
            return 'SUPER_PARAMETER_READBACK: '+','.join(missing)
        v = {n:self.cache[n][0] for n in PARAMS}
        if not .1 <= v['MPC_THR_HOVER'] <= .8:
            return 'SUPER_BAD_HOVER_THRUST'
        if not 0 <= v['THR_MDL_FAC'] <= 1 or v['RC_MAP_OFFB_SW'] != 8 or v['ZAV_EV_GUARD'] != 1:
            return 'SUPER_PX4_PROFILE_MISMATCH'
        if v['COM_OBL_RC_ACT'] != 0 or not .1 <= v['COM_OF_LOSS_T'] <= 2.:
            return 'SUPER_OFFBOARD_LOSS_PROFILE_MISMATCH'
        if any(math.radians(v[n])+1e-6 < lim for n,lim in zip(
                ('MC_ROLLRATE_MAX','MC_PITCHRATE_MAX','MC_YAWRATE_MAX'), self.rate_limit)):
            return 'SUPER_RATE_LIMIT_MISMATCH'
        return None

    def prepare_hover_thrust(self, core, now, health):
        """Only learn with RC8 LOW, while Position is armed, level and stopped."""
        handoff = self.hover_handoff
        if handoff.frozen and not self.in_offboard and now-handoff.frozen_at > 2.:
            handoff.reset()
            self.hover_thrust = None
            return False  # A cancelled/denied entry requires RC8 LOW and a fresh measurement.
        if core.mode == (6,0):
            return (handoff.frozen or self.seen_low and self.rc8 is not None and self.rc8 >= 1700
                    and self.prestream_since is not None and now-self.prestream_since >= 1.2
                    and handoff.freeze(now))
        if self.rc8 is not None and self.rc8 > 1300:
            # Freeze before the FC heartbeat announces Offboard; it can lag the
            # switch. Never re-learn using delayed/echoed Offboard telemetry.
            return handoff.freeze(now)
        if handoff.frozen:
            handoff.reset()
        stationary = (health and self.rc8 is not None and self.rc8 <= 1300
                      and self.velocity is not None and math.hypot(*self.velocity) <= .15
                      and self.roll_pitch is not None)
        self.sample_hover(now,stationary)
        if handoff.fresh(now):
            self.hover_thrust = handoff.value
            return True
        self.hover_thrust = None
        return False

    def sample_hover(self,now,stationary):
        handoff=self.hover_handoff
        target = self.attitude_target
        if not stationary:
            handoff.clear('waiting_for_stationary_level_position')
        elif (target is not None and 0 <= now-target[0] < .2 and self.local_boot is not None
              and -.15 <= (self.local_boot-target[1])/1000 <= .25):
            handoff.sample(now,target[1],target[2],math.cos(self.roll_pitch[0])*math.cos(self.roll_pitch[1]))

    def clear_rc_ground(self):
        self.px4_frame=None
        self.rc_ground=None
        self.hover_handoff.reset()
        self.hover_thrust=self.last_send=self.prestream_since=None
        self.in_offboard=self.hover_ready=self.seen_low=False

    def _tick(self, core, now):
        from pymavlink.dialects.v20 import common as mav
        self.observed_now, self.fc_mode = now, core.mode
        if self.thrust_model is not None and self.thrust_model.coefficient is not None and (
                not core.armed or core.mode != (6,0) or self.terminal):
            self.thrust_model.reset()
        if not self.enabled or self.terminal:
            return
        # Reads only, paced on the same existing UART; never change flight gains
        # or global Position speed limits merely to use a body-rate controller.
        if core.writable() and now-self.last_query >= .25:
            query=QUERY_PARAMS+self.throttle_parameters()
            name = query[self.query_index % len(query)]
            core.connection.mav.param_request_read_send(*core.parameters.target, name.encode(), -1)
            self.query_index += 1
            self.last_query = now
        issue = self.audit(now)
        status = core.snapshot(now)
        flags = (mav.ESTIMATOR_ATTITUDE | mav.ESTIMATOR_VELOCITY_HORIZ | mav.ESTIMATOR_VELOCITY_VERT
                 | mav.ESTIMATOR_POS_HORIZ_REL | mav.ESTIMATOR_POS_VERT_ABS)
        base_health = (core.writable() and status['ready'] and core.handoff.started is None
                  and not core.motion.latched
                  and all(v is not None and 0 <= now-v < .5 for v in (self.local_rx,self.att_rx,self.rc_rx,self.air_rx))
                  and core.fresh(core.est_rx, now, .5) and core.estimator_flags & flags == flags)
        command_ok = (self.command is not None and self.last_received is not None
                      and 0 <= now-self.last_received < .15 and self.command['valid'])
        recent_command = self.command is not None and self.last_received is not None and 0 <= now-self.last_received < .15
        if recent_command and (self.command.get('intent')=='land' or self.landing_requested is not None):
            self.land_mission(core,now,base_health,issue)
            return
        if self.automatic.enabled and self.automatic.stage != 'COMPLETE':
            scale = self.automatic.step(self,core,now,base_health,command_ok,issue)
            if scale is not None:
                self.send_tracking(core,now,scale,allow_zero=True)
            return
        if not self.automatic.enabled:
            seed=self.command.get('mission',{}).get('rc_ground_seed',0.) if command_ok else 0.
            selected=self.rc8 is not None and self.rc8>=1700
            on_ground=getattr(core,'landed',False) and not self.in_air
            if self.rc_ground is not None and not core.armed and on_ground and (
                    not selected or self.rc_ground.stage in ('PX4_HOVER','HANDOFF','COMPLETE')):
                self.clear_rc_ground()
            if (self.rc_ground is None and selected and on_ground and not core.armed and
                    core.mode in ((3,0),(6,0)) and type(seed) in (float,int) and .1<=seed<=.8):
                self.rc_ground=GroundTakeoff(True,seed,self.automatic.height,pilot_arm=True)
            if self.rc_ground is not None and self.rc_ground.stage!='COMPLETE':
                if not selected and core.mode==(3,0) and core.armed:
                    # The pilot armed in Position or took Position back during climb.
                    self.clear_rc_ground()
                else:
                    self.rc_ground.step(self,core,now,base_health,command_ok,issue)
                    if self.terminal or self.rc_ground.stage!='COMPLETE':
                        if (not self.terminal and self.rc_ground.stage in ('PRESTREAM','MODE','PX4_HOVER','HANDOFF')
                                and (self.last_send is None or now-self.last_send>=.02-1e-9)):
                            if self.last_send is not None and now-self.last_send>.15:
                                self.abort('SUPER_OUTPUT_SCHEDULE_GAP')
                                return
                            self.send_position(core,now,self.rc_ground.position_target)
                            self.sent+=1
                            self.last_send=now
                            if self.prestream_since is None:self.prestream_since=now
                        return
        health = base_health and core.armed and self.in_air
        if not health or issue or not command_ok:
            if not self.hover_handoff.frozen:
                self.hover_handoff.clear('flight_or_controller_not_ready')
            self.hover_ready = False
            cause = issue or ('SUPER_FLIGHT_OR_EV_UNHEALTHY' if not health else 'SUPER_CONTROLLER_LOST_OR_INVALID')
            if self.prestream_since is not None:
                self.abort(cause)
            else:
                self.reason = cause
            return
        if core.mode != (6,0):self.px4_frame=None
        if core.mode == (3,0) and self.in_offboard:
            self.in_offboard = self.seen_low = False
            self.prestream_since = self.last_send = None
            self.hover_handoff.reset()
            self.hover_thrust = None
        if self.tracking_backend=="ommpc" and core.mode in ((3,0),(6,0)) and not self.prepare_hover_thrust(core,now,health):
            self.hover_ready = False
            if core.mode == (6,0):
                self.abort('SUPER_HOVER_THRUST_NOT_READY')
            else:
                self.prestream_since = self.last_send = None
                self.reason = 'SUPER_CALIBRATING_HOVER_THRUST'
            return
        if core.mode == (3, 0):
            if self.in_offboard:
                self.in_offboard = self.seen_low = False
                self.prestream_since = self.last_send = None
            self.hover_ready = False
            if self.rc8 <= 1300:
                self.seen_low = True
            if not self.seen_low or math.hypot(*self.velocity) > .15 and self.prestream_since is None:
                self.reason = 'SUPER_WAIT_POSITION_HOVER'
                return
            self.reason = 'SUPER_READY_RC8' if self.prestream_since is not None and now-self.prestream_since >= 1.2 else 'SUPER_PRESTREAM'
        elif core.mode == (6, 0):
            if not self.in_offboard:
                if not self.seen_low or self.rc8 < 1700 or self.prestream_since is None or now-self.prestream_since < 1.2:
                    self.abort('SUPER_OFFBOARD_BEFORE_PRESTREAM')
                    return
                self.in_offboard = True
            self.reason = 'SUPER_'+self.command['phase']
            self.hover_ready = (self.rc8 >= 1700 and (self.command['phase']=='HOVER' or self.command['phase']=='ENDPOINT_HOLD' and self.command.get('mission',{}).get('complete')) and math.hypot(*self.velocity)<.15)
        else:
            self.in_offboard = self.hover_ready = self.seen_low = False
            self.prestream_since = self.last_send = None
            self.hover_handoff.reset()
            self.hover_thrust = None
            self.reason = 'SUPER_PILOT_MODE'
            return
        tracking = self.in_offboard
        self.send_tracking(core,now,tracking=tracking)

    def send_tracking(self,core,now,scale=1.,allow_zero=False,tracking=False):
        if self.tracking_backend=='ommpc':
            return self.send_rates(core,now,scale,allow_zero,tracking)
        if self.last_send is not None:
            if now-self.last_send>.15:
                self.abort('SUPER_OUTPUT_SCHEDULE_GAP')
                return
            if now-self.last_send<.02-1e-9:return
        # Fix the relationship between the LIO world and the FC local frame
        # once per Offboard entry. Re-anchoring each sample would erase tracking error.
        cmd=self.command
        if core.mode!=(6,0) or not core.armed or not self.in_offboard:
            self.px4_frame=None
            target=(*self.position,self.yaw)
            velocity=(0.,0.,0.)
        else:
            if self.px4_frame is None:
                # This vehicle fuses external-vision position and heading. Its
                # horizontal axes are the same ones used by body_odometry().
                # Anchor translation only: instantaneous attitude error is not
                # a rotation of the map.
                self.px4_frame=(tuple(cmd['position']),tuple(self.position))
            origin,ned=self.px4_frame
            x,y,z,yaw,vx,vy,vz=cmd['px4_reference']
            dx,dy,dz=frd((x-origin[0],y-origin[1],z-origin[2]))
            target=(ned[0]+dx,ned[1]+dy,ned[2]+dz,math.remainder(-yaw,2*math.pi))
            velocity=frd((vx,vy,vz))
        self.send_position(core,now,target,velocity)
        self.last_px4_target=(*target,*velocity)
        self.last_send=now
        self.sent+=1
        if self.prestream_since is None:self.prestream_since=now

    def send_rates(self,core,now,scale=1.,allow_zero=False,tracking=False):
        from pymavlink.dialects.v20 import common as mav
        fast = tracking and self.fresh_tracking_output
        if fast and self.command is not self.last_sent_command:
            # Carry the 100 Hz deadline across 1 ms poll jitter. Reset it on
            # gaps so the UART never bursts to catch up old controller samples.
            due = self.next_rate_slot
            if due is None and self.last_send is not None:
                due = self.last_send + .01
            if due is not None and now + 1e-9 < due:
                return
        elif self.last_send is not None and now-self.last_send < (.04 if fast else .02):
            return
        if self.last_send is not None and now-self.last_send > .15:
            self.abort('SUPER_OUTPUT_SCHEDULE_GAP')
            return
        if any(abs(v)>lim+1e-3 for v,lim in zip(self.command['rates_flu'], self.rate_limit)):
            self.abort('SUPER_RATE_OUT_OF_BOUNDS')
            return
        acceleration = self.command['acceleration']
        # Seed each airborne entry from the measured Position hover handoff.
        thrust = self.hover_thrust*acceleration/9.81*scale
        model = self.thrust_model
        adaptive = (model is not None and self.in_offboard and core.armed and core.mode == (6,0)
                    and self.in_air and not allow_zero)
        learn = adaptive and self.command['phase'] not in ('TAKEOFF_CLIMB', 'RTH_LANDING')
        if adaptive:
            if model.coefficient is None:
                model.coefficient = 9.81/self.hover_thrust
            thrust = min(.9, max(.04, acceleration/model.coefficient))
        if not 0 < acceleration <= 30. or not (0 <= thrust <= 1. if allow_zero else 0 < thrust <= 1.):
            self.abort('SUPER_THRUST_OUT_OF_BOUNDS')
            return
        r,p,y = (0.,0.,0.) if allow_zero and scale==0. else self.command['rates_flu']
        core.connection.mav.set_attitude_target_send(
            int(now*1000)&0xffffffff,*core.parameters.target,
            mav.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,[1.,0.,0.,0.],r,-p,-y,thrust)
        if learn:
            model.commands.append((now, thrust))
        elif model is not None:
            model.commands.clear()
        if fast:
            slot = self.next_rate_slot
            self.next_rate_slot = slot+.01 if slot is not None and 0 <= now-slot < .002 else now+.01
        else:
            self.next_rate_slot = None
        self.last_sent_command = self.command
        self.normalized_thrust = thrust
        self.last_send, self.sent = now, self.sent+1
        if self.prestream_since is None:
            self.prestream_since = now

    def land_mission(self,core,now,health,issue):
        from pymavlink.dialects.v20 import common as mav
        self.hover_ready=False
        if self.landing_requested is not None and core.mode==(4,6):
            on_ground=core.landed and core.fresh(getattr(core,'landed_rx',None),now,.5)
            self.reason='SUPER_RTH_LANDED' if not core.armed and on_ground else 'SUPER_RTH_LANDING'
            return  # PX4 owns descent and its configured automatic disarm.
        mission=self.command.get('mission',{})
        if (not health or issue or self.rc8 is None or self.rc8<1700
                or core.mode!=(6,0) or not core.armed):
            self.abort('SUPER_RTH_LAND_CANCELLED_OR_UNHEALTHY');return
        if (mission.get('kind')!='rth' or not mission.get('home_valid')
                or self.command['phase']!='RTH_LANDING'
                or math.dist(self.command['position'],self.command['goal'])>.25
                or math.hypot(*self.velocity)>.15):
            self.abort('SUPER_INVALID_LAND_INTENT');return
        disarm=self.cache.get('COM_DISARM_LAND')
        if not disarm or now-disarm[1]>=20 or not 0<disarm[0]<=30:
            self.abort('SUPER_LAND_DISARM_NOT_CONFIGURED');return
        if self.landing_requested is None:self.landing_requested=now
        if now-self.landing_requested>5.:
            self.abort('SUPER_LAND_MODE_NOT_CONFIRMED');return
        if self.landing_last_request is None or now-self.landing_last_request>=1.:
            if self.landing_requests<3:
                core.connection.mav.command_long_send(*core.parameters.target,mav.MAV_CMD_NAV_LAND,0,
                    0,0,0,math.nan,math.nan,math.nan,math.nan)
                self.landing_requests+=1;self.landing_last_request=now
        self.reason='SUPER_RTH_LAND_REQUEST'
        if self.command['valid']:self.send_tracking(core,now)

    def snapshot(self, now=None):
        result = super().snapshot(now)
        result.update(source='super',pattern='super',controller=self.command,tracking_backend=self.tracking_backend,
                      px4_target_ned=self.last_px4_target,
                      controller_age_s=now-self.last_received if now is not None and self.last_received is not None else None,
                      px4_parameters={n:v[0] for n,v in self.cache.items()},
                      parameter_issue=self.audit(now) if now is not None else None,
                      thrust=self.normalized_thrust, hover_thrust=self.hover_thrust,
                      thrust_to_acceleration=self.thrust_model.coefficient if self.thrust_model else None,
                      thrust_fit_updates=self.thrust_model.updates if self.thrust_model else 0,
                      thrust_fit_age_s=(now-self.thrust_model.updated_at if now is not None and
                                        self.thrust_model and self.thrust_model.updated_at is not None else None),
                      hover_handoff=self.hover_handoff.snapshot(now) if now is not None else None,
                      parameter_hover_thrust=self.cache.get('MPC_THR_HOVER',(None,None))[0],
                      auto_land_disarm_s=self.cache.get('COM_DISARM_LAND',(0.,None))[0],
                      rc_ground_stage=self.rc_ground.stage if self.rc_ground else "IDLE",
                      automatic_takeoff=self.automatic.snapshot(),
                      fault_recovery=self.recovery,
                      rate_limits_rad_s=list(self.rate_limit))
        return result
