"""Pure, testable Offboard reference generator and fail-closed MAVLink sender.

No connection creation, arm, takeoff, mode change or parameter writes.
Coordinates are FC local NED, yaw radians clockwise, referenced to entry heading.
"""
from dataclasses import dataclass, asdict
import math
from collections import deque

PATTERNS = ('front', 'right', 'back', 'left', 'square', 'circle')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Options:
    distance_m: float = 5.0
    radius_m: float = 2.5
    speed_m_s: float = 3.0
    acceleration_m_s2: float = 3.0
    yaw_rate_deg_s: float = 60.0
    entry_hold_s: float = 3.0
    dwell_s: float = 2.0
    acceptance_m: float = 0.2
    stopped_speed_m_s: float = 0.15
    yaw_acceptance_deg: float = 5.0
    settle_s: float = 1.0
    max_tracking_error_m: float = 1.0
    segment_timeout_s: float = 90.0
    max_reference_lead_m: float = 0.5
    max_reference_yaw_lead_deg: float = 20.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        limits = dict(distance_m=5., radius_m=2.5, speed_m_s=3., acceleration_m_s2=5., yaw_rate_deg_s=60., max_tracking_error_m=2.)
        for key, maximum in limits.items():
            if getattr(self, key) > maximum:
                raise ValueError(f'{key} exceeds test limit {maximum}')
        if not self.max_reference_lead_m < self.max_tracking_error_m:
            raise ValueError('reference lead must be smaller than tracking error limit')
        if self.max_reference_yaw_lead_deg > 45:
            raise ValueError('yaw reference lead exceeds 45 degrees')
        if self.entry_hold_s < 2 or self.dwell_s < 1 or self.segment_timeout_s < 10:
            raise ValueError('hold/timeout values too short')


def smooth(u):
    u = min(1., max(0., u))
    return u*u*u*(10 + u*(-15 + 6*u))


def segments(pattern, options):
    if pattern not in PATTERNS:
        raise ValueError('unknown test')
    d = options.distance_m
    if pattern in ('front', 'right', 'back', 'left'):
        x, y = {'front': (d, 0.), 'right': (0., d), 'back': (-d, 0.), 'left': (0., -d)}[pattern]
        return [('move', x, y, 0.)]
    if pattern == 'circle':
        return [('circle', 0., 0., 0.)]
    out = []
    for i, (x, y) in enumerate(((d, 0.), (d, d), (0., d), (0., 0.))):
        out.extend((('move', x, y, i*math.pi/2), ('turn', x, y, (i+1)*math.pi/2)))
    return out


class Mission:
    def __init__(self, pattern, options, origin, now):
        self.options, self.origin = options, tuple(origin)  # x,y,z,yaw when a command is accepted in Offboard hover
        self.path = segments(pattern, options)
        self.index = 0
        self.phase = 'ENTRY_HOLD'
        self.phase_since = now
        self.started = now
        self.segment_since = now
        self.stable_since = None
        self.start = (0., 0., 0.)
        self.target = self.origin
        self.error = None
        self.reference_time = 0.
        self.last_step = now
        self.reference_paused = False

    def world(self, x, y, yaw):
        ox, oy, oz, heading = self.origin
        return (ox + math.cos(heading)*x - math.sin(heading)*y,
                oy + math.sin(heading)*x + math.cos(heading)*y, oz, wrap(heading+yaw))

    def duration(self, segment):
        kind, x, y, yaw = segment
        o = self.options
        if kind == 'turn':
            return max(1., 1.875*abs(yaw-self.start[2])/math.radians(o.yaw_rate_deg_s))
        if kind == 'circle':
            length = 2*math.pi*o.radius_m
            return max(1.875*length/o.speed_m_s,
                       math.sqrt((5.774*length+(1.875*2*math.pi)**2*o.radius_m)/o.acceleration_m_s2))
        length = math.hypot(x-self.start[0], y-self.start[1])
        return max(1., 1.875*length/o.speed_m_s, math.sqrt(5.774*length/o.acceleration_m_s2))

    def settled(self, now, position, velocity, yaw):
        o = self.options
        okay = (math.dist(position, self.target[:3]) <= o.acceptance_m
                and math.hypot(*velocity) <= o.stopped_speed_m_s
                and abs(wrap(yaw-self.target[3])) <= math.radians(o.yaw_acceptance_deg))
        if not okay:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now-self.stable_since >= o.settle_s

    def reference(self, elapsed):
        kind, x, y, angle = segment = self.path[self.index]
        u = smooth(elapsed/self.duration(segment))
        if kind == 'circle':
            theta = 2*math.pi*u
            return self.world(self.options.radius_m*math.sin(theta),
                              self.options.radius_m*(1-math.cos(theta)), 0.)
        return self.world(self.start[0]+u*(x-self.start[0]), self.start[1]+u*(y-self.start[1]),
                          self.start[2]+u*(angle-self.start[2]))

    def step(self, now, position, velocity, yaw):
        dt = max(0., now-self.last_step)
        self.last_step = now
        if math.dist(position, self.target[:3]) > self.options.max_tracking_error_m:
            self.error = 'tracking_error'
            return None
        if self.phase == 'ENTRY_HOLD':
            stable = self.settled(now, position, velocity, yaw)
            if now-self.phase_since >= self.options.entry_hold_s and stable:
                self.phase, self.segment_since, self.stable_since = 'SEGMENT', now, None
        elif self.phase == 'SEGMENT':
            duration = self.duration(self.path[self.index])
            elapsed = now-self.segment_since
            proposed = min(duration, self.reference_time+dt)
            def within_lead(clock):
                candidate = self.reference(clock)
                return (math.dist(position, candidate[:3]) <= self.options.max_reference_lead_m
                        and abs(wrap(candidate[3]-yaw)) <= math.radians(self.options.max_reference_yaw_lead_deg))
            accepted = proposed
            if not within_lead(proposed):
                # Advance only as far as feedback permits. Never push the target farther
                # away merely because the wall clock advanced while the aircraft lagged.
                lo, hi = self.reference_time, proposed
                if within_lead(lo):
                    for _ in range(24):
                        mid = (lo+hi)/2
                        if within_lead(mid): lo = mid
                        else: hi = mid
                accepted = lo
            self.reference_paused = accepted < proposed-1e-8
            self.reference_time = accepted
            self.target = self.reference(accepted)
            if elapsed > duration+self.options.segment_timeout_s:
                self.error = 'segment_timeout'
                return None
            if self.reference_time >= duration and self.settled(now, position, velocity, yaw):
                self.phase, self.phase_since = 'DWELL', now
        elif self.phase == 'DWELL' and now-self.phase_since >= self.options.dwell_s:
            segment = self.path[self.index]
            self.start = (0., 0., 0.) if segment[0] == 'circle' else segment[1:]
            self.index += 1
            self.stable_since = None
            if self.index == len(self.path):
                self.phase = 'COMPLETE_HOLD'
                self.phase_since = now
            else:
                self.phase, self.segment_since = 'SEGMENT', now
                self.reference_time = 0.
                self.reference_paused = False
        return self.target

    def details(self, now):
        """Read-only reference progress; it never advances the mission."""
        index = min(self.index, len(self.path)-1)
        segment = self.path[index]
        goal = self.world(0., 0., 0.) if segment[0] == 'circle' else self.world(*segment[1:])
        fraction = 0.
        if self.phase == 'COMPLETE_HOLD':
            fraction = 1.
        elif self.phase == 'DWELL':
            fraction = 1.
        elif self.phase == 'SEGMENT':
            fraction = smooth(self.reference_time/self.duration(segment))
        progress = 100. if self.phase == 'COMPLETE_HOLD' else 100*(index+fraction)/len(self.path)
        return dict(reference_paused=self.reference_paused, stage=index+1, stages=len(self.path), segment_kind=segment[0],
                    leg=index//2+1 if len(self.path)==8 else 1,
                    legs=4 if len(self.path)==8 else 1,
                    reference_progress_pct=progress, elapsed_s=max(0.,now-self.started),
                    dwell_remaining_s=max(0.,self.options.dwell_s-(now-self.phase_since)) if self.phase=='DWELL' else None,
                    goal_ned_yaw=list(goal))


class OffboardControl:
    """Bridge-owned hold stream; test clients request paths only after hover is ready.

    No arm/mode commands. Fatal feedback/EV failures still stop setpoints and leave
    the existing handoff / PX4 Offboard-loss response in charge.
    """
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.default_options = Options()
        self.session = self.request = self.request_rx = None
        self.sequence = -1
        self.retired_sessions = deque(maxlen=128)
        self.command_state = 'none'
        self.command_reason = None
        self.last_rejection = None
        self.rc8 = self.rc_rx = None
        self.position = self.velocity = self.local_rx = self.local_boot = None
        self.yaw = self.att_rx = None
        self.in_air = False
        self.air_rx = None
        self.seen_low = False
        self.prestream_since = self.last_send = None
        self.mission = None
        self.terminal = False
        self.reason = 'disabled' if not enabled else 'waiting_for_armed_airborne_healthy_position'
        self.sent = 0
        self.hold_target = self.target = None
        self.observed_now = self.stopped_at = None
        self.fc_mode = None
        self.mode_history = deque(maxlen=6)
        self.in_offboard = False
        self.hover_ready = False
        self.stable_since = None
        self.fault = None

    def receive(self, msg, now):
        self.observed_now = now
        kind = msg.get_type()
        if kind == 'RC_CHANNELS':
            value = getattr(msg, 'chan8_raw', 65535)
            if 800 <= value <= 2200:
                self.rc8, self.rc_rx = value, now
        elif kind == 'LOCAL_POSITION_NED':
            values = tuple(getattr(msg, k, math.nan) for k in ('x', 'y', 'z', 'vx', 'vy', 'vz'))
            if all(math.isfinite(v) for v in values):
                reset = False
                if self.local_rx is not None:
                    dt = now-self.local_rx
                    reset = (getattr(msg, 'time_boot_ms', 0) < self.local_boot
                             or (0 < dt < 1 and math.dist(values[:3], self.position) > 1.+math.hypot(*self.velocity)*dt))
                self.position, self.velocity = values[:3], values[3:]
                self.local_rx, self.local_boot = now, getattr(msg, 'time_boot_ms', 0)
                if reset and self.prestream_since is not None:
                    self.abort('fc_local_position_reset_or_jump')
        elif kind == 'ATTITUDE' and math.isfinite(getattr(msg, 'yaw', math.nan)):
            self.yaw, self.att_rx = msg.yaw, now
        elif kind == 'EXTENDED_SYS_STATE':
            self.in_air, self.air_rx = msg.landed_state == 2, now

    def accept(self, request, now, epoch_ns):
        self.observed_now = now
        if not self.enabled:
            return
        if not isinstance(request, dict) or set(request) != {'session', 'sequence', 'stamp_ns', 'pattern', 'options'}:
            raise ValueError('invalid Offboard request schema')
        session, seq, stamp = request['session'], request['sequence'], request['stamp_ns']
        if not isinstance(session, str) or not 16 <= len(session) <= 64:
            raise ValueError('invalid session')
        if type(seq) is not int or seq < 0 or type(stamp) is not int or not 0 <= (epoch_ns-stamp)/1e9 < .3:
            raise ValueError('stale/invalid request')
        options = Options(**request['options'])
        if request['pattern'] not in PATTERNS:
            raise ValueError('invalid pattern')
        if session in self.retired_sessions:
            return
        if session == self.session:
            if self.command_state != 'running':
                return  # A completed/cancelled command never restarts from its heartbeat.
            if seq <= self.sequence or self.request != (request['pattern'], options):
                self.abort('request_sequence_or_options_changed')
                return
            self.sequence, self.request_rx = seq, now
            return
        reject = None
        if self.terminal:
            reject = 'bridge_fault_restart_required'
        elif self.command_state == 'running':
            reject = 'another_test_running'
        elif (not self.hover_ready or self.fc_mode != (6, 0) or not self.in_offboard
              or self.rc8 is None or self.rc8 < 1700
              or self.last_send is None or not 0 <= now-self.last_send < .25):
            reject = 'command_requires_offboard_hover'
        if reject:
            self.retired_sessions.append(session)
            self.last_rejection = dict(session=session, pattern=request['pattern'], reason=reject, time_s=now)
            return
        if self.session:
            self.retired_sessions.append(self.session)
        self.session, self.sequence, self.request_rx = session, seq, now
        self.request = (request['pattern'], options)
        self.mission = Mission(*self.request, (*self.position, self.yaw), now)
        self.command_state, self.command_reason = 'running', None
        self.last_rejection = None
        self.stopped_at = None
        self.hover_ready = False
        self.reason = self.mission.phase

    def cancel(self, reason, now):
        if self.command_state == 'running':
            self.command_state, self.command_reason = 'cancelled', reason
            self.stopped_at = now
        # Freeze a new hold at measured pose rather than continue an abandoned path.
        self.hold_target = (*self.position, self.yaw)
        self.target = self.hold_target
        self.hover_ready = False
        self.stable_since = None

    def abort(self, reason):
        if not self.terminal:
            self.reason, self.terminal = reason, True
            self.stopped_at = self.observed_now
            self.hover_ready = False
            self.fault = dict(reason=reason, time_s=self.observed_now, rc_pwm=self.rc8,
                              fc_mode=self.fc_mode, session=self.session,
                              target_ned_yaw=list(self.target) if self.target else None,
                              position_ned=list(self.position) if self.position else None,
                              tracking_error_m=math.dist(self.position, self.target[:3])
                                  if self.position and self.target else None)
            if self.command_state == 'running':
                self.command_state, self.command_reason = 'failed', reason

    def tick(self, core, now):
        self.observed_now = now
        from pymavlink.dialects.v20 import common as mav
        if not self.enabled or self.terminal:
            return
        if self.fc_mode != core.mode:
            self.fc_mode = core.mode
            self.mode_history.append(dict(time_s=now, mode=list(core.mode) if core.mode else None, rc_pwm=self.rc8))
        status = core.snapshot(now)
        health = (core.writable() and status['ready'] and core.handoff.started is None
                  and not core.motion.latched and core.armed and self.in_air
                  and all(v is not None and 0 <= now-v < .5 for v in (self.local_rx, self.att_rx, self.rc_rx,self.air_rx)))
        flags = (mav.ESTIMATOR_ATTITUDE | mav.ESTIMATOR_VELOCITY_HORIZ | mav.ESTIMATOR_VELOCITY_VERT
                 | mav.ESTIMATOR_POS_HORIZ_REL | mav.ESTIMATOR_POS_VERT_ABS)
        health = health and core.fresh(core.est_rx, now, .5) and core.estimator_flags & flags == flags
        if not health:
            self.hover_ready = False
            if self.prestream_since is not None:
                self.abort('flight_or_EV_feedback_unhealthy')
            else:
                self.reason = 'waiting_for_armed_airborne_healthy_position'
            return
        if core.mode == (3, 0):
            if self.in_offboard:
                self.cancel('pilot_mode_or_RC8_exit', now)
                self.in_offboard, self.seen_low = False, False
                self.prestream_since = self.last_send = None
            self.hover_ready = False
            if self.rc8 <= 1300:
                self.seen_low = True
            if not self.seen_low:
                self.reason = 'waiting_for_position_mode_and_RC8_low'
                return
            if math.hypot(*self.velocity) > self.default_options.stopped_speed_m_s and self.prestream_since is None:
                self.reason = 'waiting_for_stationary_position_hover'
                return
            # PX4 Position owns the aircraft. Follow its measured pose so a later
            # mode transition cannot pull it back to a stale prestream anchor.
            self.hold_target = (*self.position, self.yaw)
            self.target = self.hold_target
            self.reason = ('READY_FOR_RC8_HIGH' if self.prestream_since is not None
                           and now-self.prestream_since >= 1.2 else 'PRESTREAM_HOLD_RC8_LOW')
            if self.rc8 >= 1700:
                self.reason = 'WAITING_FOR_FC_OFFBOARD'
        elif core.mode == (6, 0):
            if not self.in_offboard:
                if (not self.seen_low or self.prestream_since is None or now-self.prestream_since < 1.2
                        or self.rc8 < 1700):
                    self.reason = 'offboard_entered_before_prestream'
                    return
                self.in_offboard = True
                self.hold_target = (*self.position, self.yaw)
                self.target = self.hold_target
                self.stable_since = None
            # Intermediate PWM values are normal while a physical switch changes.
            # Actual FC mode is authoritative. LOW cancels a path while awaiting
            # the Position heartbeat, but never drops the hold stream on its own.
            if self.command_state == 'running' and self.rc8 <= 1300:
                self.cancel('RC8_low', now)
            if self.command_state == 'running' and (self.request_rx is None or now-self.request_rx >= .3):
                self.cancel('test_node_lost', now)
            if self.command_state == 'running':
                self.target = self.mission.step(now, self.position, self.velocity, self.yaw)
                if self.target is None:
                    self.target = self.mission.target
                    self.abort(self.mission.error)
                    return
                self.reason = self.mission.phase
                if self.mission.phase == 'COMPLETE_HOLD':
                    self.command_state = 'complete'
                    self.stopped_at = now
                    self.hold_target = self.target
                    self.stable_since = None
            else:
                self.target = self.hold_target
                self.reason = 'OFFBOARD_HOVER_READY' if self._hover_settled(now) else 'OFFBOARD_HOVER_SETTLING'
        else:
            if self.in_offboard:
                self.cancel('pilot_mode_or_RC8_exit', now)
            self.in_offboard, self.hover_ready, self.seen_low = False, False, False
            self.prestream_since = self.last_send = None
            self.reason = 'pilot_mode_or_RC8_exit'
            return
        if self.last_send is not None and now-self.last_send < .05:
            return
        if self.last_send is not None and now-self.last_send > .25:
            self.abort('setpoint_schedule_gap')
            return
        if self.prestream_since is None:
            self.prestream_since = now
        mask = (mav.POSITION_TARGET_TYPEMASK_VX_IGNORE | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        core.connection.mav.set_position_target_local_ned_send(
            int(now*1000) & 0xffffffff, *core.parameters.target, mav.MAV_FRAME_LOCAL_NED,
            mask, *self.target[:3], 0., 0., 0., 0., 0., 0., self.target[3], 0.)
        self.last_send, self.sent = now, self.sent+1

    def _hover_settled(self, now):
        o = self.request[1] if self.request else self.default_options
        stable = (math.dist(self.position, self.hold_target[:3]) <= o.acceptance_m
                  and math.hypot(*self.velocity) <= o.stopped_speed_m_s
                  and abs(wrap(self.yaw-self.hold_target[3])) <= math.radians(o.yaw_acceptance_deg)
                  and self.rc8 >= 1700)
        if not stable:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        self.hover_ready = stable and now-self.stable_since >= o.settle_s
        return self.hover_ready

    def snapshot(self, now=None):
        result = dict(enabled=self.enabled, phase=self.reason, stopped=self.terminal, sent=self.sent,
                    hover_ready=self.hover_ready, command_state=self.command_state, command_reason=self.command_reason,
                    last_rejection=self.last_rejection, fault=self.fault, mode_history=list(self.mode_history),
                    session=self.session, pattern=self.request[0] if self.request else None,
                    rc_channel=8, rc_pwm=self.rc8, fc_mode=self.fc_mode,
                    target_ned_yaw=list(self.target) if self.target else None,
                    position_ned=list(self.position) if self.position is not None else None,
                    speed_m_s=math.hypot(*self.velocity) if self.velocity is not None else None,
                    horizontal_speed_m_s=math.hypot(*self.velocity[:2]) if self.velocity is not None else None,
                    yaw_deg=math.degrees(self.yaw) if self.yaw is not None else None,
                    options=asdict(self.request[1]) if self.request else None)
        if now is not None:
            result.update(request_age_s=now-self.request_rx if self.request_rx is not None else None,
                          local_age_s=now-self.local_rx if self.local_rx is not None else None,
                          setpoint_age_s=now-self.last_send if self.last_send is not None else None)
            if self.mission:
                result.update(self.mission.details(self.stopped_at if self.stopped_at is not None else now))
        if self.mission and self.position is not None:
            ox,oy,oz,heading=self.mission.origin
            def relative(p):
                x,y,z=p[0]-ox,p[1]-oy,p[2]-oz
                return [math.cos(heading)*x+math.sin(heading)*y,
                        -math.sin(heading)*x+math.cos(heading)*y,-z]
            result['position_relative_front_right_up_m']=relative(self.position)
            result['tracking_error_m']=math.dist(self.position,self.target[:3]) if self.target else None
            if result.get('goal_ned_yaw') is not None:
                goal=result['goal_ned_yaw']
                result.update(goal_relative_front_right_up_m=relative(goal),
                              goal_distance_m=math.dist(self.position,goal[:3]),
                              goal_yaw_error_deg=math.degrees(wrap(goal[3]-self.yaw)) if self.yaw is not None else None)
        return result
