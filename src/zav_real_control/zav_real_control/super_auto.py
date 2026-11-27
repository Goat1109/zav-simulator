"""Explicit one-shot ground takeoff, called only by the existing UART owner.

No connection creation and no force-arm. Disabled for the normal SUPER bridge.
The caller supplies verified hover scale; an uncalibrated FC initial value is
never silently substituted. Tests use a fake MAVLink sender only.
"""
import math


class GroundTakeoff:
    def __init__(self, enabled=False, hover_thrust=0., height=.4, pilot_arm=False):
        if enabled and (not math.isfinite(hover_thrust) or not .1 <= hover_thrust <= .8):
            raise ValueError('Automatic takeoff requires a verified hover_thrust in [0.1,0.8]')
        if enabled and (not math.isfinite(height) or not .3<=height<=2.):
            raise ValueError('Automatic takeoff height must be in [0.3,2]m')
        self.pilot_arm = pilot_arm
        self.position_target = None
        self.enabled, self.hover_thrust, self.height = enabled, hover_thrust, height
        self.stage = 'WAITING' if enabled else 'DISABLED'
        self.created = self.since = self.last_request = None
        self.requests = 0
        self.denied = None
        self.ground_z = None

    def receive(self, msg, now):
        from pymavlink.dialects.v20 import common as mav
        if msg.get_type() != 'COMMAND_ACK':
            return
        expected = {'MODE': mav.MAV_CMD_DO_SET_MODE, 'ARM': mav.MAV_CMD_COMPONENT_ARM_DISARM}.get(self.stage)
        if expected == msg.command and msg.result not in (mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS):
            self.denied = f'FC_COMMAND_REJECTED_{msg.command}_{msg.result}'

    def request(self, core, command, values, now):
        if self.last_request is not None and now-self.last_request < 1.:
            return
        if self.requests >= 3:
            self.denied = 'FC_COMMAND_CONFIRMATION_TIMEOUT'
            return
        core.connection.mav.command_long_send(*core.parameters.target, command, 0, *values)
        self.last_request = now
        self.requests += 1

    def step(self, gate, core, now, healthy, command_ok, issue):
        from pymavlink.dialects.v20 import common as mav
        if self.created is None:
            self.created = now
        def fail(reason):
            self.stage = 'FAILED'
            gate.abort('SUPER_AUTO_'+reason)
            return None
        if self.denied:
            return fail(self.denied)
        if now-self.created > 45. and self.stage not in ('CLIMB', 'COMPLETE') and not self.pilot_arm:
            return fail('PREPARATION_TIMEOUT')
        if not healthy or not command_ok or issue:
            gate.reason = issue or 'SUPER_AUTO_WAIT_FEEDBACK'
            if self.stage != 'WAITING':
                return fail('FEEDBACK_LOST')
            return None
        if gate.rc8 is None or gate.rc8 < 1700:
            if self.stage != 'WAITING':
                return fail('RC8_PILOT_TAKEOVER')
            gate.reason = 'SUPER_AUTO_WAIT_RC8_HIGH'
            return None
        if not self.pilot_arm and self.stage not in ('CLIMB','COMPLETE') and not gate.throttle_low(now):
            if self.stage!='WAITING':return fail('THROTTLE_NOT_LOW')
            gate.reason='SUPER_AUTO_WAIT_THROTTLE_LOW'
            return None
        if self.pilot_arm and self.stage in ('PRESTREAM','MODE','PILOT_ARM'):
            if core.mode not in ((3,0),(6,0)):
                return fail('PILOT_OR_MODE_EXIT')
            if self.stage in ('PRESTREAM','MODE') and not core.armed:
                return fail('DISARMED_BEFORE_OFFBOARD')
            if not core.landed or gate.velocity is None or math.hypot(*gate.velocity)>.1 or gate.roll_pitch is None or max(abs(v) for v in gate.roll_pitch)>math.radians(10):
                return fail('GROUND_STATE_CHANGED')
            if not core.armed and gate.command.get('mission',{}).get('rc_ground_seed') != self.hover_thrust:
                return fail('GROUND_REFERENCE_LOST')
        if self.stage == 'WAITING':
            parameter=gate.cache.get('MPC_THR_HOVER')
            if not self.pilot_arm and (parameter is None or abs(parameter[0]-self.hover_thrust)>.05):
                gate.reason='SUPER_AUTO_HOVER_PARAMETER_MISMATCH'
                return None
            allowed_modes = ((3,0),) if self.pilot_arm else ((2,0),(3,0),(6,0))
            if core.armed or not core.landed or core.mode not in allowed_modes:
                gate.reason = 'SUPER_AUTO_NEED_DISARMED_GROUND'
                return None
            if (gate.velocity is None or math.hypot(*gate.velocity) > .1
                    or gate.roll_pitch is None or max(abs(v) for v in gate.roll_pitch) > math.radians(10)):
                gate.reason = 'SUPER_AUTO_NEED_LEVEL_STOP'
                return None
            self.ground_z = gate.position[2]
            self.stage, self.since = ('PILOT_ARM' if self.pilot_arm else 'PRESTREAM'), now
            if not self.pilot_arm:
                gate.hover_thrust = self.hover_thrust
                h = gate.hover_handoff
                h.value, h.ready, h.frozen, h.frozen_at = self.hover_thrust, True, True, now
                h.reason = 'explicit_verified_ground_takeoff_thrust'
            gate.seen_low = True  # The ground-start state machine owns this mode transition.
        if self.stage=='PILOT_ARM':
            # PX4 accepts RC stick arming in Position, but rejects it in Offboard.
            # Do not advertise Offboard setpoints until native arming is confirmed.
            if core.mode!=(3,0):return fail('PILOT_OR_MODE_EXIT')
            if not core.armed:
                gate.reason='SUPER_AUTO_PILOT_ARM'
                return None
            self.position_target=(*gate.position,gate.yaw)
            self.stage,self.since='PRESTREAM',now
        if self.stage == 'PRESTREAM' and now-self.since >= 1.2:
            self.stage, self.since, self.last_request, self.requests = 'MODE', now, None, 0
        if self.stage == 'MODE':
            if core.mode == (6,0):
                self.stage = 'PX4_HOVER' if self.pilot_arm else 'ARM'
                if self.pilot_arm:
                    x,y,z,yaw=self.position_target
                    self.position_target=(x,y,z-self.height,yaw)
                    gate.hover_handoff.reset()
                self.since, self.last_request, self.requests = now, None, 0
            else:
                self.request(core, mav.MAV_CMD_DO_SET_MODE, (mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,6,0,0,0,0,0), now)
        if self.stage in ('PX4_HOVER','HANDOFF'):
            if core.mode!=(6,0) or not core.armed:
                return fail('PILOT_OR_MODE_EXIT')
            if not gate.in_air and now-self.since>12.:
                return fail('NO_LIFTOFF')
            settled=(gate.in_air and math.dist(gate.position,self.position_target[:3])<.12
                     and math.hypot(*gate.velocity)<.15 and gate.roll_pitch is not None)
            px4_path=gate.command.get('tracking_backend')=='px4'
            if not px4_path:gate.sample_hover(now,settled)
            if (settled if px4_path else gate.hover_handoff.fresh(now)):
                # The supervisor only acknowledges HOVER after seeing HANDOFF.
                if self.stage=='HANDOFF' and gate.command['phase']=='HOVER' and gate.command.get('mission',{}).get('px4_hover_acquired') is True:
                    if not px4_path:
                        gate.hover_handoff.freeze(now)
                        gate.hover_thrust=gate.hover_handoff.value
                    gate.in_offboard=True
                    self.stage='COMPLETE'
                else:self.stage='HANDOFF'
            else:self.stage='PX4_HOVER'
            gate.reason='SUPER_'+self.stage
            return None
        if self.stage=='ARM':
            if core.mode != (6,0):
                return fail('MODE_EXIT')
            if core.armed:
                self.stage, self.since = 'CLIMB', now
                gate.in_offboard = True
            else:
                # Normal arming only: param2=0, never the force-arm magic value.
                self.request(core, mav.MAV_CMD_COMPONENT_ARM_DISARM, (1,0,0,0,0,0,0), now)
        if self.stage == 'CLIMB':
            if not core.armed or core.mode != (6,0):
                return fail('PILOT_OR_MODE_EXIT')
            if gate.in_air and gate.command.get('mission',{}).get('kind')=='rth':
                self.stage='COMPLETE';return 1.
            height = self.ground_z-gate.position[2]
            if now-self.since > 8. and not gate.in_air and height < .08:
                # Never disarm in the air. Both independent ground/height checks
                # and a stopped local estimate are required for this abort.
                if core.landed and math.hypot(*gate.velocity) < .1:
                    core.connection.mav.command_long_send(*core.parameters.target,
                        mav.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
                return fail('NO_LIFTOFF')
            if (gate.command['phase']=='HOVER' and gate.in_air
                    and abs(height-self.height)<.15 and math.hypot(*gate.velocity)<.15):
                self.stage = 'COMPLETE'
                gate.reason = 'SUPER_HOVER'
                return 1.
            gate.reason = 'SUPER_AUTO_CLIMB'
            return min(1., max(0., (now-self.since)/1.))
        gate.reason = 'SUPER_AUTO_'+self.stage
        return 0.  # Zero thrust in the disarmed prestream/mode/arm phases.

    def snapshot(self):
        return {'enabled':self.enabled,'stage':self.stage,'height_m':self.height,
                'verified_hover_thrust':self.hover_thrust if self.enabled else None}
