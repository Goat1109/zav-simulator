"""Synthetic telemetry only; never opens a serial device or launches real flight."""
import math
from types import SimpleNamespace
from copy import deepcopy
import pytest
from pymavlink.dialects.v20 import common as mav
from zav_real_control.super_gate import SuperRateGate,PARAMS
from zav_real_control.mavlink_parameters import encode
from zav_real_control.super_adapter import body_message
from zav_real_control.geometry import rotate,mul,conj


def msg(name,**kw):return SimpleNamespace(get_type=lambda:name,**kw)


class Core:
    def __init__(self):
        self.calls=[]
        self.connection=SimpleNamespace(mav=SimpleNamespace(
            param_request_read_send=lambda *a:self.calls.append(('param_request_read_send',a)),
            set_position_target_local_ned_send=lambda *a:self.calls.append(('set_position_target_local_ned_send',a)),
            command_long_send=lambda *a:self.calls.append(('command_long_send',a)),
            set_attitude_target_send=lambda *a:self.calls.append(('set_attitude_target_send',a))))
        self.parameters=SimpleNamespace(target=(1,1))
        self.handoff=SimpleNamespace(started=None);self.motion=SimpleNamespace(latched=False)
        self.ready=True;self.live=True;self.armed=True;self.mode=(3,0);self.est_rx=0.
        self.estimator_flags=(mav.ESTIMATOR_ATTITUDE|mav.ESTIMATOR_VELOCITY_HORIZ|mav.ESTIMATOR_VELOCITY_VERT|mav.ESTIMATOR_POS_HORIZ_REL|mav.ESTIMATOR_POS_VERT_ABS)
    def writable(self):return self.live
    def snapshot(self,now):return {'ready':self.ready}
    def fresh(self,rx,now,age):return rx is not None and 0<=now-rx<age


def payload(t):return dict(session='super-session-offline-test',sequence=int(t*10000)+1,stamp_ns=int((1800000000+t)*1e9),valid=True,rates_flu=[.1,.2,.3],acceleration=9.81,phase='HOVER',solver_ms=1.,position=[0.,0.,2.],goal=[7.,0.,2.])


def params(g,t):
    values=dict(MPC_THR_HOVER=.18,THR_MDL_FAC=0.,RC_MAP_OFFB_SW=8,COM_OF_LOSS_T=1.,COM_OBL_RC_ACT=0,ZAV_EV_GUARD=1,MC_ROLLRATE_MAX=220.,MC_PITCHRATE_MAX=220.,MC_YAWRATE_MAX=200.,MPC_XY_VEL_MAX=5.,MPC_VEL_MANUAL=5.)
    for n,v in values.items():g.receive(msg('PARAM_VALUE',param_id=n,param_type=6 if type(v) is int else 9,param_value=encode(v) if type(v) is int else v),t)


def cycle(g,c,t,mode=(3,0),rc=1000,air=2,command=True,readback=True,thrust=.30,z=-2.):
    c.mode=mode;c.est_rx=t
    g.receive(msg('RC_CHANNELS',chan8_raw=rc),t)
    boot=1000+int(t*1000)
    g.receive(msg('LOCAL_POSITION_NED',x=0.,y=0.,z=z,vx=0.,vy=0.,vz=0.,time_boot_ms=boot),t)
    g.receive(msg('ATTITUDE',yaw=0.,roll=0.,pitch=0.),t)
    g.receive(msg('EXTENDED_SYS_STATE',landed_state=air),t)
    if thrust is not None:g.receive(msg('ATTITUDE_TARGET',thrust=thrust,time_boot_ms=boot),t)
    if readback:params(g,t)
    if command:
        p=payload(t);g.accept(p,t,p['stamp_ns'])
    g.tick(c,t)


def prepared():
    g=SuperRateGate(True);c=Core()
    for i in range(151):cycle(g,c,i*.02)
    return g,c


def test_body_rates_mask_frame_thrust_and_no_position_stream():
    g,c=prepared()
    cycle(g,c,3.02,mode=(6,0),rc=2000)
    assert g.in_offboard and g.hover_ready
    assert g.snapshot(3.02)['source']=='super'
    args=[a for n,a in c.calls if n=='set_attitude_target_send'][-1]
    assert args[1:4]==(1,1,mav.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
    assert args[5:8]==pytest.approx((.1,-.2,-.3))
    assert args[-1]==pytest.approx(.30)
    packet=mav.MAVLink_set_attitude_target_message(*args).pack(mav.MAVLink(None))
    decoded=mav.MAVLink(None).parse_buffer(packet)[0]
    assert decoded.type_mask==128 and decoded.thrust==pytest.approx(.30)
    assert all(n in ('set_attitude_target_send','param_request_read_send') for n,_ in c.calls)


@pytest.mark.parametrize('failure',['invalid','lost','rate','schedule'])
def test_offboard_fault_replaces_last_rates_and_confirms_position(failure):
    g,c=prepared();cycle(g,c,3.02,mode=(6,0),rc=2000)
    before=len(c.calls)
    t=3.20 if failure in ('lost','schedule') else 3.04
    if failure!='lost':
        p=payload(t)
        if failure=='invalid':p['valid']=False;p['phase']='REPLACEMENT_PLAN_TIMEOUT'
        if failure=='rate':p['rates_flu'][0]=2.
        g.accept(p,t,p['stamp_ns'])
    g.tick(c,t)
    assert g.terminal and g.recovery['stage']=='POSITION_REQUEST'
    outputs=c.calls[before:]
    assert not any(n=='set_attitude_target_send' for n,_ in outputs)
    hold=next(a for n,a in outputs if n=='set_position_target_local_ned_send')
    assert hold[3]==mav.MAV_FRAME_LOCAL_NED
    assert hold[5:8]==pytest.approx((0.,0.,-2.))
    assert hold[4] & 7 == 0  # Position axes ACTIVE, rates cannot remain the active control mode.
    assert hold[4] & mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    packet=mav.MAVLink_set_position_target_local_ned_message(*hold).pack(mav.MAVLink(None))
    assert mav.MAVLink(None).parse_buffer(packet)[0].z==pytest.approx(-2.)
    request=next(a for n,a in outputs if n=='command_long_send')
    assert request[2]==mav.MAV_CMD_DO_SET_MODE and request[5:7]==(3,0)
    for i in range(1,21):cycle(g,c,t+i*.02,mode=(6,0),rc=2000)
    assert not any(n=='set_attitude_target_send' for n,_ in c.calls[before:])
    count=len(c.calls);cycle(g,c,t+.42,mode=(3,0),rc=1000)
    assert g.recovery['stage']=='CONFIRMED' and len(c.calls)==count
    cycle(g,c,t+.44,mode=(6,0),rc=2000)
    assert len(c.calls)==count  # A latched fault cannot automatically re-enter control.


@pytest.mark.parametrize('failure',['EV','handoff','motion','estimator','stale_pose'])
def test_fault_recovery_preserves_ev_loss_owner(failure):
    g,c=prepared();cycle(g,c,3.02,mode=(6,0),rc=2000)
    p=payload(3.04);p['valid']=False;g.accept(p,3.04,p['stamp_ns'])
    if failure=='EV':c.ready=False
    if failure=='handoff':c.handoff.started=3.03
    if failure=='motion':c.motion.latched=True
    if failure=='estimator':c.estimator_flags=0
    if failure=='stale_pose':g.local_rx=2.
    before=len(c.calls);g.tick(c,3.04)
    assert g.terminal and g.recovery['stage']=='EV_HANDOFF'
    assert not any(n in ('set_attitude_target_send','set_position_target_local_ned_send','command_long_send') for n,_ in c.calls[before:])


def test_denied_position_requests_land_without_stopping_stabilization():
    g,c=prepared();cycle(g,c,3.02,mode=(6,0),rc=2000)
    p=payload(3.04);p['valid']=False;g.accept(p,3.04,p['stamp_ns']);g.tick(c,3.04)
    for i in range(1,132):cycle(g,c,3.04+i*.02,mode=(6,0),rc=2000)
    assert g.recovery['stage']=='LAND_REQUEST'
    requests=[a for n,a in c.calls if n=='command_long_send']
    assert requests[-1][5:7]==(4,6)
    assert all(a[2]==mav.MAV_CMD_DO_SET_MODE for a in requests)
    before=len(c.calls);cycle(g,c,5.70,mode=(4,6),rc=2000)
    assert g.recovery['stage']=='CONFIRMED' and len(c.calls)==before


def test_position_reset_and_pilot_exit_cannot_restart_fault_recovery():
    g,c=prepared();cycle(g,c,3.02,mode=(6,0),rc=2000)
    before=len(c.calls)
    g.receive(msg('LOCAL_POSITION_NED',x=10.,y=0.,z=-2.,vx=0.,vy=0.,vz=0.,time_boot_ms=4040),3.04)
    g.tick(c,3.04)
    assert g.terminal and g.recovery['stage']=='EV_HANDOFF'
    assert len(c.calls)==before
    g,c=prepared();cycle(g,c,3.02,mode=(6,0),rc=2000)
    p=payload(3.04);p['valid']=False;g.accept(p,3.04,p['stamp_ns']);g.tick(c,3.04)
    before=len(c.calls);cycle(g,c,3.06,mode=(2,0),rc=1000)
    assert g.recovery['stage']=='PILOT_MODE' and len(c.calls)==before
    cycle(g,c,3.08,mode=(6,0),rc=2000)
    assert len(c.calls)==before


@pytest.mark.parametrize('case',['disabled','monitor','ground','no_readback','unhealthy','handoff','high_at_start','bad_flags'])
def test_no_unsafe_output(case):
    g=SuperRateGate(case!='disabled');c=Core()
    if case=='monitor':c.live=False
    if case=='unhealthy':c.ready=False
    if case=='handoff':c.handoff.started=0.
    if case=='bad_flags':c.estimator_flags=0
    cycle(g,c,0.,mode=(6,0) if case=='high_at_start' else (3,0),rc=2000 if case=='high_at_start' else 1000,
          air=1 if case=='ground' else 2,readback=case!='no_readback')
    assert not any(n=='set_attitude_target_send' for n,_ in c.calls)


@pytest.mark.parametrize('case',['lost','invalid','EV','clock','jump','session','rate','thrust','stale_param','parameter_mismatch'])
def test_active_failure_stops_same_tick(case):
    g,c=prepared();before=sum(n=='set_attitude_target_send' for n,_ in c.calls)
    if case=='lost':g.tick(c,3.2)
    elif case=='EV':c.ready=False;cycle(g,c,3.02)
    elif case=='clock':c.live=False;cycle(g,c,3.02)
    elif case=='jump':
        g.receive(msg('LOCAL_POSITION_NED',x=10.,y=0.,z=-2.,vx=0.,vy=0.,vz=0.,time_boot_ms=4020),3.02);g.tick(c,3.02)
    elif case=='session':
        p=payload(3.02);p['session']='another-controller-session';g.accept(p,3.02,p['stamp_ns']);g.tick(c,3.02)
    else:
        cycle(g,c,3.01)
        before=sum(n=='set_attitude_target_send' for n,_ in c.calls)
        p=payload(3.02)
        if case=='invalid':p['valid']=False
        if case=='rate':p['rates_flu'][0]=2.
        if case=='thrust':p['acceleration']=100.
        g.accept(p,3.02,p['stamp_ns'])
        if case=='stale_param':g.cache['MPC_THR_HOVER']=(.18,-30.)
        if case=='parameter_mismatch':g.cache['COM_OBL_RC_ACT']=(3,3.02)
        g.tick(c,3.03)
    assert g.terminal
    assert sum(n=='set_attitude_target_send' for n,_ in c.calls)==before


def test_mode_exit_and_middle_pwm():
    g,c=prepared();cycle(g,c,3.02,rc=1513)
    assert not g.terminal
    cycle(g,c,3.04,rc=2000,mode=(6,0));assert g.in_offboard
    before=sum(n=='set_attitude_target_send' for n,_ in c.calls)
    cycle(g,c,3.06,mode=(4,6),rc=2000)
    assert sum(n=='set_attitude_target_send' for n,_ in c.calls)==before
    assert not g.in_offboard


def test_no_parameter_fallback_without_actual_position_thrust():
    g=SuperRateGate(True);c=Core()
    for i in range(151):cycle(g,c,i*.02,thrust=None)
    assert g.reason=='SUPER_CALIBRATING_HOVER_THRUST'
    assert g.hover_thrust is None and g.sent==0


def test_hover_scale_is_frozen_and_never_learns_own_offboard_output():
    g,c=prepared()
    cycle(g,c,3.02,mode=(6,0),rc=2000)
    for i in range(1,120):cycle(g,c,3.02+i*.02,mode=(6,0),rc=2000,thrust=.65)
    g.cache['MPC_THR_HOVER']=(.5,5.4)
    g.snapshot(5.4)
    assert g.hover_thrust==pytest.approx(.30)
    assert g.normalized_thrust==pytest.approx(.30)
    assert g.hover_handoff.frozen and not g.terminal


def test_rc8_edge_freezes_before_heartbeat_and_next_entry_relearns():
    g,c=prepared()
    cycle(g,c,3.02,rc=2000)
    assert g.hover_handoff.frozen
    cycle(g,c,3.12,mode=(6,0),rc=2000,thrust=.60)
    assert g.in_offboard and g.hover_thrust==pytest.approx(.30)
    cycle(g,c,3.14,mode=(3,0),rc=1000,thrust=.34)
    assert not g.in_offboard and not g.hover_handoff.frozen
    assert g.hover_thrust is None
    for i in range(1,151):cycle(g,c,3.14+i*.02,thrust=.34)
    cycle(g,c,6.16,mode=(6,0),rc=2000,thrust=.60)
    assert g.in_offboard and not g.terminal
    assert g.normalized_thrust==pytest.approx(.34)


def test_no_hover_calibration_during_climb_tilt_or_with_repeated_fc_stamp():
    for condition in ('climb','tilt','repeated_stamp'):
        g=SuperRateGate(True);c=Core()
        for i in range(151):
            t=i*.02
            cycle(g,c,t,thrust=None)
            if condition=='climb':g.velocity=(0.,0.,.4)
            if condition=='tilt':g.roll_pitch=(.4,0.)
            g.receive(msg('ATTITUDE_TARGET',thrust=.30,time_boot_ms=1000 if condition=='repeated_stamp' else 1000+int(t*1000)),t)
            g.tick(c,t)
        assert g.hover_thrust is None and not g.hover_handoff.ready
        assert g.sent==0


def test_stale_position_thrust_revokes_prestream_and_old_high_edge_cannot_enter():
    g,c=prepared()
    for i in range(1,30):cycle(g,c,3.+i*.02,thrust=None)
    assert g.sent>0 and g.prestream_since is None
    cycle(g,c,3.62,mode=(6,0),rc=2000,thrust=None)
    assert g.terminal and g.reason=='SUPER_HOVER_THRUST_NOT_READY'


def test_cancelled_rc8_high_does_not_keep_an_old_calibration_ready_forever():
    g,c=prepared()
    cycle(g,c,3.02,rc=1513)
    cycle(g,c,5.1,rc=1513)
    assert g.prestream_since is None
    assert g.reason=='SUPER_CALIBRATING_HOVER_THRUST'


def test_no_duplicate_sequence_or_old_epoch():
    g=SuperRateGate(True);p=payload(0.)
    with pytest.raises(ValueError):g.accept(p,1.,p['stamp_ns']+1000000000)
    p['rates_flu'][0]=math.nan
    with pytest.raises(ValueError):g.accept(p,0.,p['stamp_ns'])


def test_adapter_matches_bridge_mount_and_world_velocity():
    from zav_real_control.config import load_config
    from pathlib import Path
    from nav_msgs.msg import Odometry
    c=load_config(Path(__file__).resolve().parents[1]/'config/hardware.yaml')
    o=Odometry();o.header.frame_id=c['odom_parent'];o.child_frame_id=c['odom_child']
    q=c['body_from_imu_q_wxyz'];o.pose.pose.orientation.w,o.pose.pose.orientation.x,o.pose.pose.orientation.y,o.pose.pose.orientation.z=q
    o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=c['imu_position_body_flu_m']
    o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=rotate(conj(q),(1.,0.,0.))
    out=body_message(o,c)
    assert out.header.frame_id=='world' and out.child_frame_id=='argo_body'
    assert (out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z)==pytest.approx((0.,0.,0.))
    assert out.pose.pose.orientation.w==pytest.approx(1.)
    assert (out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z)==pytest.approx((1.,0.,0.))
    # Roll-rate lever-arm motion must not become translation of the body origin.
    o.twist.twist.angular.x,o.twist.twist.angular.y,o.twist.twist.angular.z=rotate(conj(q),(1.,0.,0.))
    from zav_real_control.geometry import cross
    vi=rotate(conj(q),cross((1.,0.,0.),c['imu_position_body_flu_m']))
    o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z=vi
    out=body_message(o,c)
    assert (out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z)==pytest.approx((0.,0.,0.),abs=1e-12)


def test_epic_mission_uses_existing_rate_gate_schema():
    gate=SuperRateGate(True)
    packet=payload(1.)
    packet['mission']={'kind':'epic','leg':1,'legs':3,'complete':False}
    gate.accept(packet,1.,packet['stamp_ns'])
    assert gate.command['mission']['kind']=='epic'
    packet=payload(2.);packet['mission']={'kind':'unknown'}
    with pytest.raises(ValueError):gate.accept(packet,2.,packet['stamp_ns'])


def test_fresh_tracking_output_sends_once_and_keeps_stream_alive():
    gate, core = prepared()
    cycle(gate, core, 3.02, mode=(6,0), rc=2000)
    gate.fresh_tracking_output = True
    first = payload(3.031)
    first["phase"] = "TRACKING"
    gate.accept(first, 3.031, first["stamp_ns"])
    gate.tick(core, 3.031)
    assert gate.last_send == 3.031
    second = payload(3.034)
    second["phase"] = "REPLANNING"
    gate.accept(second, 3.034, second["stamp_ns"])
    gate.tick(core, 3.034)
    assert gate.last_send == 3.031
    gate.tick(core, 3.042)
    assert gate.last_send == 3.042
    gate.tick(core, 3.067)
    assert gate.last_send == 3.042
    gate.tick(core, 3.083)
    assert gate.last_send == 3.083


def test_adaptive_thrust_updates_actual_wire_value_at_100hz():
    gate, core = prepared()
    from zav_real_control.thrust_model import ThrustModel
    gate.thrust_model = ThrustModel()
    gate.fresh_tracking_output = True
    cycle(gate, core, 3.02, mode=(6,0), rc=2000)
    start_sent = gate.sent
    for i in range(1, 201):
        now = 3.02 + i*.001
        if i % 10 == 0:
            p = payload(now)
            p['phase'] = 'TRACKING'
            gate.accept(p, now, p['stamp_ns'])
            core.est_rx = now
            gate.receive(msg('TIMESYNC',tc1=0,ts1=round((now-2)*1e9)),now)
            # 2 ms transport; craft produces 25 m/s² per normalized unit.
            model = gate.thrust_model
            history = [u for t,u in model.commands if .035 <= now-.002-t <= .045]
            if history:
                gate.receive(msg('HIGHRES_IMU', time_usec=round((now-2-.002)*1e6),
                                 fields_updated=4,zacc=-25*history[0]),now)
        gate.tick(core,now)
    state = gate.snapshot(now)
    assert gate.sent-start_sent == 20
    assert state['thrust_fit_updates'] >= 10
    assert state['thrust_to_acceleration'] == pytest.approx(25,rel=1e-4)
    assert [a[-1] for n,a in core.calls if n=='set_attitude_target_send'][-1] == pytest.approx(9.81/25,rel=1e-4)
    core.mode = (3,0)
    gate.tick(core,now+.001)
    assert gate.thrust_model.coefficient is None and not gate.thrust_model.commands


def test_thrust_estimator_rejects_stale_wrong_axis_and_repeated_sample():
    from zav_real_control.thrust_model import ThrustModel
    model = ThrustModel()
    model.coefficient = 32.
    model.sync(2_000_000_000,7.)
    model.commands.append((7.,.30))
    for sample,mask,az in [(2_040_000,0,-9.),(2_030_000,4,-9.),(2_041_000,4,9.)]:
        model.sample(msg('HIGHRES_IMU',time_usec=sample,fields_updated=mask,zacc=az),7.042)
    assert model.updates == 0
    m = msg('HIGHRES_IMU',time_usec=2_042_000,fields_updated=4,zacc=-9.)
    model.sample(m,7.043)
    model.commands.append((7.001,.30))
    model.sample(m,7.044)
    assert model.updates == 1
    model.sample(msg('HIGHRES_IMU',time_usec=2_044_000,fields_updated=4,zacc=-9.),7.15)
    assert model.updates == 1


def test_px4_calibrated_sticks_to_body_flu():
    from types import SimpleNamespace
    gate=SuperRateGate()
    packet=SimpleNamespace(get_type=lambda:'MANUAL_CONTROL',x=500,y=700,z=250,r=-900)
    gate.receive(packet,3.)
    assert gate.pilot_axes==(3.,[.5,-.7,-.5,.9])
    packet.z=32767
    gate.receive(packet,4.)
    assert gate.pilot_axes is None


def test_stick_stream_requires_actual_offboard_fresh_rc8_and_new_fc_sample(monkeypatch):
    from types import SimpleNamespace
    from zav_real_control import mavlink_node
    from builtin_interfaces.msg import Time
    gate=SuperRateGate(enabled=True)
    gate.in_offboard=True;gate.rc_rx=2.;gate.rc8=1950
    clock=SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=2)))
    sent=[]
    core=SimpleNamespace(offboard=gate,mode=(6,0),update=lambda *args:{},rate_samples=[])
    node=SimpleNamespace(core=core,stick_pub=SimpleNamespace(publish=sent.append),
                         get_clock=lambda:clock,last_report=2.)
    monkeypatch.setattr(mavlink_node.time,'monotonic',lambda:2.)
    def packet():
        gate.receive(SimpleNamespace(get_type=lambda:'MANUAL_CONTROL',x=1000,y=0,z=500,r=0),2.)
    packet();mavlink_node.MavlinkBridge.tick(node);mavlink_node.MavlinkBridge.tick(node)
    assert len(sent)==1 and list(sent[0].axes)==[1.,0.,0.,0.]
    core.mode=(3,0);packet();mavlink_node.MavlinkBridge.tick(node)
    core.mode=(6,0);gate.rc8=1000;packet();mavlink_node.MavlinkBridge.tick(node)
    gate.rc8=1950;gate.rc_rx=1.;packet();mavlink_node.MavlinkBridge.tick(node)
    assert len(sent)==1


@pytest.mark.parametrize(('take_position_back','backend'),[(False,'ommpc'),(True,'ommpc'),(False,'px4')])
def test_rc_ground_selector_full_gate_manual_arm_only(take_position_back,backend):
    gate,core=SuperRateGate(True),Core()
    core.armed=False;core.landed=True
    gate.throttle_low=lambda now:False  # spring-centered, including after releasing the arm gesture
    def step(index,mode,rc,stage='PRESTREAM',kind='none',air=1,z=-2.,thrust=None):
        t=index*.02
        params(gate,t)
        gate.cache['MPC_THR_HOVER']=(.31,t)
        data=payload(t);data.update(phase=stage,mission=dict(kind=kind,px4_hover_acquired=stage=='HOVER',rc_ground_seed=.31 if not core.armed else 0.))
        if backend=='px4':
            data.update(tracking_backend='px4',measured_yaw=0.,px4_reference=[0.,0.,2.4,0.,0.,0.,0.])
        gate.accept(data,t,data['stamp_ns'])
        cycle(gate,core,t,mode=mode,rc=rc,air=air,command=False,readback=False,thrust=thrust,z=z)
    for i in range(5):step(i,(3,0),1000)
    assert gate.rc_ground is None and gate.sent==0
    for i in range(5,10):step(i,(3,0),2000)
    assert gate.rc_ground.stage=='PILOT_ARM' and gate.sent==0
    assert not any(name=='command_long_send' for name,args in core.calls)
    core.armed=True
    for i in range(10,72):step(i,(3,0),2000)
    assert gate.rc_ground.stage=='MODE'
    assert any(name=='command_long_send' and args[2]==mav.MAV_CMD_DO_SET_MODE for name,args in core.calls)
    assert not any(n=='set_attitude_target_send' for n,_ in core.calls)
    step(72,(6,0),2000)
    assert gate.rc_ground.stage=='PX4_HOVER'
    assert [args[7] for n,args in core.calls if n=='set_position_target_local_ned_send'][-1]==pytest.approx(-2.4)
    core.landed=False
    if backend=='px4':
        step(73,(6,0),2000,air=2,z=-2.4)
        assert gate.rc_ground.stage=='HANDOFF'
        for i in range(74,84):step(i,(6,0),2000,stage='HOVER',air=2,z=-2.4)
        assert gate.rc_ground.stage=='COMPLETE' and gate.hover_thrust is None
        assert not any(n=='set_attitude_target_send' for n,_ in core.calls)
        return
    for i in range(73,210):step(i,(6,0),2000,air=2,z=-2.4,thrust=.26 if i%2 else .35)
    assert gate.rc_ground.stage=='PX4_HOVER' and not gate.hover_handoff.frozen
    if take_position_back:
        count=gate.sent
        step(210,(3,0),1000,air=2,z=-2.4)
        assert gate.rc_ground is None and not gate.in_offboard and gate.sent==count
        return
    for i in range(210,305):step(i,(6,0),2000,air=2,z=-2.4,thrust=.29)
    assert gate.rc_ground.stage=='HANDOFF'
    assert not any(n=='set_attitude_target_send' for n,_ in core.calls)
    step(305,(6,0),2000,stage='HOVER',air=2,z=-2.4,thrust=.29)
    step(306,(6,0),2000,stage='HOVER',air=2,z=-2.4,thrust=.29)
    assert gate.rc_ground.stage=='COMPLETE' and gate.hover_ready
    assert gate.normalized_thrust==pytest.approx(.29)
    assert not any(name=='command_long_send' and args[2]==mav.MAV_CMD_COMPONENT_ARM_DISARM for name,args in core.calls)


def test_px4_tracking_position_prestream_and_fixed_ned_transform():
    gate,core=SuperRateGate(True),Core()
    def run(i,mode,rc,x,ref):
        t=i*.02;p=payload(t)
        p.update(tracking_backend='px4',measured_yaw=math.radians(.62),position=[x,0.,2.],px4_reference=ref)
        gate.accept(p,t,p['stamp_ns'])
        cycle(gate,core,t,mode=mode,rc=rc,command=False,thrust=None)
    for i in range(70):run(i,(3,0),1000,1.,[1.,0.,2.,0.,0.,0.,0.])
    run(70,(6,0),2000,1.,[1.,0.,2.,0.,0.,0.,0.])
    run(72,(6,0),2000,1.4,[2.,-38.,2.4,.1,.2,.5,.1])
    messages=[args for n,args in core.calls if n=='set_position_target_local_ned_send']
    m=messages[-1]
    assert m[5:11]==pytest.approx((1.,38.,-2.4,.2,-.5,-.1))
    assert m[-2]==pytest.approx(-.1) and m[4]&0x3f==0
    assert gate.hover_thrust is None and not any(n=='set_attitude_target_send' for n,_ in core.calls)
