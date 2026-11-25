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


def cycle(g,c,t,mode=(3,0),rc=1000,air=2,command=True,readback=True,thrust=.30):
    c.mode=mode;c.est_rx=t
    g.receive(msg('RC_CHANNELS',chan8_raw=rc),t)
    boot=1000+int(t*1000)
    g.receive(msg('LOCAL_POSITION_NED',x=0.,y=0.,z=-2.,vx=0.,vy=0.,vz=0.,time_boot_ms=boot),t)
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
