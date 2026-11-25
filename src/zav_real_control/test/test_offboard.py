"""Offline reference and fake-connection tests. Never opens any transport."""
from dataclasses import asdict
import math
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import common as mav
from zav_real_control.offboard import Options, Mission, OffboardControl, smooth, wrap


def message(kind, **fields):
    return SimpleNamespace(get_type=lambda:kind, **fields)


class FakeCore:
    def __init__(self):
        self.calls=[]
        self.connection=SimpleNamespace(mav=SimpleNamespace(set_position_target_local_ned_send=lambda *args:self.calls.append(args)))
        self.parameters=SimpleNamespace(target=(1,1))
        self.handoff=SimpleNamespace(started=None)
        self.motion=SimpleNamespace(latched=False)
        self.armed=True
        self.est_rx=0.
        self.estimator_flags=(mav.ESTIMATOR_ATTITUDE | mav.ESTIMATOR_VELOCITY_HORIZ | mav.ESTIMATOR_VELOCITY_VERT | mav.ESTIMATOR_POS_HORIZ_REL | mav.ESTIMATOR_POS_VERT_ABS)
        self.mode=(3,0)
        self.ready=True
        self.live=True
    def writable(self): return self.live
    def snapshot(self,now): return {'ready':self.ready}
    def fresh(self,rx,now,age): return rx is not None and 0<=now-rx<age


def request(o,t,session='a'*32,seq=None,pattern='front'):
    return dict(session=session,sequence=int(t*1000)+1 if seq is None else seq,
                stamp_ns=int((1_800_000_000+t)*1e9),pattern=pattern,options=asdict(Options()))


def tick(o,c,t,rc=1000,mode=(3,0),air=2,position=(10.,20.,-2.),yaw=math.pi/2,heartbeat=True):
    c.mode=mode;c.est_rx=t
    o.receive(message('RC_CHANNELS',chan8_raw=rc),t)
    o.receive(message('LOCAL_POSITION_NED',x=position[0],y=position[1],z=position[2],vx=0.,vy=0.,vz=0.,time_boot_ms=int(t*1000)),t)
    o.receive(message('ATTITUDE',yaw=yaw),t)
    o.receive(message('EXTENDED_SYS_STATE',landed_state=air),t)
    if heartbeat and o.command_state == 'running':
        r=request(o,t,session=o.session,pattern=o.request[0]);o.accept(r,t,r['stamp_ns'])
    o.tick(c,t)


@pytest.mark.parametrize('pattern,end',[('front',(5,0)),('right',(0,5)),('back',(-5,0)),('left',(0,-5)),('square',(0,0)),('circle',(0,0))])
def test_all_paths_complete_keep_height_and_entry_heading(pattern,end):
    origin=(10.,20.,-2.,math.pi/2);m=Mission(pattern,Options(),origin,0.)
    position=origin[:3];yaw=origin[3];last=origin;speedmax=0.;clockwise_turns=0
    for i in range(1,14000):
        t=i*.05;out=m.step(t,position,(0.,0.,0.),yaw)
        assert out is not None,m.error
        assert out[2]==-2.
        speedmax=max(speedmax,math.dist(out[:3],last[:3])/.05)
        assert abs(wrap(out[3]-last[3]))/.05 <= math.radians(m.options.yaw_rate_deg_s)+.001
        position,yaw=out[:3],out[3];last=out
        if m.phase=='COMPLETE_HOLD':break
    assert m.phase=='COMPLETE_HOLD'
    assert out[:2]==pytest.approx((origin[0]-end[1],origin[1]+end[0]),abs=1e-6)
    assert wrap(out[3]-origin[3])==pytest.approx(0.,abs=1e-6)
    assert speedmax<=m.options.speed_m_s+.001


def test_square_stops_before_each_quarter_turn():
    m=Mission('square',Options(),(0.,0.,-2.,0.),0.)
    position=(0.,0.,-2.);yaw=0.;turns=set();move_phases=set()
    for i in range(1,14000):
        old=m.target;out=m.step(i*.05,position,(0.,0.,0.),yaw)
        if m.phase=='SEGMENT' and m.index%2:
            turns.add(m.index)
            assert out[:3]==pytest.approx(old[:3],abs=1e-8)
        position,yaw=out[:3],out[3]
        if m.phase=='COMPLETE_HOLD':break
    assert turns=={1,3,5,7}


def test_does_not_turn_until_vehicle_stopped():
    m=Mission('square',Options(),(0.,0.,-2.,0.),0.)
    m.phase='SEGMENT';m.segment_since=0.
    m.target=(5.,0.,-2.,0.)
    out=m.step(20.,(4.5,0.,-2.),(.3,0.,0.),0.)
    assert m.index==0 and m.phase=='SEGMENT' and out[3]==0.
    assert m.step(21.,(1.,0.,-2.),(0.,0.,0.),0.) is None
    assert m.error=='tracking_error'


def prepare_hover(o,c):
    # No test node exists during Position prestream or RC8 transition.
    for i in range(31):tick(o,c,i*.05)
    tick(o,c,1.55,rc=1513)
    tick(o,c,1.6,rc=2000)
    for i in range(22):tick(o,c,1.65+i*.05,rc=2000,mode=(6,0))
    assert o.hover_ready and o.mission is None and not o.terminal
    return 2.7


def start_command(o,t,session='a'*32,pattern='front'):
    r=request(o,t,session=session,pattern=pattern)
    o.accept(r,t,r['stamp_ns'])


def test_independent_hover_then_command_and_intermediate_pwm_is_not_an_exit():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    assert o.reason=='OFFBOARD_HOVER_READY' and o.session is None
    anchor=o.target
    start_command(o,t,pattern='square')
    tick(o,c,t+.05,rc=1513,mode=(6,0))
    assert o.command_state=='running' and not o.terminal
    assert o.mission.phase=='ENTRY_HOLD' and o.target==anchor
    args=c.calls[-1]
    assert args[1:4]==(1,1,mav.MAV_FRAME_LOCAL_NED)
    assert args[4]==2552 and args[5:8]==(10.,20.,-2.)
    assert args[-2]==pytest.approx(math.pi/2)
    # Pilot returns to Position: no permanent fault and no stale path on reentry.
    tick(o,c,t+.1,rc=1000)
    assert o.command_state=='cancelled' and not o.terminal
    for i in range(30):tick(o,c,t+.15+i*.05)
    for i in range(24):tick(o,c,t+1.65+i*.05,rc=2000,mode=(6,0))
    assert o.hover_ready and o.command_state=='cancelled'
    start_command(o,t+2.85,session='b'*32,pattern='right')
    assert o.command_state=='running' and o.request[0]=='right'


def test_early_command_rejected_and_never_autostarts_on_switch():
    o=OffboardControl(True);c=FakeCore()
    start_command(o,0.)
    assert o.last_rejection['reason']=='command_requires_offboard_hover'
    t=prepare_hover(o,c)
    start_command(o,t)  # Same old process/session must remain rejected.
    assert o.mission is None
    start_command(o,t+.01,session='b'*32)
    assert o.command_state=='running'


@pytest.mark.parametrize('case',['disabled','dry_bridge','ground','unknown_air','no_RC_edge','bad_EV','bad_flags'])
def test_no_setpoints_without_all_interlocks(case):
    o=OffboardControl(case!='disabled');c=FakeCore()
    if case=='dry_bridge':c.live=False
    if case=='bad_EV':c.ready=False
    if case=='bad_flags':c.estimator_flags=0
    for i in range(5):
        tick(o,c,i*.05,rc=2000 if case=='no_RC_edge' else 1000,
             mode=(6,0) if case=='no_RC_edge' else (3,0),air=1 if case=='ground' else 0 if case=='unknown_air' else 2)
    assert not c.calls


@pytest.mark.parametrize('failure',['EV_loss','local_stale','reset','gap','handoff','serial_error'])
def test_fatal_feedback_failure_stops_even_without_test_node(failure):
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c);before=len(c.calls)
    if failure=='EV_loss':c.ready=False;tick(o,c,t+.05,rc=2000,mode=(6,0))
    elif failure=='local_stale':o.local_rx=0.;o.tick(c,t+.05)
    elif failure=='reset':tick(o,c,t+.05,rc=2000,mode=(6,0),position=(100.,20.,-2.))
    elif failure=='gap':tick(o,c,t+.4,rc=2000,mode=(6,0))
    elif failure=='handoff':c.handoff.started=t;tick(o,c,t+.05,rc=2000,mode=(6,0))
    elif failure=='serial_error':o.abort('serial_io_error')
    assert o.terminal and len(c.calls)==before


@pytest.mark.parametrize('mode',[(4,6),(4,5),(2,0),(7,0)])
def test_pilot_other_modes_stop_output_without_overwriting(mode):
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t);before=len(c.calls)
    tick(o,c,t+.05,rc=2000,mode=mode)
    assert len(c.calls)==before and o.command_state=='cancelled' and not o.terminal


def test_node_lost_cancels_path_but_bridge_keeps_hover_and_old_session_never_restarts():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t)
    for i in range(1,11):tick(o,c,t+i*.05,rc=2000,mode=(6,0),heartbeat=False)
    assert o.command_state=='cancelled' and o.command_reason=='test_node_lost'
    assert not o.terminal and t+.5-o.last_send<.1
    start_command(o,t+.51)
    assert o.command_state=='cancelled'
    for i in range(11,36):tick(o,c,t+i*.05,rc=2000,mode=(6,0),heartbeat=False)
    start_command(o,t+1.76,session='b'*32)
    assert o.command_state=='running'


def test_another_session_cannot_replace_active_command():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t);start_command(o,t+.01,session='b'*32)
    assert o.session=='a'*32 and o.command_state=='running'
    assert o.last_rejection['reason']=='another_test_running'


def test_high_at_start_cannot_start_mission():
    o=OffboardControl(True);c=FakeCore()
    for i in range(50):tick(o,c,i*.05,rc=2000,mode=(6,0))
    assert o.mission is None and o.sent==0


def test_low_cancels_path_before_position_heartbeat_but_middle_does_not():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t)
    tick(o,c,t+.05,rc=1513,mode=(6,0))
    assert o.command_state=='running'
    tick(o,c,t+.1,rc=1000,mode=(6,0))
    assert o.command_state=='cancelled' and not o.terminal
    assert o.target==(*o.position,o.yaw)


def test_position_high_never_starts_path_without_actual_offboard():
    o=OffboardControl(True);c=FakeCore()
    for i in range(31):tick(o,c,i*.05)
    for i in range(1,101):tick(o,c,1.5+i*.05,rc=2000)
    start_command(o,6.51)
    assert o.mission is None and o.reason=='WAITING_FOR_FC_OFFBOARD'
    assert not o.terminal


@pytest.mark.parametrize('kwargs',[{'speed_m_s':3.01},{'distance_m':100.},{'radius_m':0.},{'yaw_rate_deg_s':61.},{'speed_m_s':math.nan}])
def test_invalid_options_rejected(kwargs):
    with pytest.raises(ValueError):Options(**kwargs)


def test_wire_message_is_position_and_yaw_only():
    o=OffboardControl(True);c=FakeCore();tick(o,c,0.)
    packet=mav.MAVLink_set_position_target_local_ned_message(*c.calls[-1]).pack(mav.MAVLink(None))
    decoded=mav.MAVLink(None).parse_buffer(packet)[0]
    assert decoded.get_type()=='SET_POSITION_TARGET_LOCAL_NED'
    assert decoded.coordinate_frame==mav.MAV_FRAME_LOCAL_NED and decoded.type_mask==2552
    assert (decoded.x,decoded.y,decoded.z)==pytest.approx((10.,20.,-2.))
    assert decoded.yaw==pytest.approx(math.pi/2)


def test_single_link_ev_failure_preempts_offboard_same_tick():
    from pathlib import Path
    from zav_real_control.single_link import SingleLink
    from zav_real_control.config import load_config
    config=load_config(Path(__file__).resolve().parents[1]/'config/hardware.yaml')
    config['mavlink']['rc_mode_channel']=7
    class Capture:
        def __init__(self):self.calls=[];self.mav=self
        def recv_match(self,blocking=False):return None
        def __getattr__(self,k):return lambda *a,**kw:self.calls.append((k,a))
    c=Capture();s=SingleLink(c,config,True);s.offboard.enabled=True
    s.parameters.version=tuple(config['expected_firmware'])
    s.parameters.value=0
    flags=FakeCore().estimator_flags
    epoch_ns=1_800_000_000_000_000_000
    def cycle(t,mode=(3,0),rc8=1000):
        def receive(kind,**kw):
            m=message(kind,**kw);m.get_srcSystem=lambda:1;m.get_srcComponent=lambda:1
            s.receive(m,t,epoch_ns+int(t*1e9))
        receive('HEARTBEAT',autopilot=mav.MAV_AUTOPILOT_PX4,base_mode=128,custom_mode=(mode[0]<<16)|(mode[1]<<24))
        receive('RC_CHANNELS',chan7_raw=1000,chan8_raw=rc8)
        receive('ATTITUDE',yaw=0.)
        receive('LOCAL_POSITION_NED',x=10.,y=20.,z=-2.,vx=0.,vy=0.,vz=0.,time_boot_ms=int(t*1000))
        receive('EXTENDED_SYS_STATE',landed_state=2)
        receive('ESTIMATOR_STATUS',flags=flags)
        receive('NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1000+int(t*1000),value=1.)
        receive('NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1000+int(t*1000),value=0.)
        s.parameters.value_rx=t
        ns=epoch_ns+int(t*1e9)
        s.odometry(ns//1000,(0.,0.,0.),(1.,0.,0.,0.),(0.,0.,0.),(0.,0.,0.),t,ns//1000,('camera_init','mid360_imu'))
        s.update(t,ns)
    for i in range(31):cycle(i*.05)
    cycle(1.55,(6,0),2000)
    for i in range(1,24):cycle(1.55+i*.05,(6,0),2000)
    start_command(s.offboard,2.7)
    assert s.offboard.mission is not None
    count=sum(k=='set_position_target_local_ned_send' for k,_ in c.calls)
    s.motion.latched=True;s.motion.latch_reasons=['position_residual'];s.begin_handoff(2.75)
    cycle(2.75,(6,0),2000)
    assert s.offboard.terminal
    assert sum(k=='set_position_target_local_ned_send' for k,_ in c.calls)==count
    # Existing barometer handoff is allowed; new code never emits arm/mode/takeoff.
    assert not any(k=='command_long_send' and a[2] in (400,176,22) for k,a in c.calls)


@pytest.mark.parametrize('pattern',['front','circle'])
def test_fast_reference_acceleration_and_speed_limits(pattern):
    m=Mission(pattern,Options(),(0.,0.,-2.,0.),0.)
    m.phase='SEGMENT'
    duration=m.duration(m.path[0]);previous=(0.,0.,-2.);previous_v=None
    dt=.005;max_speed=0.;max_acc=0.
    for i in range(1,int(duration/dt)+3):
        target=m.step(i*dt,previous,(0.,0.,0.),0.)
        assert target is not None
        v=tuple((a-b)/dt for a,b in zip(target[:3],previous))
        max_speed=max(max_speed,math.hypot(*v))
        if previous_v is not None:
            max_acc=max(max_acc,math.hypot(*((a-b)/dt for a,b in zip(v,previous_v))))
        previous,previous_v=target[:3],v
    assert max_speed<=m.options.speed_m_s+.01
    assert max_acc<=m.options.acceleration_m_s2+.02
    assert duration<15.  # Short paths are much faster than the original 0.5 m/s profile.


def test_progress_and_trigger_error_frozen_separately_from_current_error():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t)
    snap=o.snapshot(t)
    assert snap['goal_relative_front_right_up_m']==pytest.approx((5.,0.,0.))
    assert snap['position_relative_front_right_up_m']==pytest.approx((0.,0.,0.))
    # Reproduce the reported divergence: 1.123m target, 0.141m actual, then >1m.
    o.target=(11.15,20.,-2.,math.pi/2);o.mission.target=o.target
    o.position=(10.14,20.,-2.)
    o.observed_now=t+.1;o.abort('tracking_error')
    before=o.snapshot(t+.1)
    assert before['fault']['tracking_error_m']==pytest.approx(1.01)
    o.position=(11.07,20.,-2.)
    after=o.snapshot(t+20.)
    assert after['reference_progress_pct']==before['reference_progress_pct']
    assert after['elapsed_s']==before['elapsed_s']
    assert after['fault']==before['fault']
    assert after['tracking_error_m']==pytest.approx(.08)
    assert after['setpoint_age_s']>before['setpoint_age_s']


@pytest.mark.parametrize('pattern',['front','circle','square'])
def test_stationary_aircraft_cannot_be_outpaced_by_reference(pattern):
    m=Mission(pattern,Options(),(0.,0.,-2.,0.),0.)
    for i in range(1,1001):
        out=m.step(i*.02,(0.,0.,-2.),(0.,0.,0.),0.)
        assert out is not None
        assert math.dist(out[:3],(0.,0.,-2.))<=.500001
    assert m.reference_paused and m.phase=='SEGMENT'
    assert m.step(110.,(0.,0.,-2.),(0.,0.,0.),0.) is None
    assert m.error=='segment_timeout'


def test_slow_aircraft_finishes_without_time_only_tracking_fault():
    m=Mission('front',Options(),(0.,0.,-2.,0.),0.)
    position=(0.,0.,-2.)
    for i in range(1,3000):
        out=m.step(i*.02,position,(0.,0.,0.),0.)
        assert out is not None,m.error
        # Deliberately slow simulated response, well below nominal 3m/s profile.
        delta=out[0]-position[0]
        position=(position[0]+min(.008,max(-.008,delta)),0.,-2.)
        if m.phase=='COMPLETE_HOLD':break
    assert m.phase=='COMPLETE_HOLD'
    assert position[0]==pytest.approx(5.)


def test_yaw_reference_waits_for_slow_turning_aircraft():
    m=Mission('square',Options(),(0.,0.,-2.,0.),0.)
    m.index=1;m.start=(5.,0.,0.);m.phase='SEGMENT';m.target=(5.,0.,-2.,0.)
    for i in range(1,1001):
        out=m.step(i*.02,(5.,0.,-2.),(0.,0.,0.),0.)
        assert out is not None
        assert math.degrees(out[3])<=20.00001
    assert m.reference_paused and m.index==1


def test_complete_holds_without_client_and_accepts_next_command_at_new_origin():
    o=OffboardControl(True);c=FakeCore();t=prepare_hover(o,c)
    start_command(o,t)
    position=o.position;yaw=o.yaw
    for i in range(1,1800):
        tick(o,c,t+i*.05,rc=2000,mode=(6,0),position=position,yaw=yaw)
        position,yaw=o.target[:3],o.target[3]
        if o.command_state=='complete':break
    assert o.command_state=='complete'
    for j in range(1,32):
        tick(o,c,t+(i+j)*.05,rc=2000,mode=(6,0),position=position,yaw=yaw,heartbeat=False)
    assert o.hover_ready and not o.terminal
    start_command(o,t+(i+32)*.05,session='b'*32,pattern='right')
    assert o.command_state=='running'
    assert o.mission.origin[:3]==pytest.approx(position)
