"""No UART or ROS: capture every automatic flight command with a fake sender."""
from types import SimpleNamespace
import pytest
from pymavlink.dialects.v20 import common as mav
from zav_real_control.super_auto import GroundTakeoff
from zav_real_control.hover_handoff import HoverHandoff


def setup():
    calls=[]
    core=SimpleNamespace(armed=False,landed=True,mode=(3,0),parameters=SimpleNamespace(target=(1,1)),
        connection=SimpleNamespace(mav=SimpleNamespace(command_long_send=lambda *a:calls.append(a))))
    gate=SimpleNamespace(rc8=2000,velocity=(0.,0.,0.),roll_pitch=(0.,0.),position=(0.,0.,0.),
        cache={'MPC_THR_HOVER':(.3,0.)},
        hover_handoff=HoverHandoff(),in_air=False,in_offboard=False,command={'phase':'TAKEOFF_WAIT'},
        throttle_low=lambda now:True,reason='',abort=lambda reason:calls.append(('abort',reason)))
    return GroundTakeoff(True,.3),gate,core,calls


def step(a,g,c,t,**changes):
    return a.step(g,c,t,changes.get('healthy',True),changes.get('command_ok',True),changes.get('issue'))


def test_mode_after_prestream_arm_only_after_actual_offboard_no_force():
    a,g,c,calls=setup()
    assert step(a,g,c,0)==0.
    step(a,g,c,1.19);assert not calls
    step(a,g,c,1.21);assert calls[-1][2]==mav.MAV_CMD_DO_SET_MODE
    assert not any(x[2]==mav.MAV_CMD_COMPONENT_ARM_DISARM for x in calls)
    c.mode=(6,0);step(a,g,c,1.3)
    assert calls[-1][2]==mav.MAV_CMD_COMPONENT_ARM_DISARM
    assert calls[-1][4:6]==(1,0)  # normal arm, no force flag
    c.armed=True;assert step(a,g,c,1.4)==0.
    assert step(a,g,c,1.9)==pytest.approx(.5)
    assert step(a,g,c,2.4)==pytest.approx(1.)
    assert g.hover_handoff.frozen and g.hover_thrust==.3


@pytest.mark.parametrize('reason',['unhealthy','low_rc8','throttle','already_armed','airborne','tilt','unknown_scale','parameter_mismatch'])
def test_no_arm_for_failed_ground_preconditions(reason):
    if reason=='unknown_scale':
        with pytest.raises(ValueError):GroundTakeoff(True,0.)
        return
    a,g,c,calls=setup()
    if reason=='low_rc8':g.rc8=1000
    if reason=='throttle':g.throttle_low=lambda now:False
    if reason=='already_armed':c.armed=True
    if reason=='airborne':c.landed=False
    if reason=='tilt':g.roll_pitch=(.4,0.)
    if reason=='parameter_mismatch':g.cache['MPC_THR_HOVER']=(.18,0.)
    for t in (0,1,2,3):step(a,g,c,t,healthy=reason!='unhealthy')
    assert not any(x[0]!='abort' for x in calls)


def test_fc_arm_rejection_is_terminal_and_not_bypassed():
    a,g,c,calls=setup();step(a,g,c,0);step(a,g,c,1.21);c.mode=(6,0);step(a,g,c,1.3)
    a.receive(SimpleNamespace(get_type=lambda:'COMMAND_ACK',command=mav.MAV_CMD_COMPONENT_ARM_DISARM,
                              result=mav.MAV_RESULT_DENIED),1.4)
    step(a,g,c,1.5)
    assert a.stage=='FAILED' and calls[-1][0]=='abort'


def test_rc_takeover_after_start_never_rearms():
    a,g,c,calls=setup();step(a,g,c,0);g.rc8=1000;step(a,g,c,.1)
    assert a.stage=='FAILED' and calls[-1][0]=='abort'


def test_no_liftoff_ground_abort_never_disarms_airborne_aircraft():
    for in_air,landed,height in ((False,True,0.),(True,False,.4),(True,False,0.)):
        a,g,c,calls=setup();step(a,g,c,0);step(a,g,c,1.21);c.mode=(6,0);step(a,g,c,1.3)
        c.armed=True;step(a,g,c,1.4);g.in_air=in_air;c.landed=landed;g.position=(0.,0.,-height)
        step(a,g,c,10.)
        disarms=[x for x in calls if x[0]!='abort' and x[2]==mav.MAV_CMD_COMPONENT_ARM_DISARM and x[4]==0]
        assert bool(disarms)==(not in_air and landed and height==0.)
