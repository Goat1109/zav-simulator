"""Fake MAVLink only: RTH LAND is conditional and stops rate output on FC LAND."""
import pytest
from pymavlink.dialects.v20 import common as mav
from test_super_gate import prepared,cycle,payload


def ready_land():
    gate,core=prepared();cycle(gate,core,3.02,mode=(6,0),rc=2000)
    core.landed=False;core.landed_rx=3.02
    core.connection.mav.command_long_send=lambda *args:core.calls.append(('command_long_send',args))
    gate.cache['COM_DISARM_LAND']=(2.,3.04)
    p=payload(3.04);p.update(intent='land',phase='RTH_LANDING',position=[0.,0.,2.],goal=[0.,0.,2.],
        mission={'kind':'rth','home_valid':True,'leg':2,'legs':2})
    return gate,core,p


def test_land_request_then_actual_land_owns_output_and_px4_disarms():
    g,c,p=ready_land();g.accept(p,3.04,p['stamp_ns']);g.tick(c,3.04)
    requests=[args for name,args in c.calls if name=='command_long_send']
    assert len(requests)==1 and requests[0][2]==mav.MAV_CMD_NAV_LAND
    before=sum(name=='set_attitude_target_send' for name,_ in c.calls)
    c.mode=(4,6);g.tick(c,3.06)
    assert sum(name=='set_attitude_target_send' for name,_ in c.calls)==before
    assert g.reason=='SUPER_RTH_LANDING'
    c.armed=False;c.landed=True;c.landed_rx=3.08;g.tick(c,3.08)
    assert g.reason=='SUPER_RTH_LANDED'
    assert not any(args[2]==mav.MAV_CMD_COMPONENT_ARM_DISARM for name,args in c.calls if name=='command_long_send')


@pytest.mark.parametrize('case',['EV_failure','far_goal','moving','no_home','disarm_disabled','RC8_low','wrong_intent'])
def test_no_land_command_for_invalid_or_overridden_return(case):
    g,c,p=ready_land()
    if case=='EV_failure':c.motion.latched=True
    if case=='far_goal':p['position'][0]=.5
    if case=='moving':g.velocity=(1.,0.,0.)
    if case=='no_home':p['mission']['home_valid']=False
    if case=='disarm_disabled':g.cache['COM_DISARM_LAND']=(0.,3.04)
    if case=='RC8_low':g.rc8=1000
    if case=='wrong_intent':p['mission']['kind']='forward'
    g.accept(p,3.04,p['stamp_ns']);g.tick(c,3.04)
    assert not any(name=='command_long_send' for name,_ in c.calls)
    assert g.terminal
