import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from zav_real_control.geometry import body_odometry, rotate, conj
from zav_real_control.clock import validate_sample, validate_dds_clock
from zav_real_control.handoff import Handoff
from zav_real_control.motion import LioSafety
from zav_real_control.config import load_config
from zav_real_control.mavlink_parameters import ParameterLink, encode, decode

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('key,value',[
    ('timesync_initial_rate_hz',0),('timesync_initial_rate_hz',101),
    ('timesync_initial_rate_hz',True),('timesync_steady_rate_hz',0),
    ('ftdi_latency_timer_ms',0),('ftdi_latency_timer_ms',1.5)])
def test_invalid_clock_configuration_is_rejected(tmp_path,key,value):
    c=yaml.safe_load((ROOT/'config/hardware.yaml').read_text())
    c['mavlink'][key]=value
    p=tmp_path/'hardware.yaml';p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match=f'Invalid mavlink.{key}'):load_config(p)


def test_old_config_gets_validated_fast_sync_defaults(tmp_path):
    c=yaml.safe_load((ROOT/'config/hardware.yaml').read_text())
    for key in ('timesync_initial_rate_hz','timesync_steady_rate_hz','ftdi_latency_timer_ms'):
        c['mavlink'].pop(key)
    p=tmp_path/'hardware.yaml';p.write_text(yaml.safe_dump(c))
    link=load_config(p)['mavlink']
    assert (link['timesync_initial_rate_hz'],link['timesync_steady_rate_hz'],
            link['ftdi_latency_timer_ms'])==(100,10,1)


def test_mount_and_rotating_lever_arm():
    q=(math.cos(math.radians(11)),0,math.sin(math.radians(11)),0)
    lever=(.0388766653,.02329,.1518166791)
    # Body is level, stationary at origin, rotating about body Z at 1 rad/s.
    v_b=(-lever[1],lever[0],0)
    result=body_odometry(lever,q,rotate(conj(q),v_b),rotate(conj(q),(0,0,1)),q,lever)
    assert result[0]==pytest.approx((0,0,0))
    assert result[1]==pytest.approx((1,0,0,0))
    assert result[2]==pytest.approx((0,0,0),abs=1e-12)
    assert result[3]==pytest.approx((0,0,-1))


def test_yaw_and_world_velocity():
    q=(math.sqrt(.5),0,0,math.sqrt(.5))
    p,out,v,w=body_odometry((1,2,3),q,(1,0,0),(0,0,0),(1,0,0,0),(0,0,0))
    assert p==(1,-2,-3)
    assert out==pytest.approx((math.sqrt(.5),0,0,-math.sqrt(.5)))
    assert v==pytest.approx((0,-1,0),abs=1e-12)


@pytest.mark.parametrize('q',[(0,0,0,0),(math.nan,0,0,0)])
def test_bad_orientation(q):
    with pytest.raises(ValueError):body_odometry((0,0,0),q,(0,0,0),(0,0,0),(1,0,0,0),(0,0,0))


def test_motion_velocity_prediction_and_position_latch():
    s=LioSafety();s.set_armed(True)
    assert s.odometry(1,(0,0,0),(1,0,0,0),(2,0,0))
    assert s.odometry(1.02,(.04,0,0),(1,0,0,0),(2,0,0))
    assert not s.odometry(1.04,(2,0,0),(1,0,0,0),(2,0,0))
    assert s.latched and s.latch_reasons==['position_residual']
    s.set_armed(False)
    assert s.latched


def test_attitude_latch_and_quaternion_sign():
    s=LioSafety();s.set_armed(True)
    assert s.odometry(1,(0,0,0),(1,0,0,0),(0,0,0))
    assert s.odometry(1.01,(0,0,0),(-1,0,0,0),(0,0,0))
    assert not s.odometry(1.02,(0,0,0),(.70710678,0,0,.70710678),(0,0,0))
    assert s.latch_reasons==['attitude_residual']


def test_gap_and_invalid_are_not_landing_triggers():
    s=LioSafety();s.set_armed(True)
    s.odometry(1,(0,0,0),(1,0,0,0),(0,0,0))
    assert s.odometry(2,(100,0,0),(1,0,0,0),(0,0,0))
    assert s.comparison=='gap_rebaseline'
    assert not s.odometry(2,(100,0,0),(1,0,0,0),(0,0,0))
    assert not s.odometry(2.01,(math.nan,0,0),(1,0,0,0),(0,0,0))
    assert not s.latched


def test_sample_epoch_and_age():
    n=1_800_000_000_000_000
    assert validate_sample(n-10000,n,0,.25) is None
    assert validate_sample(100,n,0,.25)=='source_not_ros_epoch'
    assert validate_sample(n,n,n,.25)=='nonmonotonic_source_stamp'
    assert validate_sample(n-300000,n,0,.25)=='source_measurement_age'
    assert validate_sample(n+10000,n,0,.25)=='source_measurement_age'


def test_dds_clock_contract():
    c=load_config(ROOT/'config/dds_legacy.yaml');n=1_800_000_000_000_000
    sync=dict(source_protocol=2,round_trip_time=100)
    assert validate_dds_clock(n,n,1,sync,1,1,c) is None
    assert validate_dds_clock(n,100,1,sync,1,1,c)=='px4_dds_not_ros_epoch_or_clock_skew'
    assert validate_dds_clock(n,n,1,sync,1,5,c)=='px4_status_missing_or_stale'
    assert validate_dds_clock(n,n,5,sync,1,5,c)=='dds_timesync_missing_or_stale'
    sync['source_protocol']=1
    assert validate_dds_clock(n,n,1,sync,1,1,c)=='dds_timesync_not_converged'


def advance(h,t=0,**kw):
    args=dict(now=t,latched=True,armed=True,status_fresh=True,nav_state=2,intention=2,
              parameter_on=False,baro_good=False,vertical_good=False)
    args.update(kw);return h.update(**args)


def test_land_waits_for_fusion_and_vertical_estimate():
    h=Handoff()
    assert not advance(h)['send_land']
    assert not advance(h,.1,parameter_on=True)['send_land']
    assert not advance(h,.2,parameter_on=True,baro_good=True)['send_land']
    r=advance(h,.3,parameter_on=True,baro_good=True,vertical_good=True)
    assert r['send_land'] and r['baro_confirmed']
    assert not advance(h,.4)['send_land']
    assert not advance(h,2,nav_state=18)['send_land']
    assert advance(h,3)['send_land']


def test_timeout_is_best_effort_not_confirmation():
    h=Handoff();advance(h)
    r=advance(h,3.1)
    assert r['send_land'] and r['baro_timeout'] and not r['baro_confirmed']


def test_explicit_pilot_takeover_and_disarm():
    h=Handoff();advance(h)
    assert advance(h,1,intention=1)['pilot_takeover']
    assert not advance(h,4)['send_land']
    assert advance(h,4)['enable_baro']
    h=Handoff();advance(h)
    assert not advance(h,4,armed=False)['send_land']
    h=Handoff();advance(h)
    assert not advance(h,1,nav_state=1,intention=2)['pilot_takeover']


class Connection:
    def __init__(self):self.queue=[];self.calls=[];self.mav=self
    def recv_match(self,blocking=False):return self.queue.pop(0) if self.queue else None
    def __getattr__(self,name):return lambda *args:self.calls.append((name,args))
    def message(self,kind,source=(1,1),**kwargs):
        self.queue.append(SimpleNamespace(get_type=lambda:kind,get_srcSystem=lambda:source[0],
                         get_srcComponent=lambda:source[1],**kwargs))


def link():
    c=Connection();p=ParameterLink(c,(1,14,3));p.target=(1,1)
    c.message('HEARTBEAT');return p,c


def version(c,v=(1,14,3)):
    c.message('AUTOPILOT_VERSION',flight_sw_version=(v[0]<<24)|(v[1]<<16)|(v[2]<<8))


def sets(c):return [x for x in c.calls if x[0]=='param_set_send']


def test_bytewise_parameter_and_confirmed_readback():
    assert decode(encode(1))==1 and encode(1)!=1.
    p,c=link();version(c)
    p.tick(1,True,0,True)
    assert len(sets(c))==1 and sets(c)[0][1][3]==encode(1)
    assert not p.snapshot(1,0)['confirmed']
    c.message('PARAM_VALUE',param_type=6,param_id=b'EKF2_BARO_CTRL',param_value=encode(1))
    p.tick(1,True,.6,True)
    assert p.snapshot(1,.6)['confirmed']
    assert not p.snapshot(1,3)['confirmed']


@pytest.mark.parametrize('mode',['unknown','wrong','readonly','armed_disable','stale'])
def test_no_parameter_write_without_conditions(mode):
    p,c=link()
    if mode!='unknown':version(c,(1,15,0) if mode=='wrong' else (1,14,3))
    p.poll(0)
    p.tick(0 if mode=='armed_disable' else 1,True,4 if mode=='stale' else 0,mode!='readonly')
    assert not sets(c)


def test_wrong_source_cannot_confirm_parameter():
    p,c=link();version(c)
    c.message('PARAM_VALUE',source=(2,1),param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(1))
    p.tick(1,True,0,False)
    assert not p.snapshot(1,0)['confirmed']


def test_config_rejects_shared_uart(tmp_path):
    c=yaml.safe_load((ROOT/'config/dds_legacy.yaml').read_text())
    c['mavlink']['connection']=c['agent']['device']
    p=tmp_path/'config.yaml';p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match='share'):load_config(p)


def test_firmware_preparation_is_idempotent_and_checks_abi(tmp_path):
    spec=importlib.util.spec_from_file_location('prepare',ROOT/'scripts/prepare_px4.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    msgs=tmp_path/'messages';msgs.mkdir();(tmp_path/'msg').mkdir()
    for name in module.REQUIRED:
        (msgs/f'{name}.msg').write_text('uint64 timestamp # comment\n')
        (tmp_path/'msg'/f'{name}.msg').write_text('uint64 timestamp\n')
    d=tmp_path/'src/modules/uxrce_dds_client';d.mkdir(parents=True)
    p=d/'dds_topics.yaml';original='publications: []\nsubscriptions: []\n';p.write_text(original)
    assert len(module.prepare(tmp_path,msgs,True))==8
    assert p.read_text()==original
    assert len(module.prepare(tmp_path,msgs))==8
    assert module.prepare(tmp_path,msgs)==[]
    (msgs/'VehicleStatus.msg').write_text('uint8 changed\n')
    with pytest.raises(ValueError,match='ABI mismatch'):module.prepare(tmp_path,msgs)
