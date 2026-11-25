import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
pytest.importorskip('pymavlink')
from pymavlink.dialects.v20 import common as mav
from zav_real_control.single_link import SingleLink
from zav_real_control.config import load_config
from zav_real_control.mavlink_parameters import encode

ROOT=Path(__file__).resolve().parents[1]
EPOCH=1_800_000_000_000_000


class Connection:
    def __init__(self):self.messages=[];self.calls=[];self.mav=self
    def recv_match(self,blocking=False):return self.messages.pop(0) if self.messages else None
    def __getattr__(self,name):return lambda *a,**kw:self.calls.append((name,a,kw))


def msg(kind,source=(1,1),**fields):
    return SimpleNamespace(get_type=lambda:kind,get_srcSystem=lambda:source[0],
                           get_srcComponent=lambda:source[1],**fields)


def make(live=True):
    c=load_config(ROOT/'config/hardware.yaml')
    c['expected_firmware']=[1,14,3]  # Protocol fixtures below explicitly emulate this version.
    c['firmware_verified']=True;c['mavlink']['rc_mode_channel']=7
    connection=Connection();return SingleLink(connection,c,live),connection


def receive(s,t,kind,**kwargs):s.receive(msg(kind,**kwargs),t,EPOCH*1000+int(t*1e9))


def heartbeat(s,t=0,armed=True,mode=(3,0)):
    receive(s,t,'HEARTBEAT',autopilot=mav.MAV_AUTOPILOT_PX4,
            base_mode=128 if armed else 0,custom_mode=(mode[0]<<16)|(mode[1]<<24))
    s.parameters.poll(t)


def prepared(s,t=0,armed=True):
    heartbeat(s,t,armed)
    receive(s,t,'AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(14<<16)|(3<<8))
    receive(s,t,'PARAM_VALUE',param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(0))
    s.parameters.poll(t)
    receive(s,t,'NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1000+int(t*1000),value=1.)
    receive(s,t,'NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1000+int(t*1000),value=0.)
    receive(s,t,'RC_CHANNELS',chan7_raw=1000)


def odom(s,t,x=0,q=(1,0,0,0),v=(0,0,0)):
    stamp=EPOCH+int(t*1e6)
    return s.odometry(stamp,(x,0,0),q,v,(0,0,0),t,stamp,('camera_init','mid360_imu'))


def sent(c,name):return [a for kind,a,kw in c.calls if kind==name]


def sync_requests(c):
    return [a for a in sent(c,'command_long_send')
            if a[2]==mav.MAV_CMD_SET_MESSAGE_INTERVAL and a[4]==mav.MAVLINK_MSG_ID_TIMESYNC]


def test_timesync_fast_start_and_immediate_slowdown_only_after_fc_confirmation():
    s,c=make(live=False);heartbeat(s,armed=False)
    s.update(0,EPOCH*1000)
    assert [a[5] for a in sync_requests(c)]==[10000]
    # Five seconds passing never fakes convergence or permits EV output.
    heartbeat(s,5.,armed=False)
    s.update(5.,EPOCH*1000+5_000_000_000)
    assert not s.clock_good(5.) and not s.snapshot(5.)['ready']
    receive(s,5.1,'NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=5100,value=1.)
    s.update(5.1,EPOCH*1000+5_100_000_000)
    assert [a[5] for a in sync_requests(c)]==[10000,100000]
    # The periodic telemetry queue must never put an old rate back afterward.
    for i in range(1,15):s.update(5.1+i*.01,EPOCH*1000+int((5.1+i*.01)*1e9))
    assert [a[5] for a in sync_requests(c)]==[10000,100000]
    assert not sent(c,'odometry_send') and not sent(c,'param_set_send')
    assert not [a for a in sent(c,'command_long_send') if a[2] in (400,176,21)]


@pytest.mark.parametrize('lost',[False,True])
def test_timesync_resumes_fast_rate_on_stale_or_negative_fc_confirmation(lost):
    s,c=make(live=False);prepared(s,armed=False)
    s.update(0,EPOCH*1000)
    assert [a[5] for a in sync_requests(c)]==[100000]
    now=.1 if lost else 2.01
    if lost:receive(s,now,'NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1100,value=0.)
    s.update(now,EPOCH*1000+int(now*1e9))
    assert not s.clock_good(now)
    assert [a[5] for a in sync_requests(c)]==[100000,10000]


def test_timesync_rate_reassertion_does_not_burst_and_requires_heartbeat():
    s,c=make(live=False)
    s.update(0,EPOCH*1000)
    assert not sync_requests(c)
    heartbeat(s,armed=False);s.update(.01,EPOCH*1000+10_000_000)
    for t in (.02,.03,.04):s.update(t,EPOCH*1000+int(t*1e9))
    assert [a[5] for a in sync_requests(c)]==[10000]
    heartbeat(s,10.1,armed=False);s.update(10.1,EPOCH*1000+10_100_000_000)
    assert [a[5] for a in sync_requests(c)]==[10000,10000]


@pytest.mark.parametrize('super_source',[False,True])
def test_hover_output_readback_is_requested_only_for_super(super_source):
    from zav_real_control.super_gate import SuperRateGate
    s,c=make();prepared(s)
    s.offboard.enabled=True
    if super_source:s.offboard=SuperRateGate(True)
    for i in range(12):s.update(i*.01,(EPOCH+int(i*.01*1e6))*1000)
    requests=sent(c,'command_long_send')
    target=[a for a in requests if a[2]==mav.MAV_CMD_SET_MESSAGE_INTERVAL and a[4]==mav.MAVLINK_MSG_ID_ATTITUDE_TARGET]
    assert bool(target)==super_source
    if target:assert target[0][5]==50000


def test_super_hover_calibration_cannot_use_an_echo_from_the_companion():
    from zav_real_control.super_gate import SuperRateGate
    s,c=make();s.offboard=SuperRateGate(True)
    s.receive(msg('ATTITUDE_TARGET',source=(255,191),thrust=.6,time_boot_ms=1234),0,EPOCH*1000)
    assert s.offboard.attitude_target is None


def test_serial_write_failure_is_not_counted_as_ev_sent():
    s,c=make();prepared(s)
    def broken_write(*args, **kwargs):
        raise OSError('serial short write')
    c.odometry_send=broken_write
    with pytest.raises(OSError,match='serial short write'):
        odom(s,.01)
    assert s.counts['out']==0
    s.fail_io(OSError('serial short write'))
    status=s.snapshot(.02)
    assert status['error']=='serial short write'
    assert not status['checks']['serial_io']


def test_one_connection_emits_mavlink_odometry_with_measurement_epoch():
    s,c=make();prepared(s)
    assert odom(s,.01)
    out=sent(c,'odometry_send')[0]
    assert out[0]==EPOCH+10000
    assert out[1:3]==(mav.MAV_FRAME_LOCAL_FRD,mav.MAV_FRAME_BODY_FRD)
    a=math.radians(22)
    assert out[3:6]==pytest.approx((-.03888*math.cos(a)+.15182*math.sin(a),
                                           .02329,.03888*math.sin(a)+.15182*math.cos(a)))
    assert out[13][0]==.04 and out[13][15]==.03
    assert out[14][0]==.04 and math.isnan(out[14][20])
    assert c.calls[-1][2]['estimator_type']==mav.MAV_ESTIMATOR_TYPE_UNKNOWN


def test_actual_wire_is_mavlink2_and_body_velocity_matches_quaternion():
    s,c=make();prepared(s)
    q=(math.sqrt(.5),0,0,math.sqrt(.5))
    assert odom(s,.01,q=q,v=(1,0,0))
    _,a,kw=c.calls[-1]
    packet=mav.MAVLink_odometry_message(*a,**kw).pack(mav.MAVLink(None))
    assert packet[0]==0xfd
    decoded=mav.MAVLink(None).parse_buffer(packet)[0]
    assert decoded.get_type()=='ODOMETRY'
    assert decoded.time_usec==EPOCH+10000
    assert decoded.child_frame_id==mav.MAV_FRAME_BODY_FRD


def test_monitor_and_wrong_version_never_write_ev_parameters_or_land():
    for live,wrong_version in [(False,False),(True,True)]:
        s,c=make(live);prepared(s)
        if wrong_version:s.parameters.version=(1,15,0)
        odom(s,.01);odom(s,.02,x=2)
        s.update(4,EPOCH*1000+4_000_000_000)
        assert not sent(c,'odometry_send') and not sent(c,'param_set_send')
        assert not [a for a in sent(c,'command_long_send') if a[2]==mav.MAV_CMD_NAV_LAND]


def test_jump_cuts_ev_enables_baro_and_waits_for_real_fusion():
    s,c=make();prepared(s)
    assert odom(s,.01)
    assert not odom(s,.02,x=2) and s.motion.latched
    s.update(.03,EPOCH*1000+30_000_000)
    assert len(sent(c,'odometry_send'))==1
    assert sent(c,'param_set_send')[-1][3]==encode(1)
    assert not [a for a in sent(c,'command_long_send') if a[2]==21]
    receive(s,.04,'PARAM_VALUE',param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(1))
    s.parameters.poll(.04)
    receive(s,.04,'LOCAL_POSITION_NED',z=-1.,vz=0.)
    receive(s,.04,'ESTIMATOR_STATUS',flags=mav.ESTIMATOR_VELOCITY_VERT | mav.ESTIMATOR_POS_VERT_ABS)
    s.land_tick(.04)
    assert not s.handoff.confirmed
    receive(s,.05,'NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1100,value=1.)
    s.land_tick(.05);assert not s.handoff.confirmed
    receive(s,.06,'NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1200,value=3.)
    s.land_tick(.06)
    assert s.handoff.confirmed
    lands=[a for a in sent(c,'command_long_send') if a[2]==21]
    assert len(lands)==1 and lands[0][:2]==(1,1)
    assert not odom(s,.07,x=0)
    assert len(sent(c,'odometry_send'))==1
    assert not [a for a in sent(c,'command_long_send') if a[2] in (400,176)]  # no arm or set-mode


def test_timeout_retries_land_without_claiming_baro_success():
    s,c=make();prepared(s);odom(s,.01);odom(s,.02,x=2)
    heartbeat(s,3.2)
    s.land_tick(3.2)
    assert s.handoff.timed_out and not s.handoff.confirmed
    assert len([a for a in sent(c,'command_long_send') if a[2]==21])==1


def test_takeover_requires_both_rc_switch_and_manual_mode():
    s,c=make();prepared(s);odom(s,.01);odom(s,.02,x=2)
    heartbeat(s,.03,mode=(2,0))  # native fallback alone must not count as pilot action
    s.land_tick(.03);assert not s.handoff.pilot_takeover
    receive(s,.04,'RC_CHANNELS',chan7_raw=1800)
    s.land_tick(.04);assert s.handoff.pilot_takeover
    heartbeat(s,4,mode=(2,0));s.land_tick(4)
    assert not [a for a in sent(c,'command_long_send') if a[2]==21]


def test_wrong_source_and_duplicate_diagnostics_cannot_unlock_clock():
    s,c=make()
    receive(s,0,'NAMED_VALUE_FLOAT',source=(2,1),name='zav_sync',time_boot_ms=1000,value=1.)
    assert not s.clock_good(0)
    receive(s,0,'NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1000,value=0.)
    receive(s,.1,'NAMED_VALUE_FLOAT',name='zav_sync',time_boot_ms=1000,value=1.)
    assert not s.clock_good(.1)


def test_epoch_timesync_response_uses_same_clock_as_odometry():
    s,c=make();receive(s,.2,'TIMESYNC',tc1=0,ts1=123456000)
    assert sent(c,'timesync_send')==[(EPOCH*1000+200000000,123456000)]


def test_reboot_and_clock_step_require_ground_restart():
    s,c=make();prepared(s)
    receive(s,0,'SYSTEM_TIME',time_boot_ms=10000)
    receive(s,.1,'SYSTEM_TIME',time_boot_ms=100)
    assert s.reboot_detected and not s.writable()
    s,c=make();prepared(s)
    s.update(0,EPOCH*1000);s.update(.1,EPOCH*1000+5_000_000_000)
    assert s.clock_step_detected and not s.writable()


def test_absent_rc_channel_blocks_live_ev():
    s,c=make();prepared(s);s.c['mavlink']['rc_mode_channel']=0
    assert not odom(s,.01)
    assert not s.snapshot(.01)['checks']['rc_takeover']


def test_single_link_config_does_not_require_agent_or_px4_schema_flags():
    c=load_config(ROOT/'config/hardware.yaml')
    assert c['transport']=='mavlink' and 'agent' not in c and 'firmware_abi_verified' not in c


def test_117_version_match_does_not_bypass_required_diagnostics():
    s,c=make()
    s.parameters.expected_version=(1,17,0)
    heartbeat(s,0,armed=False)
    receive(s,0,'AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(17<<16))
    receive(s,0,'PARAM_VALUE',param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(0))
    s.parameters.poll(0)
    assert s.parameters.supported
    assert not odom(s,.01)
    snapshot=s.snapshot(.01)
    assert not snapshot['ready']
    assert 'clock' in snapshot['blocked_by']
    assert 'baro_off_confirmed' in snapshot['blocked_by']
    assert not sent(c,'odometry_send')
    receive(s,.02,'AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(17<<16)|(1<<8))
    s.parameters.poll(.02)
    assert not s.parameters.supported


def test_firmware_diagnostic_patch_is_atomic_idempotent_and_primary_aware(tmp_path):
    spec=importlib.util.spec_from_file_location('prepare_mavlink',ROOT/'scripts/prepare_mavlink.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    baseline=ROOT.parents[2]/'src/PX4-Autopilot'
    if not baseline.exists():pytest.skip('Optional host PX4 source fixture is unavailable')
    # Workspace-relative fixture: only read the real source; changes stay in tmp_path.
    for relative in ('src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp','src/modules/mavlink/mavlink_timesync.cpp'):
        p=tmp_path/relative;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text((baseline/relative).read_text())
    assert len(module.prepare(tmp_path,True))==2
    assert len(module.prepare(tmp_path))==2
    assert module.prepare(tmp_path)==[]
    stream=(tmp_path/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp').read_text()
    assert 'ChangeInstance(_estimator_status_sub.get_instance())' in stream
    assert 'innovation_rejected' in stream and 'timestamp_sample' in stream
    assert '_timesync.sync_converged()' in (tmp_path/'src/modules/mavlink/mavlink_timesync.cpp').read_text()


def test_real_pymavlink_usb_ttl_transport_on_virtual_uart(monkeypatch):
    """Use a temporary pseudo-terminal, not a physical UART, FC, network or simulator."""
    import os
    import select
    import time
    pytest.importorskip('serial')
    from pymavlink import mavutil
    monkeypatch.setenv('MAVLINK20','1');mavutil.set_dialect('common')
    master,slave=os.openpty()
    connection=None
    try:
        connection=mavutil.mavlink_connection(os.ttyname(slave),baud=921600,dialect='common',
                                             source_system=255,source_component=191)
        class Writer:
            def write(self,data):return os.write(master,data)
        fc=mav.MAVLink(Writer(),srcSystem=1,srcComponent=1)
        template,_=make();s=SingleLink(connection,template.c,live=True)
        fc.heartbeat_send(mav.MAV_TYPE_QUADROTOR,mav.MAV_AUTOPILOT_PX4,128,3<<16,mav.MAV_STATE_ACTIVE)
        # AUTOPILOT_VERSION constructor is long; construct fields using the dialect's declared schema.
        fields=dict(capabilities=0,flight_sw_version=(1<<24)|(14<<16)|(3<<8),middleware_sw_version=0,
                    os_sw_version=0,board_version=0,flight_custom_version=[0]*8,
                    middleware_custom_version=[0]*8,os_custom_version=[0]*8,vendor_id=0,product_id=0,uid=1)
        fc.send(mav.MAVLink_autopilot_version_message(**fields))
        fc.named_value_float_send(1000,b'zav_sync',1.)
        fc.named_value_float_send(1000,b'zav_baro',0.)
        channel_fields={f'chan{i}_raw':1000 if i==7 else 1500 for i in range(1,19)}
        fc.send(mav.MAVLink_rc_channels_message(time_boot_ms=1000,chancount=18,rssi=200,**channel_fields))
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            now=time.monotonic()
            s.update(now,EPOCH*1000+int(now*1e9))
            if s.parameters.supported and s.clock_good(now) and s.rc_value is not None:break
        assert s.parameters.supported and s.clock_good(now) and s.rc_value==1000
        assert odom(s,now+.001)
        parser=mav.MAVLink(None);packets=[]
        while select.select([master],[],[],.05)[0]:
            packets.extend(parser.parse_buffer(os.read(master,65536)) or [])
        outputs=[p for p in packets if p.get_type()=='ODOMETRY']
        assert len(outputs)==1 and outputs[0].get_srcComponent()==191
        assert outputs[0].estimator_type==mav.MAV_ESTIMATOR_TYPE_UNKNOWN
        assert outputs[0].get_msgbuf()[0]==0xfd
    finally:
        if connection:connection.close()
        os.close(master);os.close(slave)


def test_rc_channel_auto_reads_fc_mapping_for_swb():
    s,c=make();prepared(s);s.c['mavlink']['rc_mode_channel']=0
    assert not odom(s,.01)
    receive(s,.02,'PARAM_VALUE',param_type=6,param_id='RC_MAP_FLTMODE',param_value=encode(7))
    receive(s,.02,'RC_CHANNELS',chan7_raw=1000)
    assert s.mode_channel(.02)==7
    assert odom(s,.03)
    receive(s,.04,'PARAM_VALUE',param_type=6,param_id='RC_MAP_FLTMODE',param_value=encode(8))
    assert s.rc_mapping_changed and not s.writable()


@pytest.mark.parametrize('jump', ['position', 'attitude'])
def test_117_virtual_jump_full_handoff(jump):
    """Run the hardware-configured bridge with an in-memory FC, never a serial port."""
    s,c=make();prepared(s)
    s.parameters.expected_version=(1,17,0)
    receive(s,0,'AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(17<<16))
    s.parameters.poll(0)
    assert s.parameters.supported and odom(s,.01)
    kwargs={'x':2} if jump=='position' else {'q':(math.sqrt(.5),0,0,math.sqrt(.5))}
    assert not odom(s,.02,**kwargs)
    assert s.motion.latch_reasons==[jump+'_residual']
    s.update(.03,EPOCH*1000+30_000_000)
    assert sent(c,'param_set_send')[-1][3]==encode(1)
    assert not [a for a in sent(c,'command_long_send') if a[2]==21]
    receive(s,.04,'PARAM_VALUE',param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(1))
    s.parameters.poll(.04)
    receive(s,.04,'LOCAL_POSITION_NED',z=-1.,vz=0.)
    receive(s,.04,'ESTIMATOR_STATUS',flags=mav.ESTIMATOR_VELOCITY_VERT|mav.ESTIMATOR_POS_VERT_ABS)
    receive(s,.04,'NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1100,value=3.)
    s.land_tick(.04)
    assert s.handoff.confirmed
    assert len([a for a in sent(c,'command_long_send') if a[2]==21])==1
    assert not odom(s,.07)
    assert len(sent(c,'odometry_send'))==1
    assert not [a for a in sent(c,'command_long_send') if a[2] in (400,176)]


@pytest.mark.parametrize('condition', ['missing', 'stale', 'resumed'])
def test_117_ev_input_loss_uses_jump_handoff(condition):
    s,c=make();prepared(s)
    s.parameters.expected_version=(1,17,0)
    receive(s,0,'AUTOPILOT_VERSION',flight_sw_version=(1<<24)|(17<<16))
    s.parameters.poll(0)
    assert odom(s,.01)
    heartbeat(s,.3)
    if condition=='stale':
        assert not s.odometry(EPOCH+20000,(0,0,0),(1,0,0,0),(0,0,0),(0,0,0),
                              .3,EPOCH+300000,('camera_init','mid360_imu'))
    elif condition=='resumed':
        assert not odom(s,.3)
    s.update(.3,EPOCH*1000+300_000_000)
    assert s.motion.latched and s.handoff.started is not None
    assert sent(c,'param_set_send')[-1][3]==encode(1)
    assert not [a for a in sent(c,'command_long_send') if a[2]==21]
    receive(s,.31,'PARAM_VALUE',param_type=6,param_id='EKF2_BARO_CTRL',param_value=encode(1))
    s.parameters.poll(.31)
    receive(s,.31,'LOCAL_POSITION_NED',z=-1.,vz=0.)
    receive(s,.31,'ESTIMATOR_STATUS',flags=mav.ESTIMATOR_VELOCITY_VERT|mav.ESTIMATOR_POS_VERT_ABS)
    receive(s,.31,'NAMED_VALUE_FLOAT',name='zav_baro',time_boot_ms=1310,value=3.)
    s.land_tick(.31)
    assert s.handoff.confirmed
    assert len([a for a in sent(c,'command_long_send') if a[2]==21])==1
    assert not odom(s,.32)  # recovery cannot restart EV after latching
    assert len(sent(c,'odometry_send'))==1


@pytest.mark.parametrize('armed,live', [(False,True),(True,False)])
def test_input_loss_does_not_handoff_disarmed_or_readonly(armed,live):
    s,c=make(live);prepared(s,armed=armed)
    odom(s,.01)
    heartbeat(s,.3,armed=armed)
    s.update(.3,EPOCH*1000+300_000_000)
    assert not s.motion.latched and s.handoff.started is None
    assert not sent(c,'param_set_send')
    assert not [a for a in sent(c,'command_long_send') if a[2]==21]


def test_old_measurements_cannot_hide_behind_fresh_arrivals():
    s,c=make();prepared(s)
    assert odom(s,.01)
    # Receipt is fresh, but the last accepted measurement reaches the age limit first.
    assert s.odometry(EPOCH+20000,(0,0,0),(1,0,0,0),(0,0,0),(0,0,0),
                      .20,EPOCH+200000,('camera_init','mid360_imu'))
    s.update(.28,EPOCH*1000+280_000_000)
    assert s.motion.latch_reasons==['ev_measurement_stale']


def test_input_watchdog_allows_short_gap_and_requires_prior_ev():
    s,c=make();prepared(s)
    assert odom(s,.01)
    s.update(.2,EPOCH*1000+200_000_000)
    assert not s.motion.latched
    s,c=make();prepared(s)
    s.update(.4,EPOCH*1000+400_000_000)
    assert not s.motion.latched
