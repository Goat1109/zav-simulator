from types import SimpleNamespace
from zav_real_control.telemetry import FlightTelemetry
from zav_real_control.status_view import render


def message(kind, **kwargs):
    return SimpleNamespace(get_type=lambda: kind, **kwargs)


def test_battery_total_unknown_and_stale():
    t = FlightTelemetry()
    assert not t.snapshot(0)['battery_fresh']
    t.receive(message('BATTERY_STATUS', id=0, voltages=[19900]+[65535]*9,
                      battery_remaining=-1, charge_state=4), 1)
    s = t.snapshot(1)
    assert s['battery_fresh']
    assert s['battery']['voltage_v'] == 19.9
    assert s['battery']['remaining_pct'] is None
    assert not t.snapshot(5)['battery_fresh']


def test_odometry_age_not_receive_interval_and_silence():
    t = FlightTelemetry()
    t.odometry(0, 1000000, 1200000)
    t.odometry(.01, 1010000, 1210000)
    s = t.snapshot(.01)['odometry']
    assert round(s['hz']) == 100
    assert s['age_median_ms'] == 200
    assert round(t.snapshot(3)['odometry']['last_measurement_age_ms']) == 3190
    assert t.snapshot(3)['odometry']['hz'] == 0


def test_land_not_assumed_battery_cause_and_no_stale_arm_claim():
    data = dict(px4_mode=[4, 6], armed=True, checks={'heartbeat': True},
                phase='NORMAL_EV_ONLY', telemetry={'fc_system_status': 5,
                'battery_fresh': True, 'battery': {'voltage_v': 19.9, 'charge_state': 4}})
    text = render(data)
    assert '착륙 의심' in text and '정확한 원인 미확인' in text
    data['telemetry']['battery_fresh'] = False
    assert '착륙 의심' not in render(data)
    data['checks']['heartbeat'] = False
    assert '시동 상태 미확인' in render(data)
    assert '시동 ON' not in render(data)


def test_fc_text_controls_removed_and_recent_history():
    t = FlightTelemetry()
    for i in range(8):
        t.receive(message('STATUSTEXT', severity=4, text=f'Battery low {i}\x1b\n'), i)
    texts = t.snapshot(10)['messages']
    assert len(texts) == 5
    assert texts[-1] == {'text': 'Battery low 7', 'age_s': 3}


def test_offboard_render_live_details_and_stopped_reason():
    from zav_real_control.status_view import offboard_lines
    data=dict(enabled=True,pattern='square',phase='SEGMENT',stopped=False,
        stage=3,stages=8,leg=2,legs=4,segment_kind='move',reference_progress_pct=30.,elapsed_s=12.,
        position_relative_front_right_up_m=[5.,1.,0.],goal_relative_front_right_up_m=[5.,5.,0.],
        goal_distance_m=4.,tracking_error_m=.1,goal_yaw_error_deg=2.,horizontal_speed_m_s=2.,
        options={'speed_m_s':5.,'yaw_rate_deg_s':60.},rc_pwm=2000,request_age_s=.02,
        local_age_s=.1,setpoint_age_s=.04,sent=200)
    text='\n'.join(offboard_lines(data))
    for word in ('waypoint_square','변 2/4','동작 3/8','직선 이동','4.00 m','실제 수평 2.00 m/s','60°/s'):
        assert word in text
    data.update(stopped=True,phase='tracking_error')
    assert '전송 중단 / 목표 추종 오차 초과' in '\n'.join(offboard_lines(data))
    assert '기체 정지' not in '\n'.join(offboard_lines(data))


def test_offboard_render_wait_and_complete():
    from zav_real_control.status_view import offboard_lines
    for phase,expected in [('READY_FOR_RC8_HIGH','준비 완료'),('COMPLETE_HOLD','마지막 위치 유지'),('test_node_lost','시험 노드 입력 끊김')]:
        assert expected in '\n'.join(offboard_lines(dict(enabled=True,phase=phase)))
    assert '비활성' in '\n'.join(offboard_lines(dict(enabled=False)))


def test_hover_ready_cancel_rejection_and_frozen_fault_are_distinct():
    from zav_real_control.status_view import offboard_lines
    data=dict(enabled=True,phase='OFFBOARD_HOVER_READY',hover_ready=True,
              command_state='cancelled',command_reason='test_node_lost',pattern='front',
              last_rejection={'pattern':'right','reason':'command_requires_offboard_hover'})
    text='\n'.join(offboard_lines(data))
    for word in ('시험 명령 입력 가능','취소','시험 노드 입력 끊김','명령 거절','새로 실행'):
        assert word in text
    data.update(phase='tracking_error',stopped=True,command_state='failed',command_reason='tracking_error',
                fault={'fc_mode':[6,0],'rc_pwm':2003,'tracking_error_m':1.01})
    text='\n'.join(offboard_lines(data))
    assert '실패 발생 시점  FC Offboard | RC8 2003 us | 기준점 오차 1.01 m (고정 기록)' in text


def test_paused_reference_is_visible_while_running_only():
    from zav_real_control.status_view import offboard_lines
    data=dict(enabled=True,phase='SEGMENT',reference_paused=True,command_state='running')
    assert '기체 추종을 기다리는 중' in '\n'.join(offboard_lines(data))
    data['command_state']='failed'
    assert '기체 추종을 기다리는 중' not in '\n'.join(offboard_lines(data))


def test_super_status_shows_controller_and_does_not_claim_position_cap():
    from zav_real_control.status_view import offboard_lines
    data=dict(enabled=True,source='super',phase='SUPER_TRACKING',stopped=False,
        controller={'phase':'TRACKING','solver_ms':4.2,'position':[2.,0.,2.],'goal':[7.,0.,2.]},
        horizontal_speed_m_s=3.7,thrust=.22,hover_thrust=.18,rc_pwm=2000,
        controller_age_s=.02,local_age_s=.03,setpoint_age_s=.01,sent=100)
    text='\n'.join(offboard_lines(data))
    for word in ('SUPER / OMMPC','SUPER 경로 추종','4.20 ms','5.00 m','0.220','실제 수평 3.70 m/s'):
        assert word in text
    assert '기준 상한' not in text and 'waypoint' not in text
    data.update(stopped=True,phase='SUPER_PARAMETER_READBACK',parameter_issue='MPC_THR_HOVER')
    assert '전송 중단' in '\n'.join(offboard_lines(data))
    assert 'PX4 확인  MPC_THR_HOVER' in '\n'.join(offboard_lines(data))


def test_rc7_freshness_invalid_pwm_and_recovery():
    t=FlightTelemetry()
    assert not t.snapshot(0)['rc7']['fresh']
    t.receive(message('RC_CHANNELS',chan7_raw=1500),1.)
    assert t.snapshot(1.)['rc7']=={'pwm':1500,'age_s':0.,'fresh':True}
    assert not t.snapshot(1.6)['rc7']['fresh']
    t.receive(message('RC_CHANNELS',chan7_raw=65535),2.)
    assert not t.snapshot(2.)['rc7']['fresh']
    t.receive(message('RC_CHANNELS',chan7_raw=1000),3.)
    assert t.snapshot(3.)['rc7']['fresh']
