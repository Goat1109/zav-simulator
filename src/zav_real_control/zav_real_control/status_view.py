"""Compact display of reported facts; battery warning is not an exact landing cause."""
MODE = {(1, 0): 'Manual', (2, 0): 'Altitude', (3, 0): 'Position',
        (4, 6): 'Land', (4, 5): 'Return', (5, 0): 'Acro',
        (6, 0): 'Offboard', (7, 0): 'Stabilized'}
BATTERY = {0: '미확인', 1: '정상', 2: '낮음', 3: '위험', 4: '긴급',
           5: '고장', 6: '비정상', 7: '충전 중'}
REASONS = {'ev_input_timeout': 'EV 입력 끊김', 'ev_measurement_stale': 'EV 측정값 지연',
           'position_residual': '위치 급변', 'attitude_residual': '자세 급변',
           'source_measurement_age': '측정 시각 지연/미래 시각',
           'recent_odometry': '유효 odometry 없음', 'clock': 'FC 시각 동기화',
           'heartbeat': 'FC 연결', 'baro_off_confirmed': '기압계 OFF 미확인',
           'rc_takeover': 'RC 모드 채널 확인', 'runtime_firmware': 'FC 펌웨어 확인'}
OFFBOARD_TESTS = {'front':'go_front / 전진', 'right':'go_right / 우측',
                 'back':'go_back / 후진', 'left':'go_left / 좌측',
                 'square':'waypoint_square / 사각형', 'circle':'waypoint_circle / 원'}
OFFBOARD_PHASES = {
    'disabled':'기능 비활성', 'waiting_for_test':'시험 노드 대기',
    'waiting_for_armed_airborne_healthy_position':'무장·공중·정상 EV/FC 피드백 대기',
    'waiting_for_stationary_position_hover':'Position 정지 호버링 대기',
    'waiting_for_position_mode_and_RC8_low':'Position + RC8 LOW 대기',
    'PRESTREAM_HOLD_RC8_LOW':'현재 위치 유지 목표 준비 중',
    'READY_FOR_RC8_HIGH':'준비 완료 — RC8 전환 대기',
    'WAITING_FOR_FC_OFFBOARD':'RC8 HIGH — 실제 FC Offboard 전환 확인 중',
    'OFFBOARD_HOVER_SETTLING':'Offboard 위치 유지 — 정지 확인 중',
    'OFFBOARD_HOVER_READY':'Offboard 호버링 준비 완료 — 시험 명령 입력 가능',
    'command_requires_offboard_hover':'Offboard 호버링 준비 후 시험 노드를 새로 실행하세요',
    'another_test_running':'다른 시험 진행 중',
    'bridge_fault_restart_required':'브리지 실패 고정 — 지상 재시작 필요',
    'RC8_low':'RC8 LOW — 경로 취소',
    'ENTRY_HOLD':'명령 시작 위치 유지', 'SEGMENT':'경로 진행', 'DWELL':'정지 대기',
    'COMPLETE_HOLD':'시험 완료 — 마지막 위치 유지',
    'pilot_mode_or_RC8_exit':'RC8/모드 이탈 — 조종권 회수',
    'test_node_lost':'시험 노드 입력 끊김', 'flight_or_EV_feedback_unhealthy':'기체/EV/FC 피드백 이상',
    'tracking_error':'목표 추종 오차 초과', 'segment_timeout':'구간 도착 시간 초과',
    'setpoint_schedule_gap':'목표 전송 주기 끊김', 'serial_io_error':'시리얼 오류',
    'fc_local_position_reset_or_jump':'FC 로컬 위치 리셋/급변',
    'RC8_high_but_FC_not_offboard':'RC8 HIGH지만 FC Offboard 미전환',
    'offboard_entered_before_prestream':'위치 유지 목표 준비 전 Offboard 진입',
    'multiple_test_publishers':'시험 노드 중복',
    'another_test_session_ground_restart_required':'다른 시험 세션 감지 — 지상 재시작 필요',
    'request_sequence_or_options_changed':'요청 순서/설정 변경', 'invalid_test_request':'유효하지 않은 시험 요청',
}


def number(value, suffix='', digits=0):
    return '--' if value is None else f'{value:.{digits}f}{suffix}'


def offboard_lines(data):
    if not data.get('enabled'):
        return ['Offboard  비활성 (브리지 offboard:=false)']
    if data.get('source')=='super':
        phase=data.get('phase','--')
        labels={'SUPER_READY_RC8':'준비 완료 — RC8 전환 대기',
                'SUPER_CALIBRATING_HOVER_THRUST':'Position 호버 추력 측정 중',
                'SUPER_HOVER_THRUST_NOT_READY':'호버 추력 미확인 — RC8 LOW에서 다시 준비',
                'SUPER_PRESTREAM':'OMMPC 준비 스트림', 'SUPER_HOVER':'호버링 — 목표 명령 대기',
                'SUPER_PLANNING':'회피 경로 계산', 'SUPER_TRACKING':'SUPER 경로 추종',
                'SUPER_ENDPOINT_HOLD':'경로 끝점 유지', 'SUPER_ENDPOINT_SETTLING':'끝점 감속·수렴 중', 'SUPER_PILOT_MODE':'RC 조종 모드',
                'SUPER_WAIT_POSITION_HOVER':'Position 정지 호버링 대기'}
        lines=["SUPER / OMMPC  "+('전송 중단 / ' if data.get('stopped') else '')+labels.get(phase,phase)]
        c=data.get('controller') or {}
        mission=c.get('mission') or {}
        if mission:
            name={'none':'대기','forward':'전방 이동','patrol':'Patrol','rth':'RTH','takeoff':'자동 이륙'}.get(mission.get('kind'),'--')
            progress=f" {mission.get('leg',0)}/{mission['legs']}" if mission.get('legs') else ''
            lines.append(f"임무  {name}{progress} | 이륙 기준점 {'확인' if mission.get('home_valid') else '미확인'}")
            if mission.get('home_valid'):lines.append('Home [world m]  '+', '.join(f'{v:.2f}' for v in mission.get('home',[])))
        automatic=data.get('automatic_takeoff') or {}
        if automatic.get('enabled'):lines.append('자동 이륙  '+automatic.get('stage','--'))
        lines.append(f"OMMPC  {c.get('phase','--')} | 연산 {number(c.get('solver_ms'),' ms',2)} | 입력 나이 {number(data.get('controller_age_s'),' s',2)}")
        if c.get('phase') in ('PLANNING','TRACKING','ENDPOINT_SETTLING','ENDPOINT_HOLD'):
            import math
            distance=math.dist(c['position'],c['goal'])
            lines.append(f"SUPER 목표  남은 직선거리 {number(distance,' m',2)} (회피 경로 잔여 길이와 다름)")
        lines.append(f"출력  실제 수평 {number(data.get('horizontal_speed_m_s'),' m/s',2)} | 추력 {number(data.get('thrust'),digits=3)} | 호버 기준 {number(data.get('hover_thrust'),digits=3)}")
        handoff=data.get('hover_handoff')
        if handoff:
            state='전환 기준 유지' if handoff.get('frozen') else '측정 완료' if handoff.get('ready') else '측정 대기/진행'
            source='사용자 검증 초기값' if handoff.get('source')=='explicit_verified_ground_takeoff_thrust' else f"Position 표본 {handoff.get('samples',0)}개"
            lines.append(f"호버 보정  {state} | {source} | FC 초기 파라미터 {number(data.get('parameter_hover_thrust'),digits=3)}")
        lines.append(f"SUPER 연결  RC8 {number(data.get('rc_pwm'),' us')} | FC 위치 나이 {number(data.get('local_age_s'),' s',2)} | 출력 나이 {number(data.get('setpoint_age_s'),' s',2)} | 누적 {data.get('sent',0)}개")
        if data.get('parameter_issue'):lines.append('PX4 확인  '+data['parameter_issue'])
        return lines
    phase=data.get('phase','--')
    state=OFFBOARD_PHASES.get(phase,phase)
    test=OFFBOARD_TESTS.get(data.get('pattern'),'시험 노드 대기')
    lines=[f"Offboard  {test} | {'전송 중단 / ' if data.get('stopped') else ''}{state}"]
    command=data.get('command_state')
    if command in ('complete','cancelled','failed'):
        label={'complete':'완료','cancelled':'취소','failed':'실패'}[command]
        reason=data.get('command_reason')
        lines.append(f"시험 결과  {label}" + (' / '+OFFBOARD_PHASES.get(reason,reason) if reason else ''))
    rejected=data.get('last_rejection')
    if rejected:
        lines.append('명령 거절  '+OFFBOARD_TESTS.get(rejected['pattern'],rejected['pattern'])+' / '+OFFBOARD_PHASES.get(rejected['reason'],rejected['reason']))
    fault=data.get('fault')
    if fault:
        mode=MODE.get(tuple(fault.get('fc_mode') or ()), '--')
        lines.append(f"실패 발생 시점  FC {mode} | RC8 {number(fault.get('rc_pwm'),' us')} | 기준점 오차 {number(fault.get('tracking_error_m'),' m',2)} (고정 기록)")
    if data.get('reference_paused') and command == 'running':
        lines.append('기준점 진행 제한  기체 추종을 기다리는 중')
    if data.get('stage') is not None:
        kind={'move':'직선 이동','turn':'제자리 회전','circle':'원주 이동'}.get(data.get('segment_kind'),'--')
        lines.append(f"시험 진행  변 {data.get('leg')}/{data.get('legs')} | 동작 {data['stage']}/{data.get('stages')} {kind} | 기준 진행 {number(data.get('reference_progress_pct'), '%', 1)} | 경과 {number(data.get('elapsed_s'), ' s', 1)}")
        if data.get('dwell_remaining_s') is not None:
            lines.append('정지 대기  '+number(data['dwell_remaining_s'],' s',1)+' 남음')
    actual=data.get('position_relative_front_right_up_m')
    goal=data.get('goal_relative_front_right_up_m')
    if actual is not None and goal is not None:
        fmt=lambda v:'/'.join(number(x,digits=2) for x in v)
        lines.append(f"상대 위치 앞/우/위[m]  실제 {fmt(actual)} | 구간 목표 {fmt(goal)}")
        lines.append(f"추적  구간 종점 거리 {number(data.get('goal_distance_m'),' m',2)} | 현재 기준점 오차 {number(data.get('tracking_error_m'),' m',2)} | 목표 yaw 차이 {number(data.get('goal_yaw_error_deg'),'°',1)}")
    options=data.get('options') or {}
    if options:
        lines.append(f"속도  실제 수평 {number(data.get('horizontal_speed_m_s'),' m/s',2)} | 기준 상한 {number(options.get('speed_m_s'),' m/s',1)} | yaw 기준 상한 {number(options.get('yaw_rate_deg_s'),'°/s',0)}")
    lines.append(f"Offboard 연결  RC8 {number(data.get('rc_pwm'),' us')} | 요청 나이 {number(data.get('request_age_s'),' s',2)} | FC 위치 나이 {number(data.get('local_age_s'),' s',2)} | 마지막 목표 전송 {number(data.get('setpoint_age_s'),' s 전',2)} | 누적 {data.get('sent',0)}개")
    return lines


def render(data):
    t = data.get('telemetry', {})
    connected = data.get('checks', {}).get('heartbeat', False)
    raw_mode = tuple(data.get('px4_mode') or ())
    mode = MODE.get(raw_mode, str(raw_mode)) if connected else '연결 끊김/대기'
    armed = ('시동 ON' if data.get('armed') else '시동 OFF') if connected else '시동 상태 미확인'
    lines = [f"기체  {mode} | {armed} | {data.get('mode', '--')}"]
    offboard = data.get('offboard', {})
    if offboard:
        lines.extend(offboard_lines(offboard))
    b = t.get('battery') or {}
    if t.get('battery_fresh'):
        lines.append(f"배터리  {number(b.get('voltage_v'), ' V', 2)} | 잔량 {number(b.get('remaining_pct'), '%')} (FC 추정) | {BATTERY.get(b.get('charge_state'), '미확인')}")
    else:
        lines.append('배터리  수신 대기/오래된 데이터 (현재 전압 미확인)')
    od = t.get('odometry', {})
    lines.append(f"LIO/VIO 출력  {number(od.get('hz'), ' Hz', 1)} | 지연 중앙/최대 {number(od.get('age_median_ms'))}/{number(od.get('age_max_ms'), ' ms')} (최근 2초)")
    lines.append(f"마지막 측정값 나이  {number(od.get('last_measurement_age_ms'), ' ms')} | 제한 {number(data.get('odom_age_limit_ms'), ' ms')}")
    reasons = [REASONS.get(r, r) for r in data.get('reasons', [])]
    active = connected and t.get('fc_system_status') in (5, 6)
    if reasons:
        fail = '브리지 감지: ' + ', '.join(reasons)
    elif active:
        fail = 'FC failsafe 활성 — 아래 FC 메시지 확인 (정확한 원인 미확인)'
    else:
        fail = 'FC failsafe 미보고' if connected else 'FC failsafe 상태 미확인'
    if connected and raw_mode == (4, 6) and t.get('battery_fresh') and b.get('charge_state') in (2, 3, 4):
        fail += ' | 저배터리 경고 + Land 동시 관측 (배터리 착륙 의심)'
    lines.append('대응  ' + fail)
    if data.get('phase') != 'NORMAL_EV_ONLY':
        lines.append(f"EV 대응  {data.get('phase', '--')} | 기압계 확인 {data.get('baro_confirmed')} | 타임아웃 {data.get('baro_timeout')}")
    blocked = data.get('blocked_by', [])
    lines.append('EV 전송 조건  ' + (', '.join(REASONS.get(r, r) for r in blocked) if blocked else '충족 (시동 허가/비행 준비 완료 판정 아님)'))
    if data.get('input_reason'):
        lines.append('입력  ' + REASONS.get(data['input_reason'], data['input_reason']))
    if data.get('error'):
        lines.append('통신 오류  ' + data['error'])
    for m in t.get('messages', [])[-2:]:
        lines.append(f"FC 메시지 ({m['age_s']:.0f}초 전)  {m['text']}")
    return '\n'.join(lines)
