# 실기체: USB-TTL 하나로 Radxa ↔ FC 연결

노트북/휴대폰에서 기존 별칭 작업·Bag·zcomm·state를 제어하는 로컬 웹 화면은 [웹 제어 안내](docs/web_control.md)를 따른다. argo 컨테이너에서 `zweb` 한 번으로 백그라운드 서버를 시작한다.

SUPER 로컬 회피 + OMMPC의 새 실기 실행 절차는 [SUPER 실기 안내](docs/super_hardware.md)를 참조한다. 이 경로는 명시적으로 `offboard_source:=super`를 선택하며 기존 RC/위치 목표 시험과 출력을 섞지 않는다.

EPIC 탐사 경로를 SUPER로 추종하는 RC7·웹 설정 선택·상태·중단 절차는 [EPIC 실기 안내](../zav_super/EPIC.md)를 참조한다.

**이 구성이 현재 기본이다. 별도 FC USB 통신 케이블은 필요하지 않다.**

```text
Mid360s ──Ethernet── Radxa Dragon Q6A
                         │ USB
                    USB-to-TTL 1개
                         │ TX / RX / GND
                    NXT PX4 v2 TELEM2

조종기 ──무선── ST-RX8S ── FC의 기존 RC 연결 유지
```

Radxa의 `mavlink_bridge` **한 프로세스**가 이 UART를 열어 odometry 송신, FC 상태 수신, 기압계 파라미터 변경·확인, LAND 명령을 모두 MAVLink 2로 처리한다. MAVROS와 DDS Agent는 필요하지 않으며 같은 포트를 함께 열면 안 된다. 기존 지상국용 TELEM1/915MHz 링크는 별도로 유지할 수 있다.

기본 경로는 조종기가 FC에 직접 입력하고 **Position 모드**로 조종한다. 자동 시동·이륙은 하지 않는다. 별도로 활성화한 Offboard 시험은 목표 또는 각속도·추력을 전송하므로 해당 시험 문서의 실행 절차를 따른다. 시뮬레이터와 flight manager는 실행하지 않는다.

## 1. 배선

- USB-to-TTL 어댑터 USB → Radxa USB.
- 어댑터 TX → FC TELEM2 RX, 어댑터 RX → FC TELEM2 TX, GND → GND.
- FC TELEM2와 호환되는 신호 전압을 사용한다. 포트 핀 번호/전압은 NXT 보드 문서를 확인한다. RS-232 어댑터와 TTL 어댑터는 다르다.
- 전원은 기존 FC/센서 전원 설계를 따른다. TTL 어댑터의 VCC를 임의로 연결하지 않는다.
- GPIO UART로 변경할 때는 Q6A pinmux·전압·핀 번호를 확인하고 YAML의 `mavlink.connection`, `baud`를 변경한다.

## 2. FC 준비 — 실제 펌웨어 버전 확인 필요

현재 지원 기준은 **PX4 v1.14.3**이다. NXT PX4 v2는 보드 이름이므로 QGC에서 실제 설치 버전과 보드 타깃을 확인한다. 다른 버전이라면 설정 숫자만 바꾸지 말고 코드·파라미터·진단 패치를 함께 검토해야 한다.

PX4 1.17.0용 소스 패치, 오프라인 테스트 및 `hkust_nxt-dual_default` 전체 펌웨어 빌드는 [1.17 검토 기록](docs/px4_1_17_review.md)에 정리했다. 업로드·실 FC 동작 검증은 아직 완료되지 않았으므로, 빌드 성공을 실기체 지원 완료로 해석하지 않는다.

FC 자체 EV 상실 감시를 추가한 v3의 파라미터·실패 대응·검증 범위는 [EV guard v3](docs/px4_ev_guard_v3.md)에 정리했다. v3는 별도 업로드가 필요하며 `ZAV_EV_GUARD` 기본값은 0이다.

FC의 일반 MAVLink 상태 메시지만으로는 **기압계가 실제로 융합됐는지** 확인할 수 없다. 기존 failsafe의 실제 융합 확인을 유지하려고, 같은 MAVLink UART에 작은 진단 메시지 두 개를 추가하는 도구를 제공한다. DDS, px4_msgs 메시지 ABI 변경, 두 번째 케이블은 필요 없다.

FC에 실제로 올릴 동일한 v1.14.3 펌웨어 소스에서:

```bash
python3 /path/to/real_ws/src/zav_real_control/scripts/prepare_mavlink.py \
  --px4-dir /path/to/actual/PX4-Autopilot
python3 /path/to/real_ws/src/zav_real_control/scripts/prepare_mavlink.py \
  --px4-dir /path/to/actual/PX4-Autopilot --verify-only
```

그 소스를 **NXT PX4 v2의 올바른 타깃**으로 빌드/업로드한다. 이 스크립트는 소스만 수정하며 빌드·플래시하지 않는다. 시뮬레이션 바이너리를 FC에 올리면 안 된다. 커스텀 펌웨어 빌드는 FC 빌드 환경에서 진행하면 되며 Radxa에서 할 필요는 없다.

진단 메시지는 표준 `NAMED_VALUE_FLOAT`를 사용한다:

- `zav_sync`: **FC 내부** MAVLink timesync filter의 실제 수렴 여부(0/1). SBC가 수렴했다고 가정하지 않는다.
- `zav_baro`: primary EKF의 신선한 baro aid 상태 비트. bit0=융합 enabled, bit1=해당 sample 정상 fused, bit2=innovation rejected. 정상 융합 확인은 값 3일 때만 한다. 파라미터 readback만으로 성공 처리하지 않는다.

이 진단 패치를 올리지 않으면 `clock`/`baro_off_confirmed`가 준비되지 않으며 EV 전송도 막힌다. 따라서 일반 펌웨어에 케이블만 연결한 상태를 비행 준비 완료라고 하지 않는다. 진단 패치의 실 FC 컴파일/동작은 아직 검증 전이다.

FC에서 [통신 파라미터](firmware/serial_links.params), [EKF 파라미터](firmware/external_vision.params)를 QGC로 검토·설정한다. 파일은 체크리스트이며 QGC import 형식이 아니다. 기본 TELEM2 예시는 `MAV_1_CONFIG=102`, `MAV_1_MODE=2`, `SER_TEL2_BAUD=921600`, `UXRCE_DDS_CFG=0`, MAVLink 2다. **실제 TELEM2를 담당하는 MAVLink 인스턴스를 확인**하고 포트 변경 후 FC를 재부팅한다. ST-RX8S와 TELEM1 설정을 임의로 변경하지 않는다.

## 3. Radxa 빌드

이미 시험한 ROS Humble, Livox-SDK2, 센서 의존성을 사용한다. 다른 PC의 x86 `build/install`을 복사하지 않는다.

```bash
cd ~/zenith_ws/real_ws
sudo apt update
sudo apt install python3-venv python3-pip python3-rosdep python3-colcon-common-extensions \
  cmake git build-essential libopencv-dev libeigen3-dev \
  ros-humble-ament-cmake-auto ros-humble-rmw-fastrtps-cpp
# rosdep 최초 설치라면 sudo rosdep init 을 한 번 수행
rosdep update
sudo usermod -aG dialout "$USER"
# 권한 적용은 로그아웃/로그인 후 확인
bash src/zav_real_control/scripts/build_radxa.sh
```

Agent 설치 스크립트는 실행하지 않는다. 빌드는 `.venv` Python을 사용하고 FAST-LIVO2/드라이버/control 의존성만 선택한다. px4_msgs는 기본 경로에 필요하지 않다. 메모리가 부족하면 `ZAV_BUILD_JOBS=1 bash .../build_radxa.sh`로 줄인다. Radxa 전체 빌드는 여기서 실행하지 않았다.

## 4. 설정 파일

```bash
ls -l /dev/serial/by-id/
nano src/zav_real_control/config/hardware.yaml
```

- `mavlink.connection`: 실제 **USB-TTL 한 개**의 장치 경로.
- `mavlink.baud`: FC TELEM2 baud와 일치. 기본 921600.
- `mavlink.target_system/component`: 실제 FC MAVLink ID와 일치. 기본 1/1.
- `mavlink.rc_mode_channel`: 조종기의 **비행 모드 스위치 채널**(1~18). 물리 스위치 이름은 사용자 확인값 `swB`로 기록했다. `swB`가 CH5/CH7 등 어느 RC 채널에 매핑됐는지는 별도로 확인한다. 기본값 0은 FC의 `RC_MAP_FLTMODE`를 같은 UART로 자동 조회한다. FC 설정이 0이거나 읽지 못하면 `respect_rc_takeover=true`인 live EV 전송을 막는다. 필요하면 QGC Flight Modes/Radio 화면에서 확인한 채널 번호를 직접 입력한다.
- `firmware_verified`: 실제 버전·진단 소스·업로드한 타깃을 확인한 뒤 true.
- `publish_enabled`: 기본 false. 실제 전송은 launch의 `publish:=true`로 명시한다.

장착값은 사용자 제공 body-FLU(앞·왼쪽·위) 기준이다. 위치는 기체/FC EKF 기준 원점에 대한 값이며, 회전은 Z-Y-X 순서다.

| 센서 | 위치 XYZ (m) | 회전 ZYX (degree) |
| --- | --- | --- |
| LiDAR | 0.01215, 0, 0.11503 | 0, 22, 0 |
| 카메라 기구 프레임 | 0.03215, 0, 0.03503 | 0, 0, 0 |
| Mid360s IMU | 0.03888, 0.02329, 0.15182 | 0, 22, 0 |

브리지는 IMU pose와 회전 중 lever-arm 속도를 기체 기준점으로 변환한다. `EKF2_EV_POS_X/Y/Z=0`으로 중복 보정을 피한다. 카메라 optical 보정과 기존 LiDAR–IMU 내부 보정은 유지했다. 기존 Livox 설정은 Radxa IP `192.168.1.50`, LiDAR `192.168.1.173`이므로 실제 Ethernet 설정과 맞춘다.

FAST-LIVO2는 opt-in `/zav/lio/odometry`를 추가했다. 기존 출력은 유지하며, 새 출력은 측정 시각과 child-IMU 속도를 사용한다. MAVLink 출력은 `LOCAL_FRD` pose + `BODY_FRD` twist이며 임의 heading을 진북 NED로 표시하지 않는다. epoch 측정 시각을 그대로 보내고 **PX4가 timesync offset을 한 번 적용**한다. PX4 v1.14.3은 `MAV_ESTIMATOR_TYPE_LIDAR` 메시지를 거부하므로 수신기가 허용하는 `UNKNOWN` 타입으로 외부 EV 경로에 전달한다. 센서 PTP/host receive timestamp fallback의 실제 지연은 별도 확인한다. `EKF2_EV_DELAY`와 잡음값은 실 ULog로 튜닝한다.

실제 MAVLink 전송 세션에서 무장 중 마지막 유효 odometry의 수신 또는 측정 시각이 `max_odom_age_s`(현재 0.25초)를 초과하면, 입력 상실을 위치·자세 급변과 같은 실패로 고정한다. 콜백이 완전히 끊겨도 주기적 점검으로 감지한다. 이후 EV 전송을 차단하고 기압계 활성화·기존 LAND 절차로 진행하며, 데이터가 돌아와도 지상에서 브리지를 재시작하기 전에는 EV를 재개하지 않는다. 비무장·읽기 전용 또는 아직 EV를 보내지 않은 세션에서는 이 대응을 시작하지 않는다. 이 감시는 FC 수신 확인이 아니라 브리지의 입력 감시이므로 UART 단절·브리지 프로세스 종료 시 FC 자체 failsafe는 별도로 필요하다.

## 5. 실행 — 터미널 3개

새 터미널마다:

```bash
cd ~/zenith_ws/real_ws
source src/zav_real_control/scripts/env.sh
```

**A: Mid360s + FAST-LIVO2**

```bash
ros2 launch zav_real_control sensors_lio.launch.py
```

기본은 카메라 없이 LiDAR+Mid360s IMU다. 이미 LiDAR driver를 실행했다면 `start_driver:=false`를 사용한다. `camera:=true`는 MVS 카메라 드라이버와 영상 융합을 함께 시작한다. 보정된 카메라 노드를 이미 별도 실행 중이라면 `camera:=true start_camera:=false`를 사용한다. 모든 센서와 FAST-LIVO2 터미널은 같은 `ROS_DOMAIN_ID`를 사용해야 한다.

이 launch는 Reliable publisher인 MVS 카메라에 맞춰 `common.img_reliable=true`를 사용한다. FAST-LIVO2를 다른 launch에서 직접 실행할 때 이 값의 기본은 false이며, Best Effort 전용 카메라에서는 false를 사용해야 한다.

**B: USB-TTL 브리지** — 처음에는 읽기 전용:

```bash
ros2 launch zav_real_control rc_flight.launch.py config:="$ZAV_REAL_CONFIG"
```

연결/버전/시각 점검 후 B를 종료하고 **프로펠러를 제거한 지상 시험**에서:

```bash
ros2 launch zav_real_control rc_flight.launch.py config:="$ZAV_REAL_CONFIG" publish:=true
```

**C: 상태 확인**

```bash
ros2 run zav_real_control inspect_status
ros2 topic hz /zav/lio/odometry
```

MONITOR_ONLY에서는 heartbeat, 상태 주기 요청, TIMESYNC 응답, 파라미터 읽기만 하며 EV·파라미터 쓰기·LAND는 보내지 않는다. EV가 없어서 PX4 위치 추정이 준비되지 않는 것은 이 모드에서는 예상된다. `clock`은 실제 FC timesync 수렴을 기다린다. 기본값은 초기 100Hz, FC의 `zav_sync=1` 확인 후 10Hz이며, 신선한 완료 진단이 사라지면 다시 100Hz로 요청한다. PX4의 유효 샘플 500개·RTT 10ms 미만 조건은 변경하지 않는다. 약 5초는 목표이며 시간만 지나서 준비 완료로 처리하지 않는다. `blocked_by`로 RC 채널·펌웨어·시각·baro 상태를 확인한다.

FTDI USB-TTL은 브리지 시작 시 실제 연결된 tty의 `latency_timer`를 `mavlink.ftdi_latency_timer_ms`(기본 1ms)로 설정하고 읽어 확인한다. USB를 다시 꽂아 tty 번호가 바뀌어도 by-id 경로로 다시 찾는다. FTDI에서 권한·읽기 확인이 실패하면 시작을 중단하며, argo 컨테이너 root 실행은 현재 검증된 경로다. 다른 UART 드라이버에는 이 설정을 쓰지 않는다. `mavlink.timesync_initial_rate_hz=100`, `timesync_steady_rate_hz=10`이 주기를 제어하며 FC 펌웨어 재업로드는 필요 없다.

같은 UART에 MAVROS, MicroXRCEAgent, 다른 pymavlink 노드를 동시에 실행하지 않는다. 상위 `run_fastlivo.sh`, `zav_sim.py`, flight manager도 실행하지 않는다. `ready=true`는 RC arming을 강제 차단하는 인터록은 아니므로 QGC EKF 상태도 확인한다.

## 6. 급변 failsafe

판단 기준은 기존 요청대로 **운동 연속성 하나**다.

- 위치 잔차: `|p_now-(p_prev+v_prev_world*dt)| > 0.5+0.3*dt+10*dt²` m.
- 자세 잔차: quaternion 최단 회전각 `> 20*pi/180+4*dt` rad.
- 최대 비교 간격 0.25초. armed 중 한 번 초과하면 latch한다.

순서: **EV 즉시 차단 → 같은 USB-TTL로 BARO_CTRL=1 → readback + 실제 baro 정상 융합 + 유효 수직 추정 확인 → LAND**. LAND 진입 전 1초마다 재시도한다. 3초 내 전환 확인 실패 시 `BARO_TIMEOUT`으로 best-effort LAND를 요청하며 기압계 융합 성공이라고 표시하지 않는다. ACK만으로 착륙 완료를 판단하지 않는다.

정상 비행에서 기압계 **센서를 끄는 것이 아니라 EKF 융합만 OFF**다. FC IMU는 계속 사용한다. 착륙 후 disarm/landed를 확인하고 원인을 해결한 뒤 지상에서 재시작한다. latch는 비행 중 자동 해제하지 않는다.

MAVLink heartbeat에는 DDS의 `nav_state_user_intention`이 없다. 조종사 인계는 **설정한 RC 모드 채널의 스위치 변화와 Manual/Altitude/Stabilized 계열 모드**를 함께 확인해 LAND 재시도를 멈춘다. 자동 fallback 모드 변화만으로는 조종사 인계로 보지 않는다. 모드 채널은 비행 전 반드시 확인한다. RC override 판정은 실제 TX/수신기/모드 설정에서 지상 검증해야 한다.

FC 재부팅, 비행 중 RC 모드 채널 매핑 변경 또는 host clock 급변 감지 시 전송을 막고 지상 재시작을 요구한다. 점진적 drift, 데이터 공백, LiDAR 보정 정지, EKF reject, Radxa/통신 상실은 motion-only LAND 트리거가 아니다. **Radxa나 유일한 UART가 죽으면 이 코드가 기압계 ON/LAND 명령을 보낼 수 없다.** FC 기본 failsafe를 유지하고, 이 경우까지 보호하려면 별도 FC watchdog 또는 상시 보조 고도 설계가 필요하다.

## 검증 범위

프로펠러 제거 상태에서 방향/부호/장착 보정/정상 baro OFF부터 확인한다. 그다음 SITL/HIL 등 모터가 격리된 환경에서 점프→EV 차단→baro 실제 융합→LAND 및 RC takeover를 재현한 뒤 저고도 정상 비행부터 검증한다. 여기서 실행한 테스트는 가짜 MAVLink/ROS 입력 및 실제 MAVLink 2 패킷 직렬화 검사이며, **실 FC 펌웨어 빌드·업로드·실비행은 수행하지 않았다.**

```bash
# real_ws 환경을 source한 뒤 실행. pytest가 없으면 python3-pytest 설치.
ROS_DOMAIN_ID=217 PYTHONPATH="$PWD/src/zav_real_control:$PYTHONPATH" \
  python3 -m pytest -q src/zav_real_control/test
```

이전 DDS 2링크 구성은 `config/dds_legacy.yaml`, `rc_flight_dds_legacy.launch.py`, `docs/dds_legacy.md`에 분리 보관했다. 현재 기본 실행에서 사용하지 않는다.

## 탐조등용 RC AUX1 토글

NXT Dual의 `hkust_nxt-dual_v1.17.0_zav_aux1_toggle_v5.px4`에는 선택적인 탐조등
토글 출력이 있다. 이 기능은 FC에서 처리하며 ROS나 Radxa 연결이 필요하지 않다.
모터 출력, 비행 모드, RC 입력 메시지 자체는 변경하지 않는다.

지상 비무장 상태에서 펌웨어를 업로드한 뒤 `RC_MAP_AUX1=9`,
`RC_AUX1_TOG=1`을 저장하고 FC를 재부팅한다. `RC_AUX1_TOG=0`(기본값)은 기존
비례 전달, `1`은 누를 때 채널 값이 커지는 버튼, `2`는 작아지는 버튼용이다.
이 옵션은 부팅 시 읽으므로 바꾼 뒤에는 재부팅이 필요하다.

기존에 탐조등을 연결한 출력의 기능은 `RC AUX 1`을 유지한다. `RC_MAP_AUX1=9`는
RC 입력 채널 9를 뜻하며 물리 출력 9번을 뜻하지 않는다. 다른 RC 출력이나 모터
출력으로 기능을 재할당하지 않는다.

버튼을 약 0.1초 이상 누를 때마다 OFF/ON이 바뀌며 놓으면 유지된다. 길게 눌러도
반복 전환하지 않는다. 부팅·입력 상실 후에는 버튼을 먼저 놓은 상태로 유지해야
첫 누름을 인정한다. 유효하지 않은 RC, 잘못된 입력 또는 원본 RC 표본이 500ms를 초과해
오래되면 논리 상태가 OFF로 초기화된다. 기본 논리 출력은 OFF=-1(MIN), ON=+1(MAX)다.
출력 반전, PWM MIN/MAX/DIS/FAIL, prearm·kill·termination 등 기존 PX4 출력 제한은
그대로 적용되므로 실제 점등/소등과 비무장 시 동작은 탐조등 장치에서 확인해야 한다.
`manual_control_setpoint.aux1`에는 래치 상태가 아닌 원래 버튼 입력이 계속 표시된다.

공식 근거: [PX4 외부 odometry](https://docs.px4.io/v1.14/en/ros/external_position_estimation), [MAVLink 메시지](https://mavlink.io/en/messages/common.html), [PX4 시리얼 설정](https://docs.px4.io/v1.14/en/peripherals/serial_configuration). 진단 패치는 로컬 PX4 v1.14.3의 `ESTIMATOR_STATUS.hpp`, `mavlink_timesync.cpp`, `Timesync.hpp` 동작을 기준으로 작성했다.
