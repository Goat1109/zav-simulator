# 이전 DDS 구성 — 현재 기본 실행과 다름

아래는 이전 2링크 구성 기록이다. 현재 구성은 패키지 README를 따른다.

# Radxa Dragon Q6A + Mid360s + NXT PX4 v2 실기체

ROS 2 Humble에서 조종기 **Position 모드** 비행을 위한 구성이다. 조종기는 FC에 직접 연결한다. Radxa는 외부 odometry와 이상 시 LAND 명령만 전달한다. 자동 시동, 이륙, Offboard 제어, 모터 명령은 없다. 시뮬레이터와 flight manager는 실행하지 않는다.

현재 코드는 **PX4 1.14.3 / 제공된 px4_msgs release/1.14** 기준이다. NXT PX4 v2는 보드 이름이지 펌웨어 버전이 아니다. 실제 버전·메시지 ABI·필수 DDS 토픽을 확인하기 전 기본 실행은 읽기 전용이며 EV/파라미터/LAND를 쓰지 않는다. 다른 버전이라면 `expected_firmware` 숫자만 바꾸지 말고 메시지, 파라미터, 상태 enum과 DDS 설정을 함께 맞춰야 한다.

## 데이터 흐름

```
Mid360s Ethernet -> Livox driver -> FAST-LIVO2 /zav/lio/odometry
                                      |
                                 lio_bridge
                                      |
Radxa USB-TTL ===== TELEM2 XRCE-DDS ===== PX4 EKF2 <- FC IMU
                                      |
Radxa USB ========= FC USB MAVLink ===== EKF2_BARO_CTRL 변경/확인
```

**DDS와 MAVLink는 별도 링크 2개가 필요하다.** USB-TTL 한 UART를 두 프로세스가 동시에 사용하면 안 된다. 기본 구성은 USB-TTL ↔ TELEM2 DDS, 별도 FC USB ↔ Radxa MAVLink다. 두 번째 링크를 별도 UART/UDP로 바꿔도 된다. FC USB 포트를 QGC와 Radxa가 동시에 열 수 없으므로 QGC는 다른 텔레메트리 링크를 이용하거나 연결을 분리한다.

USB-TTL은 FC의 해당 포트 전압과 호환되는 것을 사용하고 TX↔RX, RX↔TX, GND를 연결한다. FC/USB/라이다 전원 구성은 보드 문서를 따른다. 나중에 Q6A GPIO UART로 옮길 때는 pinmux·핀 번호·전압을 확인하고 `agent.device`, `agent.baud`만 바꾼다. GPIO가 일반 5V TTL을 받아들인다고 가정하면 안 된다.

## 이번에 추가한 것

- `zav_real_control`: 실제 FC용 좌표/속도/장착 위치 변환, 측정 시각 검증, 외부 odometry 전송, 운동 연속성 감시, 기압계 전환 및 LAND.
- FAST-LIVO2의 `uav.px4_odom_en` 옵션과 `/zav/lio/odometry`: pose는 중력 정렬 Z-up 월드, twist는 Mid360s IMU 좌표계. 기존 `/LIVO2/imu_propagate`, `/aft_mapped_to_init` 출력은 유지한다. 기존 IMU propagate 출력의 속도는 world 좌표이므로 그대로 일반 Odometry로 해석하면 안 된다.
- 하드웨어 센서/LIO launch, 개별 UART 설정, Agent 설치/실행, 실제 FC DDS/메시지 확인 스크립트.
- 기존 카메라·dehaze·bundle sender 코드는 보존했다. 기본은 **카메라 없이 LiDAR+Mid360s IMU**이며, 카메라가 필요하면 기존 보정/드라이버를 실행하고 `camera:=true`를 준다.

## 1. 실제 FC 버전과 DDS 준비

QGC에서 설치된 펌웨어 버전과 정확한 보드/빌드 대상을 먼저 확인한다. 실제로 FC에 올릴 **동일 소스**를 사용해 다음을 실행한다. PX4 소스는 Radxa ROS 워크스페이스 밖에 둔다.

```bash
cd ~/zenith_ws/real_ws
python3 src/zav_real_control/scripts/prepare_px4.py \
  --px4-dir /path/to/actual/PX4-Autopilot --verify-only
```

메시지가 다르면 중단한다. 메시지가 맞고 DDS 토픽만 빠졌다면 다음 명령으로 그 **펌웨어 소스의** `dds_topics.yaml`을 보완한다.

```bash
python3 src/zav_real_control/scripts/prepare_px4.py \
  --px4-dir /path/to/actual/PX4-Autopilot
```

이후 NXT PX4 v2의 올바른 타깃으로 펌웨어를 빌드/업로드해야 토픽이 실제로 나온다. 이 스크립트는 펌웨어를 빌드하거나 FC에 쓰지 않는다. 시뮬레이터용 PX4 바이너리를 실기체에 올리면 안 된다. 플래시된 빌드가 검사한 소스와 같음을 확인한다.

필요한 DDS 출력은 `vehicle_status`, `vehicle_local_position`, `vehicle_land_detected`, `timesync_status`, `vehicle_command_ack`, `estimator_aid_src_baro_hgt`; 입력은 `vehicle_visual_odometry`, `vehicle_command`다. 기압계 aid 토픽은 파라미터 ACK와 별개로 **실제 융합 성공** 확인에 필요하다. 해당 토픽을 빼고 확인 플래그만 켜면 READY가 되지 않는다.

## 2. Radxa 준비/빌드

이미 시험한 Livox-SDK2와 센서 의존성을 사용한다. 다른 PC의 `build/install`, x86 라이브러리는 옮기지 않는다. 아래 경로는 워크스페이스를 옮겼다면 맞춰서 사용한다.

```bash
cd ~/zenith_ws/real_ws
sudo apt update
sudo apt install python3-venv python3-pip python3-rosdep python3-colcon-common-extensions \
  cmake git build-essential libopencv-dev libeigen3-dev \
  ros-humble-ament-cmake-auto ros-humble-rmw-fastrtps-cpp
# rosdep이 처음이면 sudo rosdep init 을 한 번 실행
rosdep update
sudo usermod -aG dialout "$USER"
# dialout 적용은 로그아웃/로그인 후 확인
bash src/zav_real_control/scripts/build_radxa.sh
bash src/zav_real_control/scripts/install_agent.sh
```

ROS 패키지는 `.venv`의 Python으로 빌드하고 필요한 FAST-LIVO2/드라이버/px4_msgs/control 의존성만 선택한다. Agent는 **v2.4.3**을 `.local/agent`에 별도 설치하며 시스템 ROS의 DDS 라이브러리를 덮어쓰지 않는다. 메모리가 부족하면 `ZAV_BUILD_JOBS=1 bash .../build_radxa.sh`로 빌드한다. 이 저장소에서 Radxa 전체 빌드는 실행하지 않았으므로 실제 ARM/SDK/OS 의존성 오류는 Radxa 빌드 로그로 확인해야 한다.

## 3. 배선과 설정 파일

```bash
ls -l /dev/serial/by-id/
nano src/zav_real_control/config/hardware.yaml
```

수정할 항목:

- `agent.device`: 실제 DDS USB-TTL 장치. 기본 921600 baud, ROS domain 23.
- `mavlink.connection`: **별도 FC USB** 장치. 기본 baud 115200; UDP라면 pymavlink 연결 문자열 사용.
- `expected_firmware`: 실제로 검증한 버전. 현재 지원/테스트 기준은 `[1,14,3]`.
- `firmware_abi_verified`: 위 메시지/DDS/실제 업로드 확인을 완료한 뒤 `true`.
- 장착값: 사용자 확인에 따라 시뮬레이션과 동일한 pitch +22°, IMU lever arm을 기본 적용했다. 값은 body-FLU 기준이며 **FC EKF 기준 원점**과 실제 장착이 같아야 한다. 다른 FC IMU 원점/장착이면 다시 측정하고 수정한다.

사용자가 지정한 장착값(body-FLU 기준, 회전은 Z-Y-X 순서):

| 센서 | 위치 x, y, z (m) | 회전 z, y, x (degree) |
| --- | --- | --- |
| LiDAR | 0.01215, 0, 0.11503 | 0, 22, 0 |
| 카메라 기구 좌표계 | 0.03215, 0, 0.03503 | 0, 0, 0 |
| Mid360s IMU | 0.03888, 0.02329, 0.15182 | 0, 22, 0 |

카메라 회전 0은 기구 장착 좌표계 값으로 기록했다. 영상의 optical 좌표계 변환과 FAST-LIVO의 카메라 내부/외부 보정값은 별도로 유지한다. IMU의 입력 위치는 기존 계산값을 소수점 5자리로 반올림한 값과 일치한다. FAST의 LiDAR–IMU 보정값은 이 반올림만을 이유로 변경하지 않는다.

브리지가 IMU 위치를 기체 기준점으로 보정하므로 `EKF2_EV_POS_X/Y/Z=0`을 쓴다. PX4에서 같은 lever arm을 또 빼지 않는다. FAST `extrinsic_T/R`를 바꾸면 bridge 장착값도 다시 계산한다.

기존 Livox 설정은 Radxa IP `192.168.1.50`, LiDAR `192.168.1.173`이다. 실제 센서 주소/인터페이스가 다르면 `livox_ros_driver2/config/MID360s_config.json`을 수정하고 실제 Ethernet 인터페이스를 설정한다. 실행 전 포트/주소 충돌과 센서 패킷 수신을 확인한다.

FC 파라미터는 `firmware/external_vision.params`, `firmware/serial_links.params`를 **체크리스트로 QGC에서 검토/설정**한다. QGC import 형식이 아니며 일괄 자동 적용하지 않는다. 실제 TELEM2를 점유한 MAVLink 인스턴스만 해제한다. 기본은 `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600`, `UXRCE_DDS_DOM_ID=23`. 포트 변경 후 FC를 재부팅한다.

현재 기압계 진단은 estimator instance 0을 관찰하므로 체크리스트의 `SENS_IMU_MODE=1`, `EKF2_MULTI_IMU=0`인 단일 EKF 구성을 사용한다. 다중 EKF의 primary 전환을 추적하는 구현은 포함하지 않았다.

정상 상태는 GPS 융합 OFF, EV 위치/속도/고도/yaw 사용, baro 융합 OFF다. FC IMU는 계속 EKF2에서 사용한다. baro **센서 자체를 끄는 것이 아니라 EKF 융합을 끄는 것**이다. FC의 기체 설정, IMU/RC/ESC 보정, 실제 PID, 배터리 및 RC loss failsafe는 실제 기체 설정을 유지한다. SITL의 safety 해제 파라미터는 복사하지 않는다.

`EKF2_EV_DELAY=0`과 잡음/분산은 시작값이다. 측정 지연·ULog innovation을 보고 튜닝해야 한다. 출력 world는 중력 정렬된 임의 heading FRD이며 진북 NED라고 표시하지 않는다. ROS/PX4 DDS 시각은 실제 epoch여야 하고 offset은 PX4 DDS가 한 번만 보정한다. Livox host receive timestamp fallback이 epoch라는 사실만으로 하드웨어 동기화가 보장되지는 않는다. PTP/센서 timestamp 및 end-to-end 지연을 별도로 확인한다.

## 4. 실행

새 터미널마다 아래 환경을 source한다. 상위 시뮬레이션 `install/setup.bash`, `run_fastlivo.sh`, `zav_sim.py`는 실행하지 않는다.

```bash
cd ~/zenith_ws/real_ws
source src/zav_real_control/scripts/env.sh
```

터미널 A — Agent:

```bash
bash src/zav_real_control/scripts/run_agent.sh "$ZAV_REAL_CONFIG"
```

터미널 B — Mid360s + FAST-LIVO2:

```bash
ros2 launch zav_real_control sensors_lio.launch.py
# 이미 Livox driver를 별도로 실행했다면 start_driver:=false
# 카메라는 기존 카메라 노드를 따로 실행한 뒤 camera:=true
```

터미널 C — 최초 읽기 전용 점검:

```bash
ros2 launch zav_real_control rc_flight.launch.py config:="$ZAV_REAL_CONFIG"
```

터미널 D — 상태:

```bash
ros2 run zav_real_control inspect_status
ros2 topic hz /zav/lio/odometry
ros2 topic hz /fmu/out/vehicle_status
```

MONITOR_ONLY에서는 EV도 보내지 않으므로 PX4 위치 추정이 준비되지 않을 수 있다. 이 단계는 연결/시각/버전/입력 확인용이다. 확인 후 C를 종료하고, **프로펠러를 제거한 지상 점검**에서 다음으로 전환한다.

```bash
ros2 launch zav_real_control rc_flight.launch.py config:="$ZAV_REAL_CONFIG" publish:=true
```

`publish:=true`여도 ABI/장착 확인 플래그와 MAVLink 실제 버전 확인이 모두 통과해야 EV/파라미터/LAND를 전송한다. 정상 기압계 OFF 설정은 disarmed일 때만 수행하며, latch 후 기압계 ON은 armed여도 허용한다. 조종기는 FC에 직접 연결하고 Position 모드로 수동 조종한다. 이 구성은 Offboard 모드를 사용하지 않는다.

`ready=true`는 companion 상태이며 **RC arming을 차단하는 하드웨어 인터록은 아니다**. 반드시 QGC의 EKF/센서 상태와 함께 확인한다. `blocked_by`는 실패 항목을 표시한다. `runtime_firmware`는 별도 MAVLink 링크, `clock`은 DDS 시각/동기화, `baro_off_confirmed`는 실제 baro aid 토픽/융합 OFF 상태를 확인한다. 다른 EV 브리지와 동시에 실행하면 안 된다.

## 5. 운동 연속성 failsafe

비행 중 연속 두 샘플의 **측정 시각 차이 dt**로 계산한다.

- 위치: `|p_now - (p_prev + v_prev_world * dt)| > 0.5 + 0.3*dt + 0.5*20*dt²` m.
- 자세: quaternion 최단 회전각 `> 20deg + 4*dt` rad/s 항. 정확히는 `20*pi/180 + 4*dt` rad.
- 비교 간격은 최대 0.25초. 기준 초과 1회에 latch한다. 50Hz에서 대략 위치 0.51m, 자세 24.6deg다. 기본값은 보장된 실기체 한계가 아니라 튜닝 시작점이다.

발동 순서:

1. 튄 샘플부터 모든 EV 전송 중단, latch 유지.
2. 별도 MAVLink로 `EKF2_BARO_CTRL=1` 요청 및 readback 확인.
3. baro aid의 실제 정상 융합과 유효한 수직 위치/속도를 확인.
4. NAV_LAND 요청. LAND/DESCEND 진입 전까지 1초마다 재시도. ACK만으로 착륙 성공이라고 하지 않는다.
5. 3초 내 기압계 확인 실패 시 `BARO_TIMEOUT`으로 표시하고 best-effort LAND를 요청한다. **융합/착륙 성공을 보장하지 않는다.**

조종사가 명시적으로 Altitude/Stabilized/Manual로 모드 스위치를 바꾸면 반복 LAND 요청을 멈추고 수동 인계를 존중한다. EV 차단과 baro ON latch는 유지한다. 자동 fallback으로 nav_state만 바뀐 경우와 조종사의 `nav_state_user_intention` 변경을 구분한다. 착륙·disarm 후 프로세스를 종료하고 원인을 확인한 뒤 지상에서만 재시작한다. 비행 중 재시작으로 latch를 해제하지 않는다.

이 정책의 판단 기준은 요청대로 **운동 연속성 하나뿐**이다. 점진적 drift, 0.25초 이상 데이터 공백, LiDAR 보정 정지, EKF reject, 통신 단절은 착륙 트리거에 포함하지 않았다. 잘못된 시각/프레임/NaN은 전송에서 제외하지만 그것만으로 LAND하지 않는다. 특히 **Radxa 전원 상실·프로세스 중단·링크 단절 시 companion이 기압계를 켤 수 없다.** baro OFF에서 이 경우의 착륙은 이 코드가 보장하지 않으며 FC 기본 failsafe와 별도 watchdog/상시 보조 고도 설계가 필요하다. baro 자체 이상과 수평 위치 상실도 복구하는 구성은 아니다.

## 6. 비행 전 검증 순서

1. 실제 FC 버전/타깃/DDS ABI를 확인하고 FC 센서·RC·모드·기체 설정을 완료한다.
2. 프로펠러를 제거하고 EV 전송, 정상 baro OFF, 전후좌우/상하 이동과 yaw 시 EKF 방향·부호·크기, ULog innovation을 확인한다. 정지시켜 LIO 초기화하고 비행 중 맵/원점을 재시작하지 않는다.
3. 먼저 SITL/HIL 또는 모터 출력이 격리된 시험 환경에서 위치/자세 jump 주입 → EV 차단 → baro ON/fused → LAND 상태 전환과 RC takeover를 재현한다. 이 저장소의 호스트 테스트는 가짜 ROS 입력만 사용하며 실제 FC 착륙 검증을 대신하지 않는다.
4. 짧은 저고도 비행에서 정상 Position 제어·고도·지연부터 확인한다. 실비행 중 강제 jump 주입을 첫 시험으로 사용하지 않는다.
5. 실제 빌드/실행 오류 공유 시 Radxa OS 버전, QGC 펌웨어 버전, 빌드 첫 오류, `inspect_status` 출력, 포트 설정/ULog를 함께 제공한다.

로컬 검증은 `test/`의 수학/시간/급변/전환/파라미터 프로토콜 단위 테스트 및 격리 ROS callback 통합 테스트다. 실제 Radxa 빌드·FC 업로드·실비행은 수행하지 않았다.

공식 근거: [PX4 v1.14 외부 위치](https://docs.px4.io/v1.14/en/ros/external_position_estimation), [uXRCE-DDS](https://docs.px4.io/v1.14/en/middleware/uxrce_dds), [시리얼 연결 예](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_rpi), [파라미터](https://docs.px4.io/v1.14/en/advanced_config/parameter_reference), [Q6A GPIO](https://docs.radxa.com/en/dragon/q6a/hardware-use/pin-gpio).

테스트 실행(ROS 메시지가 빌드된 환경):

```bash
cd ~/zenith_ws/real_ws
source src/zav_real_control/scripts/env.sh
ROS_DOMAIN_ID=217 PYTHONPATH="$PWD/src/zav_real_control:$PYTHONPATH" \
  python3 -m pytest -q src/zav_real_control/test
```
