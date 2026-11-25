# 실기 SUPER + OMMPC 운용

`zav-simulator`의 `argo_v2` 브랜치에서 가져온 SUPER/ROG-Map과 OMMPC 코어를 사용한다. 기체 설정은 `argo_v1`, 플래너·제어기 튜닝은 `src/zav_super/config/common.yaml`이다. EPIC은 포함하지 않는다. 기존 실행은 자동 시동하지 않으며, 아래의 별도 자동 이륙 구성을 명시적으로 실행한 경우에만 정상 시동·모드 전환을 요청한다.

경로: **SUPER-LIO → body/world 어댑터 → SUPER 회피 궤적 → OMMPC → 기존 단일 MAVLink 브리지 → PX4 각속도 제어기**. EV도 같은 브리지가 보낸다. 기존 위치 목표 시험과 SUPER 각속도 출력을 동시에 보내지 않는다.

## 1. 빌드 및 터미널 환경

현재 argo 컨테이너에서 빌드를 완료했다. 재빌드할 때만 실행한다.

```bash
cd /root/argo_ws
bash scripts/build_super_hardware.sh
```

각 터미널을 argo 컨테이너로 열고 아래 환경을 동일하게 적용한다. `119`는 현재 hardware.yaml의 실기 도메인이다. 다른 번호를 쓰려면 모든 터미널을 같은 번호로 맞춘다.

```bash
source /opt/ros/humble/setup.bash
source /root/argo_ws/install/setup.bash
export ROS_DOMAIN_ID=119
export ROS_LOCALHOST_ONLY=0
```

아래 실행 명령은 사용자가 지상 확인을 마친 뒤 직접 실행한다. 코드 설치/테스트 과정에서 실제 FC 연결, 무장, 모터 구동, 파라미터 변경은 실행하지 않았다.

## 2. 실행할 명령

터미널 A — 센서, SUPER-LIO, SUPER 플래너, OMMPC를 함께 실행한다. 기존 `super_lio`/`lio`/`livo`를 중복 실행하지 않는다.

```bash
ros2 launch zav_super super_navigation.launch.py execute:=true
```

터미널 B — SUPER 전용 브리지 선택. 기존 `bridge` 별칭의 legacy 브리지와 동시에 실행하지 않는다. FC 연결은 기존 TELEM2 USB-TTL 설정 그대로다.

```bash
ros2 launch zav_real_control rc_flight.launch.py \
  publish:=true offboard:=true offboard_source:=super
```

터미널 C — 현재 `state` 별칭 또는 다음 명령으로 확인한다.

```bash
ros2 run zav_real_control inspect_status
```

`SUPER / OMMPC`, 단계, 실제 수평 속도, 목표까지 직선거리, 연산 시간, 제어 메시지 나이, 정규화 추력/호버 기준, FC 파라미터 불일치가 표시된다. EV/배터리/기체 상태 표시는 기존 기능을 유지한다. `SUPER_READY_RC8`는 공중 Position 정지 상태에서 준비 스트림이 1.2초 이상 전송된 경우다. 지상에서는 `SUPER_FLIGHT_OR_EV_UNHEALTHY`로 기다리는 것이 정상이다. 이것은 PX4 무장 허가 판정과 다르다.

### Position → Offboard 호버 추력 인계

`log_64`에서 Position 호버 출력 약 0.30이 Offboard 진입 직후 기존 `MPC_THR_HOVER=0.18` 기준으로 떨어지는 문제를 확인했다. 실기 SUPER 브리지는 이제 **Position의 실제 `ATTITUDE_TARGET.thrust`를 같은 MAVLink 링크에서 읽어** 호버 스케일을 측정한다. FC 파라미터를 쓰거나 카메라/점군 구독을 추가하지 않는다.

- Position·무장·공중·RC8 LOW·3차원 속도 0.15m/s 이하·기울기 10° 이내에서 측정한다. 1초 이상에 걸친 최소 20개 표본, 최근 1.5초 출력 범위 0.04 이하, 수신 공백 0.25초 미만을 요구한다. 실제 요청 주기는 20Hz다.
- 먼저 `SUPER_CALIBRATING_HOVER_THRUST`가 표시되고, 측정 및 준비 스트림 1.2초가 끝나면 `SUPER_READY_RC8`가 된다. 보통 수 초간 정지 호버링이 필요하다. 측정이 없으면 0.18로 대신 전송하지 않는다.
- RC8 상승 시 측정값을 고정하고, 해당 Offboard 진입 동안 `thrust = 측정한 호버 기준 × collective_acceleration / 9.81`을 사용한다. Offboard의 출력은 재학습하지 않는다. Position으로 회수 후 다음 진입에는 새로 측정한다. RC8 상승 후 진입이 2초 넘게 지연되면 LOW에서 다시 준비한다.
- OMMPC는 Offboard 모드 통보가 도착할 때 높이를 새로 잡지 않고, 마지막 Position 준비 스트림의 위치·높이 목표를 유지한다.
- state/웹의 `호버 기준`은 실제 측정값이다. `FC 초기 파라미터`는 기존 `MPC_THR_HOVER` 읽기값으로 구분한다. EV 단절·시각·점프·제어기 오류 및 기존 Offboard-loss 대응은 유지한다.

이 수정은 경로 계획 시간 초과와 `NO_PLAN` 정책을 변경하지 않는다. 첫 재시험은 **이동 시작 버튼을 누르지 않고 Position → Offboard 정지 호버 전환만** 기록한다. 원시 센서 Bag과 같은 비행의 PX4 ULog를 함께 보관한다. 코드 변경 전 실행 중이던 프로세스에는 적용되지 않으며, 착륙·비무장 후 SUPER/브리지를 다시 실행해야 한다.

터미널 D — **실제 Offboard 호버링에 들어간 다음** 아래 명령을 실행하면 현재 기체 전방 10m의 목표를 한 번 요청한다. 기본 거리는 10m이며, SUPER 실행 시 `distance:=7.0`처럼 0 초과 20m 이하의 거리를 지정할 수 있다.

```bash
ros2 service call /zav/super/go_front std_srvs/srv/Trigger '{}'
```

웹에서는 `SUPER 전방 거리 (m)`를 입력한 뒤 시험을 시작한다. 기본값 10m, 범위는 0m 초과~20m다. 거리와 시작 요청은 `/zav/super/go_forward` (`zav_super/srv/GoForward`)로 함께 보내므로 제어기를 재시작할 필요가 없다. 터미널에서 거리를 함께 지정하는 실제 이동 요청 예시는 다음과 같다.

```bash
ros2 service call /zav/super/go_forward zav_super/srv/GoForward '{distance_m: 5.0}'
```

이 서비스는 실제 이동 요청이다. 자동 이륙·무장 서비스가 아니다. 지상/비무장/Position/센서 지연/제어기 오류/이동 중에는 거절한다. 최종 목표 도착·정지 확인 후에는 같은 Offboard 진입에서도 다음 임무를 받을 수 있다.

### Patrol / RTH / 자동 이륙 (2026-11-25)

웹에 `Patrol · 7m 정사각형`, `RTH · 출발점 복귀·착륙`, `자동 이륙 · 40cm 호버링`을 추가했다. 기존 전방 이동과 함께 SUPER/OMMPC를 사용한다. 이 변경은 장치 없는 합성 상태·가짜 MAVLink 송신기로 검증하며 실기 비행 검증을 뜻하지 않는다.

- Patrol은 요청 순간의 위치·yaw로 네 목표를 고정한다. 앞 7m → 오른쪽 7m → 뒤 7m → 왼쪽 7m이며 장애물 회피로 실제 경로는 휘어질 수 있다. 각 변의 바깥쪽 yaw는 최초 기체 기준 왼쪽 → 앞 → 오른쪽 → 뒤다. 이동 전에 제자리에서 최대 0.4rad/s 기준으로 방향을 맞추고, 5° 이내·속도 0.15m/s 미만을 1초 확인한 뒤 다음 변을 시작한다. 도착은 목표 거리 0.2m 미만·속도 0.15m/s 미만·정지 궤적 끝을 1초 확인한다. 마지막에는 출발점에서 뒤쪽을 보며 호버링한다.
- Home은 SUPER를 이륙 전에 실행했을 때, 비무장·지상 odometry에서 시동 직전 좌표와 yaw를 저장한다. LIO/world 상대 좌표이며 GPS Home이나 세계 원점이 아니다. 이미 떠 있는 상태에서 처음 실행했거나 위치/시각이 리셋되면 RTH에 사용하지 않는다.
- RTH는 정상 SUPER Offboard에서 정지 중뿐 아니라 이동 중에도 요청할 수 있다. 진행 중 목표를 교체하고 새 계획을 기다리는 동안 기존 유효 궤적을 잠시 유지한다. 교체 궤적을 1초 안에 확인하지 못하면 전송을 중단해 기존 Offboard-loss 대응으로 넘긴다. 복귀 목표는 Home XY와 `Home Z+0.4m`의 단일 3D 목표다. 현재 높이의 Home 상공 경유점은 만들지 않는다. 다층 복귀는 지도에 층 사이 연결 경로가 있고 지도 범위와 가상 고도 제한 안에서 경로가 성립해야 한다. SUPER는 3차원 회피를 하므로 중간 고도가 변할 수 있고 전역 최단 경로를 보장하지 않는다. 지도가 복귀 경로를 만들 수 없으면 Home을 향한 직선 이동으로 우회하지 않는다.
- 최종 접근·정지 확인 뒤 기존 UART에서 정상 `MAV_CMD_NAV_LAND`를 요청한다. 실제 AUTO.LAND를 확인하면 OMMPC 출력을 멈추고 PX4가 착륙·시동 해제를 담당한다. `COM_DISARM_LAND`의 신선한 읽기값이 0초 초과 30초 이하여야 하며 값을 자동 변경하지 않는다. 착륙·비무장 확인 후 `RTH_LANDED`를 표시한다. EV 보호·RC 회수가 우선이며 이 RTH를 FC의 GPS RTL로 구현하지 않았다.
- 자동 이륙 버튼은 기존 SUPER/브리지가 꺼진 지상에서 별도 구성을 한 번 실행한다. 사용자가 승인한 기체 설정 `config/automatic_takeoff.json`의 초기 호버 기준 0.31을 웹과 launch가 공유한다. 매번 입력하지 않으며, FC에서 읽은 값을 무조건 추종하거나 자동으로 덮어쓰지 않는다. RC8 HIGH·신선한 RC·스로틀 최저·비무장·지상·수평·EV/추정기/FC 프로필을 확인하고 1.2초 준비 스트림 → 실제 Offboard 확인 → 정상 시동 확인 → 추력 1초 램프 → 3초의 7차 수직 위치/속도/가속도 프로파일로 Home+0.4m → 정지 호버링 순서로 진행한다. 강제 시동은 없으며 FC 거절·준비 45초 초과·RC8 회수·피드백 실패는 재시동하지 않는다. 자동 이륙은 상공이 비어 있음을 사용자가 확인하는 동작이며 SUPER 수평 목표를 발행하지 않는다.

웹의 세 임무 요청은 웹 서버가 보유한 ROS 서비스 클라이언트를 재사용한다. 버튼마다 `ros2 service call` 프로세스를 새로 만들지 않고, 서비스의 `success=False`도 화면 작업 실패로 표시한다.

자동 이륙 전에는 기체 설정의 초기 호버 기준과 읽어온 `MPC_THR_HOVER`의 차이가 0.05 이하여야 한다. Position 회수·PX4 착륙 시 큰 추력 기준 차이를 피하기 위한 조건이다. 불일치는 `SUPER_AUTO_HOVER_PARAMETER_MISMATCH`로 대기하며 자동으로 FC 값을 덮어쓰지 않는다. 현재 0.31은 이 기체의 과거 로그에 근거해 승인한 초기 후보이며 새로운 자동 이륙의 실기 검증 완료를 뜻하지 않는다. 기체·배터리 구성이 달라지면 재평가해야 한다. 설정 파일이 없거나 잘못되면 자동 이륙을 거절하며 임의 기본값으로 대체하지 않는다. 기존 Position 호버 측정 경로는 그대로 유지한다.

자동 이륙 후 Position으로 회수하려면 swB Position을 확인하고, 아직 Offboard일 때 스로틀을 중앙에 맞춘 뒤 RC8 LOW로 전환한다. 실제 FC 모드가 Position인지 확인한다. 최저 스로틀인 채로 Position에 들어가면 하강 입력이다. swB가 Altitude이면 RC8 해제만으로 Position을 보장하지 않는다.

EPIC 등 상위 임무 노드는 정상 SUPER Offboard에서 `/zav/super/patrol` 또는 `/zav/super/return_home` (`std_srvs/srv/Trigger`)를 호출하고 `success/message`와 `/zav/super/controller_status`의 `phase/mission`을 확인할 수 있다. RTH는 현재 임무를 대체하며 자동 재요청하지 않는다. 배터리 절반 사용에 따른 자동 RTH 트리거는 이번 구현에 포함하지 않았다.

## 3. 예상 동작 순서

1. A/B/C를 실행하고 EV 정상·배터리·FC 메시지와 센서 방향을 확인한다. 초기 연결은 RC8 LOW로 둔다.
2. 기존 RC 절차로 Position 모드에서 이륙하고 스틱을 중앙으로 놓아 정지 호버링한다. 이륙 명령은 이 패키지가 보내지 않는다.
3. `state`에 `준비 완료 — RC8 전환 대기`가 표시되면 RC8 HIGH로 전환한다.
4. **FC 실제 모드가 Offboard이고 SUPER 단계가 HOVER인지 확인한다.** 현재 위치·고도·yaw를 기준으로 호버링한다. 단순히 스위치가 HIGH인 것만으로 진입 성공을 판단하지 않는다.
5. D의 서비스를 호출한다. 목표를 받은 시점의 기체 전방으로 기본 10m, 같은 높이에 목표를 만든다. SUPER가 점유 지도를 사용해 우회 경로를 계속 갱신하고 OMMPC가 궤적의 위치·속도·가속도·jerk/yaw를 사용한다. 기존 `go_front` 별칭은 5m 위치 제어 시험이므로 이 시험에 사용하지 않는다.
6. 최종 경로 끝에서는 호버링을 유지한다. `ENDPOINT_HOLD`는 정지한 경로 끝점이라는 뜻으로, 백업 경로 끝점일 수도 있다. 최종 10m 도착은 `남은 직선거리` 및 실제 위치를 함께 확인한다. 목표 스냅 허용 거리는 원본 설정대로 0.3m다.
7. 중단하거나 착륙하려면 RC8 LOW로 Position 조종권을 회수하고 기존 RC 착륙 절차를 사용한다. 비행 중 A/B를 Ctrl+C로 종료하는 것을 정상 착륙 절차로 쓰지 않는다. 지상 정지·비무장 후 프로그램을 종료한다.

회피는 **3차원**이다. 목표 고도가 같아도 장애물을 피하며 고도가 변할 수 있다. 10m는 목표의 전방 변위이며 우회 경로 총길이가 아니다. RC 스틱의 Offboard 강제 해제 여부는 FC의 `COM_RC_OVERRIDE` 설정에 따른다. RC8 LOW를 명시적인 Position 복귀 수단으로 사용한다. Kill은 기존 FC 기능으로 유지된다.

## 4. PX4 파라미터와 속도

**Position의 `MPC_VEL_MANUAL`/`MPC_XY_VEL_MAX`를 올리지 않는다.** 이번 입력은 MAVLink `SET_ATTITUDE_TARGET`의 body-rate + collective thrust이며, PX4의 위치·속도·자세 제어기를 건너뛰고 각속도 제어기를 사용한다. 따라서 해당 Position 제한으로 SUPER의 속도를 제한할 수 없다. 시뮬레이터용 PX4 전체 파라미터를 실기에 덮어쓰지 않는다.

원본 `common.yaml`의 주요 값:

| 항목 | 값 | 의미 |
|---|---:|---|
| `traj_opt.boundary.max_vel` | 3 m/s | 현재 common.yaml 궤적 최적화 속도 상한 |
| `max_acc` / `max_jerk` | 6 m/s² / 10 m/s³ | 궤적 최적화 경계 |
| `max_bodyrate_xy` / `max_bodyrate_z` | 1.5 / 0.8 rad/s | OMMPC roll·pitch / yaw 각속도 경계 |
| `control_period` / `step_T` | 0.02 / 0.02 s | 제어 주기 / 예측 간격 |
| `fsm.replan_rate` | 15 Hz | 재계획 주기 |
| `robot_r` / map resolution | 0.2 / 0.15 m | 충돌 모델 반경 / 지도 해상도 |

3m/s는 계획 궤적의 최적화 경계이며 실제 순간 속도를 강제로 잘라내는 PX4 제한이 아니다. 추종 오차·외란에 따라 실제 속도는 다를 수 있다. 다른 튜닝 파일은 `common_config:=/절대경로.yaml`로 지정한다.

브리지는 기존 UART에서 다음 값을 **읽기만** 한다. 불일치하면 각속도 전송을 막고 이유를 표시한다. 값 변경은 자동 수행하지 않는다.

| 파라미터 | 확인 조건/용도 |
|---|---|
| `RC_MAP_OFFB_SW` | 8 |
| `COM_OF_LOSS_T` | 0.1~2.0s, 사용자 현재 설정 1.0s를 유지 |
| `COM_OBL_RC_ACT` | 0 = Position |
| `ZAV_EV_GUARD` | 1, 기존 FC EV 감시 활성화 |
| `MPC_THR_HOVER` | 0.1~0.8 범위 확인 및 초기 파라미터 기록; 실기 전환 스케일은 안정된 Position 출력에서 별도 측정 |
| `THR_MDL_FAC` | 0~1 범위 읽기; ESC 추력 선형화는 FC가 담당 |
| `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` | 최소 85.94°/s |
| `MC_YAWRATE_MAX` | 최소 45.84°/s |
| `MPC_XY_VEL_MAX` / `MPC_VEL_MANUAL` | 읽어서 기록, 변경하지 않음 |

추력 변환은 `Position에서 측정·고정한 호버 기준 × OMMPC collective_acceleration / 9.81`이다. `MPC_THR_HOVER`를 그대로 스케일로 사용하지 않는다. 시뮬레이터의 로터 계수/idle/scale 역산을 실기에 복사하지 않았다. 실제 모터·ESC·탑재중량에서 전환 응답을 검증해야 한다. 코어는 시뮬레이터와 동일하지만 실제 FC 내부 각속도 루프와 추력 모델을 포함한 폐루프 비행 성능은 아직 검증하지 않았다.

## 5. 추정기 선택 및 준비 출력

```bash
# 기본: SUPER-LIO, 카메라 없음
ros2 launch zav_super super_navigation.launch.py estimator:=super_lio execute:=true

# 기존 FAST-LIVO2의 LiDAR/IMU 모드
ros2 launch zav_super super_navigation.launch.py estimator:=lio execute:=true

# 기존 FAST-LIVO2의 카메라 융합 모드
ros2 launch zav_super super_navigation.launch.py estimator:=livo execute:=true
```

`super_lio` 모드는 원본 common의 IMU noise 설정을 기존 Mid360s 설정에 합치고, 플래너에 필요한 dense world cloud를 켠다. 기존 독립 `super_lio` 별칭 동작은 바꾸지 않는다. RViz를 켜지 않아도 플래너용 cloud가 나온다.

외부에서 추정기를 이미 실행했다면 다음처럼 센서 시작을 생략할 수 있다.

```bash
ros2 launch zav_super super_navigation.launch.py estimator:=external execute:=true \
  odom_topic:=/zav/lio/odometry cloud_topic:=/super_lio/cloud_world cloud_frame:=world
```

외부 입력은 기존 EV와 같은 `camera_init → mid360_imu` IMU odometry, IMU 좌표계 속도, world에 등록된 cloud 계약을 지켜야 한다. 임의 body odometry를 그대로 넣으면 안 된다. body 레버암/회전 보정은 EV와 같은 hardware.yaml에서 한 번 적용한다. cloud는 이미 world 등록이 끝났으므로 좌표값을 다시 회전하지 않는다. FAST-LIVO의 `/cloud_registered`는 `cloud_frame:=camera_init`이다.

기본 `execute:=false`는 플래너·센서와 제어기 상태만 띄우고 `/zav/super/rates`를 발행하지 않는다. 이동 서비스도 거절한다. 센서만 점검할 때는 이 기본 모드를 사용한다. `execute:=true`도 스스로 무장·이륙·Offboard 전환하지 않는다.

## 6. 중단 및 검증 범위

- 기존 EV 지연·점프·FC 감시/기압계 전환·착륙 대응이 우선한다. SUPER가 이를 덮어쓰지 않는다.
- OMMPC 입력 중단/무효/범위 초과/파라미터 불일치/다중 제어기/전송 공백은 각속도 전송을 중단하고 실패를 고정한다. 이 경우 EV가 계속 정상이라면 `COM_OF_LOSS_T` 이후 FC의 Offboard-loss 대응(Position)이 적용된다. **플래너 오류가 곧바로 LAND 명령이라는 뜻은 아니다.**
- 플래너 응답 5초 초과, 미완료 궤적의 heartbeat 1초 초과, 연속 solver 실패는 무효 출력을 보낸다. cloud 0.5초, odometry 0.25초, 브리지 상태 0.6초 초과도 무효다. 브리지는 마지막 제어 메시지 0.15초 초과 또는 무효를 즉시 거부한다.
- 이미 정지 끝점에 도달한 유효 궤적은 끝점 호버를 유지할 수 있다. 공중에서 고정된 실패 상태를 재시작해 억지로 복구하지 않고 RC로 조종권을 회수한다.

실제 장치를 열지 않는 테스트:

```bash
export ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1
python3 -m pytest -q src/zav_real_control/test/test_super_gate.py
colcon test --packages-select zav_super
python3 src/zav_super/test/ros_pipeline.py
```

마지막 명령은 가짜 odometry/cloud/FC 상태와 이상적인 각속도·가속도 기체 모델만 사용한다. UART/실제 FC/센서 노드를 시작하지 않는다. 빈 공간 및 박스 장애물 우회 7m 도착, cloud 단절 차단을 검사하고 `log/hardware_checks/super_offline_pipeline.json`에 저장한다. 이는 실기 비행 통과 판정이 아니다.
