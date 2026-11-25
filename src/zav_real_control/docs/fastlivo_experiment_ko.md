# FAST-LIVO2 + PX4 v3 실험 실행 순서

현재 Radxa의 `argo_dev` Docker와 `/root/argo_ws`, ROS 도메인 119 기준이다.
아래 launch는 자동 시동·이륙을 하지 않는다. EV 전송 활성화는 실제 FC의 추정과
실패 대응에 영향을 준다. 처음에는 프로펠러 제거·비무장·swA Kill ON으로 확인한다.
정상 운용의 입력 지연 안정성은 아직 미검증이며, 비상 대응 시험 통과와 구분한다.

## 1. 연결과 전원

- Radxa 외부 전원, LiDAR/카메라 연결과 전원을 확인한다.
- 조종기 ON, 스로틀 최저, swA Kill ON, swB Altitude로 둔다.
- FC에 LiPo를 연결하고 FC USB를 연결한다.
- 기존 LIVO/브리지가 실행 중이면 중복 실행하지 않는다. 이 문서는 자동 시작 서비스가 아니다.

컨테이너가 꺼져 있다면 Radxa 호스트에서 한 번 실행한다.

```bash
docker start argo_dev
```

## 2. 터미널 공통 준비

각 호스트 터미널에서 다음을 실행한다.

```bash
docker exec -it argo_dev bash
cd /root/argo_ws
source /root/argo_ws/src/zav_real_control/scripts/env.sh
echo "$ROS_DOMAIN_ID"
```

도메인 출력은 `119`여야 한다. env.sh는 ROS Humble, 이 워크스페이스와 실제 장비 설정을 읽는다.

## 3. 터미널 A: 센서 + LIVO

```bash
ros2 launch zav_real_control sensors_lio.launch.py \
  camera:=true \
  livo_config:=/root/argo_ws/src/FAST-LIVO2/config/zenith.yaml
```

Mid360s 드라이버, 카메라 드라이버, FAST-LIVO2를 함께 실행한다.
`camera:=true`는 카메라까지 융합하는 LIVO이고 기본값 `false`는 LIO다.
카메라 드라이버를 별도로 이미 실행했다면 `start_camera:=false`를 추가한다.
센서 초기화 중에는 기체를 가만히 둔다. 카메라·LiDAR 오류가 없는지 확인한다.
RViz나 영상 주파수 측정은 필요할 때만 켜고 종료한다.

## 4. 터미널 B: FC 연결 상태 확인, 이후 EV 활성화

처음에는 읽기 전용으로 실행한다.

```bash
ros2 launch zav_real_control rc_flight.launch.py \
  config:=/root/argo_ws/src/zav_real_control/config/hardware.yaml \
  publish:=false
```

다음 터미널의 status에서 firmware, heartbeat, clock, recent_odometry,
baro_off_confirmed, rc_takeover와 serial_io를 확인한다.
재부팅 직후 시간 동기화는 정상 표본 500개가 필요하므로 10 Hz 기준 약 50초 이상
걸릴 수 있다. `clock`이 계속 막히면 시간 동기화가 완료되지 않은 상태다.
`recent_odometry` 또는 `source_measurement_age`가 반복되면 정상 운용 준비로 보지 않는다.
체크를 우회하거나 허용 지연을 임의로 늘리지 않는다.

정상이면 B만 Ctrl+C로 종료하고 실제 EV 전송을 켠다.

```bash
ros2 launch zav_real_control rc_flight.launch.py \
  config:=/root/argo_ws/src/zav_real_control/config/hardware.yaml \
  publish:=true
```

브리지는 하나만 실행한다. 별도 UART 진단 프로그램으로 같은 FTDI 포트를 점유하지 않는다.
표준 브리지의 통신 확인 주기는 1ms, 상태 발행은 5Hz이며 LIVO 처리 주기와는 별개다.

## 5. 터미널 C: 가벼운 상태 확인

```bash
ros2 run zav_real_control inspect_status
```

정상 비무장 EV 전송 상태의 주요 항목:

```text
mode: LIVE
ready: true
armed: false
latched: false
blocked_by: []
phase: NORMAL_EV_ONLY
counts.out: 증가
```

`ready`는 브리지의 전송 조건이지, FC 시동 허가나 비행 안전 판정이 아니다.
확인 후 C만 Ctrl+C로 종료해도 B의 EV 전송은 유지된다.

필요하면 QGC MAVLink Console에서 FC 상태를 읽는다.

```text
listener zav_ev_guard_status -n 1
listener estimator_selector_status -n 1
listener vehicle_local_position -n 1
listener vehicle_status -n 1
```

FC guard의 `enabled=True`, 비무장 중 `watching=False`, `latched=False`가 정상이다.
실제 무장 후 정상 EV 융합이 확인되면 watching이 True가 된다.
`vehicle_local_position`은 위치·속도 validity와 `dead_reckoning`을 확인한다.
FRD 외부 yaw에서 `heading_good_for_control=False`만으로 EV 융합 실패라고 판정하지 않는다.

## 6. 지상 실험과 시동/비행의 구분

프로펠러 제거·Kill ON·비무장 상태에서 입력과 추정 상태를 관찰한다.
이 문서의 launch만으로 시동하거나 이륙하지 않는다. 무장 모터 시험은 별도의
주변 정리·Kill 조작 준비·짧은 제한 시간을 두고 진행한다.
실제 비행 중에는 LIVO와 EV 브리지를 계속 유지해야 한다.
반복된 지연으로 EV가 차단되거나 비상 전환이 발생하는 상태는 정상 비행 준비 상태가 아니다.

FC v3의 주요 설정은 `ZAV_EV_GUARD=1`, `ZAV_EV_TOUT=0.25`, 정상 운용 시
`EKF2_BARO_CTRL=0`, `NAV_RCL_ACT=3`이다. 이 문서는 해당 값을 자동으로 쓰지 않는다.
상실 시 기압계 활성화 후 융합 확인 또는 3초 타임아웃에 따라 native Land를 요청한다.
수평 위치를 잃으면 실제 모드는 Descend 등으로 달라질 수 있다. 제자리 착륙 보장은 아니다.

## 7. 종료와 실패 후 재시작

정상 종료는 실제 비행 중이라면 먼저 착륙하고 비무장을 확인한다.
그다음 swA Kill ON → B 브리지 Ctrl+C → A LIVO Ctrl+C 순서로 종료한다.
무장 중 B를 종료하는 동작은 FC EV 상실 대응을 유발할 수 있다.

FC guard가 latched=True이면 자동 재개하지 않는다. 지상·비무장·모터 정지 상태에서
`EKF2_BARO_CTRL=0` 복원을 확인하고, LiPo와 FC USB를 모두 분리해 FC 전원이 꺼졌는지
확인한 뒤 재부팅한다. 재부팅 후 guard latch 해제와 설정을 다시 확인한다.
Radxa 외부 전원은 유지해도 된다. SBC 브리지도 실패가 고정됐다면 지상에서 재시작한다.

현재 남아 있는 미검증 항목: 장시간 입력 지연 안정성, 실제 비행 중 착륙 성능,
수평 추정 상실 후 장시간 동작, 일부 조종기 인수 조건. 이 절차는 비행 승인서가 아니다.

### 간결한 상태판 (inspect_status)

```bash
ros2 run zav_real_control inspect_status
# 상세 원본 JSON이 필요할 때
ros2 run zav_real_control inspect_status --ros-args -p detailed:=true
```

기본 화면은 1초마다 갱신한다. 기체 모드·시동, FC 배터리 1의 전압/추정 잔량/경고,
odometry 수신 Hz와 측정값 지연(최근 2초), 브리지 감지 원인·기압계 대응,
EV 전송 조건과 최근 FC 경고 2개를 표시한다. 상태 토픽만 구독하며 추가 카메라/FC 연결을 열지 않는다.
배터리 MAVLink 수신은 기존 브리지 연결에서 1Hz로 요청한다.

`LIO/VIO 출력 Hz`는 브리지에 도착한 odometry 빈도이며 LiDAR/영상 보정 FPS나
알고리즘 연산 소요시간은 아니다. 마지막 측정값 나이는 입력이 끊겨도 증가한다.
배터리 데이터 3초, 브리지 상태 2초 이상 끊기면 현재 상태 미확인으로 표시한다.
잔량은 FC 추정치이며 전류 센서 교정 여부에 따라 정확도가 달라진다.

FC heartbeat의 failsafe 상태와 최근 STATUSTEXT 경고를 함께 표시한다.
`저배터리 경고 + Land 동시 관측`은 배터리 착륙의 의심 근거이며 확정 원인은 아니다.
수동 Land나 다른 실패가 동시에 발생할 수 있고 STATUSTEXT는 유실되거나 생략될 수 있다.
정확한 FC 착륙 원인과 FC 자체 EV 감시 고정 여부는 ULog/전용 진단으로 확인한다.
19.9V에서 착륙한 사실만으로 20.0V 고정 임계값을 단정하지 않는다.
이 화면은 비행 제어·배터리 임계값·failsafe 설정을 바꾸지 않는다.
