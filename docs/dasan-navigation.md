# Dasan 복도 주행

현재 실행 설정은 `src/zav_super/config/common.yaml`로 통합됐다. 내용은 Dasan 최종값 그대로이며 이전 파일은 `src/zav_super/config_archive/20261118/`에 보존했다. 아래의 예전 `dasan.yaml` 경로는 과거 기록이다.

현재 기본 경로는 116.83m 상층 왕복으로 확장했다. [확장 경로·시험 결과](dasan-long-route.md)를 먼저 참고한다. 아래의 43.74m 출발점과 회귀 기록은 이전 짧은 경로에 대한 내용이다.

FAST-LIVO2 복원 및 실제 카메라 색상은 [지도 안내](dasan-map.md)를 따른다. 현재 실행 경로는 전체 bag의 완주 경로가 아니라, 같은 복도 안에서 연결되고 기체 여유가 확보된 한 구간이다. 초기 외부 시작점과 외부를 거쳐 연결하던 855m 경로는 폐기했다.

현재 구현은 **상위 탐사 경로 → SUPER 재계획 → OMMPC → PX4**다. EPIC 자체는 아직 연결하지 않았으며, bag에서 얻은 다중 경유점 경로로 상위 입력을 대신한다. `interior-trial10/11`에서 외부 경로 반복 전송·주행 중 교체·복도 왕복·명시적 착륙을 확인했다. trial11은 정지 확인 후 다음 경로를 8초 기다리는 것도 확인했다. 두 실행의 TRACKING 구간에서 PhysX 충격/관통 보고는 0이며 진단 누락/오류는 없었다.

**통과와 추종 정확도는 다르다.** trial10의 실제 body와 bag 기준선 차이는 표본 최근접 거리 중앙값 0.32m, p95 0.87m, 최대 0.97m였다. 높이 차이가 주성분(p95 약 0.83m)이다. 따라서 정확한 bag 고도 추종이나 모든 환경에 대한 튜닝 완료로 표시하지 않는다. 실제 가속도/저크 피크도 설정의 계획 한계보다 클 수 있다. penalty/동역학 한계를 변경해 결과를 맞추지 않았으며, 실제 수치와 추정·제어 오차를 기록한다.

| 실행 | 경로 입력/종료 | 결과 |
|---|---|---|
| interior-trial05 | 파일의 30개 경유점 왕복 | 경로 진행 완료, PX4 AUTO_LAND 거부로 실패 |
| interior-trial06/07/08 | 외부 경로 + 교체 + 독립 yaw | 복귀/반전 부근 재계획 실패로 실패 |
| interior-trial09 | 제동 거리 preview | 구간 끝 도달, 대기 heartbeat FAULT로 실패 |
| interior-trial10 | 외부 경로 반복/교체, 명시적 착륙 | LANDED, 실제 최대 속도 5.17m/s |
| interior-trial11 | 정지 확인 후 hold, 8초 대기, 명시적 착륙 | LANDED, 실제 최대 속도 4.21m/s |
| interior-trial12 | 기본 파일 임무 | preview 축소 시 목표 후퇴와 추종 불안정 발견, 수동 중지 |
| interior-trial13 | 목표 arc 단조화 후 기본 파일 임무 | COMPLETED + LANDED, 실제 최대 속도 4.89m/s |

trial10 실제 a/j 피크는 약 7.00m/s² / 52.82m/s³, trial11은 약 10.92m/s² / 155.11m/s³다. 장착 IMU truth에서 body origin으로 변환하고 약 0.2초 Savitzky–Golay 필터로 계산한 norm이다. 계획된 최대값(6m/s², 10m/s³)과 실제 추종 결과의 차이가 남아 있다. 이 수치를 만족했다고 주장하지 않는다. `report.html`, `summary.json`, `kinematics.csv`, `samples.jsonl`, `contact-audit.json`이 각 실행 아래에 있다.


## 실행

터미널 두 개에서 순서대로 실행한다. 첫 터미널의 Isaac `[Ready] dasan` 이후 두 번째 명령을 실행한다. RViz는 navigation에서 자동으로 열린다.

```bash
./run_super_sim.sh dasan
```

```bash
./run_super.sh --scenario dasan --backend super_lio --profile common --execute
```

자동 이륙 없이 확인하려면 두 번째 명령에서 `--execute`를 뺀다. 별도 결과 디렉터리는 `--record-dir log/dasan-my-run`으로 지정한다. 기록에는 사용 설정·mission·추정/실제 기체 상태·최종 임무 상태와 HTML 보고서가 포함된다. 실제 기체 truth는 진단용이며 제어 입력으로 사용하지 않는다.

## 출발점과 경로

`simulator/assets/super/dasan_mission.json`이 Isaac의 출발점·패드와 navigation의 목표를 함께 정의한다. 지도 SHA256을 검증해 다른 지도에 오래된 경로를 사용하지 않도록 했다.

- 출발 body 위치: 약 `[5.558, -13.271, 0.560]`m. 이륙 목표 Z는 2.285m이다.
- 원본 bag 약 19.10–32.70초, 같은 복도 안의 연속 구간을 사용한다.
- 편도 약 22m, 왕복 기준선 약 43.74m이다. 원본 bag 시작점과는 약 15.75m 떨어져 있다.
- 원본 경로에서 가로·높이 각각 최대 35cm 조정하고, 원본 점에서 후보점까지 벽을 관통하지 않는지 확인했다. 끊긴 구간을 외부로 연결하지 않는다.
- 10cm 지형 voxel에 대한 보수적인 정적 중심 여유는 최소 0.55m다. 이 검사는 추정·제어 오차를 보장하지 않는다.
- 출발점의 양쪽 벽·바닥·천장과 수직 이륙 공간을 검사했다. 원본 카메라 영상에서도 복도 내부임을 확인했다.
- 여러 XYZ 경유점을 모두 입력받고 경로 진행 위치에서 앞쪽 목표를 보간해 계속 갱신한다. 중간점마다 감속/정지를 요구하지 않는다. 왕복의 진행 순서를 보존한다. RViz의 `/mission/global_path`는 bag 기반의 장애물 여유를 검사한 기준선이다. 실제 회피 궤적은 SUPER가 실시간 LiDAR 지도로 생성한다.
- 파일 임무의 최종 복귀점만 오차 0.15m, 속도 0.7m/s 미만에서 착륙을 요청한다. 외부 탐사 경로는 속도 0.3m/s 미만에서 controller의 명시적 hold 응답을 확인한 뒤 다음 경로를 기다린다. 60 시뮬레이션 초 동안 진전이 없거나 제어가 FAULT이면 ABORTED로 기록한다.

경로를 재생성하려면 다음 명령을 실행하고 simulator를 재시작한다.

```bash
source simulator/ros_setup.sh
python3 simulator/tools/super/dasan_route.py
```

원본 전체 경로의 계단·협소 구간·분리 구간은 이 임무에 포함하지 않는다. 사람 흔적은 영상과 여러 시각의 빈 공간 증거가 모두 있는 309점만 보수적으로 제거했다. 벽이나 불확실한 구조를 주행을 위해 삭제하지 않았다.

## 설정

별도 설정은 `src/zav_super/config/dasan.yaml`이다. 기존 `common_super_lio.yaml`과 `ommpc_high_speed.yaml`은 유지한다. penalty, `traj_opt` 전체, 최대 속도·가속도·저크, OMMPC 설정은 기존 common과 같다.

지도는 고정 원점의 전체 건물 지도가 아닌 45×45×18m 슬라이딩 지도다. 높이가 다른 환경에도 절대 바닥/천장을 가정하지 않으며, 실시간 LiDAR raycasting으로 점유를 갱신한다. 해상도 0.15m, inflation 3칸(0.45m), 기체 반경 0.4m를 사용한다. `p_miss=0.45`는 재방문 때 빈 공간 관측으로 오래된 점유를 갱신하기 위한 값이다. 3D 목표 Z를 보존하기 위해 `fsm.click_height=-10`을 사용한다. 기본 1.5m 고정값은 다른 층 목표를 덮어쓸 수 있다.

같은 설정으로 나중에 mixed를 비교할 때는 다음처럼 명시한다. 아직 mixed 재검증이 끝난 설정이나 모든 환경에서 동작한다고 보장하는 설정은 아니다.

```bash
./run_super_sim.sh mixed
./run_super.sh --scenario mixed --backend super_lio --profile common \
  --config src/zav_super/config/dasan.yaml --execute
```

## 실험 상태

`log/dasan-20261117/`에 원본 기록을 보존한다. `navigation-smoke`의 16m 로비 왕복 성공은 현재 복도 임무의 성공이 아니다. `navigation-trial01/02`는 외부 경로이므로 폐기했다. `interior-trial01`은 27개 중간점 중 18개 진행 후 복귀 중 재계획 실패·heartbeat FAULT·낙하 충돌이 발생했다. `interior-trial02`의 중간점 감속 실험도 실패했으며 해당 감속 정책을 철회했다. 둘 다 완주나 안전 착륙으로 기록하지 않는다.

`interior-trial03`의 끝점/복귀점 두 목표 실험도 복귀 중 heartbeat FAULT로 실패했다. 끝점만 남기는 방식을 철회했다. 경로 단순화가 왕복 반전점을 건너뛰던 결함도 수정해 편도 반전점이 기준선에 반드시 포함되도록 했다. `interior-trial04`는 다중 경유점 lookahead를 사용했지만, 실제 기체가 중간점의 높이 변화를 둥글게 통과하자 경로 진행 판정이 멈추고 60초 무진전으로 ABORTED가 됐다. PX4 LAND 진입도 5초 내 확인되지 않아 FAULT로 끝났으며 성공/안전 착륙으로 기록하지 않는다. 진행 판정을 전체 경로가 아닌 국소 전방 구간에 대한 투영으로 수정했고, 반전점 순서와 모서리 통과를 포함한 단위검사는 통과했다. 수정본의 실제 완주는 아직 검증하지 않았다.

EPIC의 시간 있는 위치·yaw 궤적은 이 경유점 추종과 다르다. [EPIC 연동 검토](epic-integration-review.md)에 실제 출력 형식과 연결 제약을 정리했다.

## EPIC 상위 경로를 대신하는 외부 입력

선택한 구조는 **탐사 경로 → SUPER 재계획 → OMMPC → PX4**다. 위치 다항식의 원래 시간표를 강제하지 않는다. ROS2 입력은 `/mission/exploration_path`의 `nav_msgs/Path`이며, 현재는 EPIC 본체를 실행하지 않고 bag 경로 발행기로 검증한다.

```bash
./run_super_sim.sh dasan
# 위 터미널의 Ready 이후 별도 터미널
./run_super.sh --scenario dasan --backend super_lio --profile common \
  --route-source external --execute
# ROS 환경을 불러온 또 다른 터미널
source simulator/ros_setup.sh
python3 simulator/tools/super/publish_bag_route.py \
  --replace-after-meters 8 --yaw-policy fixed --land-at-end --wait-at-end-seconds 8
```

마지막 명령은 시험용 상위 경로 발행기다. 현재 기본 bag 경로는 `yaw_waypoints=null`로 진행 방향 yaw를 사용한다. 아래 외부 입력 시험 발행기는 독립 yaw 입력을 검증하기 위해 고정 yaw를 지정하고, 약 8m 진행 후 현재 위치부터 남은 경로로 교체하며, 같은 경로도 반복 발행한다. `--land-at-end`는 **시험 발행기가 명시적으로** 착륙을 요청하는 옵션이다. 실제 탐사 입력을 받을 때는 이 발행기를 실행하지 않는다.

입력 계약:

- Path와 각 Pose의 좌표계는 `world`다(Pose frame은 빈 값도 허용). 시간 stamp는 발행 시각일 뿐 시간 궤적의 도착 시간으로 해석하지 않는다. ROS1 EPIC 측의 경로 추출/ROS2 전달은 아직 구현하지 않았다.
- 위치는 유한한 XYZ이며 서로 다른 두 점 이상이어야 한다. 단위 quaternion의 yaw를 목표로 사용한다. 전부 0인 quaternion은 yaw 미지정이다. 탐사 yaw를 속도 방향으로 덮어쓰지 않는다. 위치가 같은 yaw-only 회전 구간은 별도 명령 계약이 필요하다.
- 동일 위치/yaw 배열의 재발행은 진행을 초기화하지 않는다. 다른 새 경로는 현재 기체 위치 2m 이내에서 시작해야 한다. 오래된 전체 경로를 처음부터 다시 따라가지 않도록 발행기가 현재 위치부터 경로를 제공한다.
- 새 경로는 기존 실행 궤적을 controller에 직접 덮어쓰지 않고 SUPER의 새 목표로 전달해 재계획한다. 기본 preview 4m에 현재 속도의 jerk 제한 제동 거리와 0.5초 응답 여유를 더하되 planner horizon 15m 안으로 제한한다. 계산에는 YAML의 기존 max_acc/max_jerk를 읽어 사용하며 한계값 자체를 바꾸지 않는다. 실제 왕복 반전점은 건너뛰지 않는다. 진행점은 국소 전방 구간에 투영하며, 왕복/교차 경로에서 전체 최근접점을 검색하지 않는다.
- 경로 끝에서 위치 오차 0.15m/속도 0.3m/s 미만을 확인하고 `/zav/super/hold` 성공 응답을 받은 뒤 `WAITING_PATH`로 다음 탐사 경로를 기다린다. hold는 신선하고 정지한 비행 피드백이 있을 때만 수락되며, 이동 중 heartbeat 검사/FAULT를 무시하는 기능이 아니다. 외부 경로의 끝을 전체 탐사 종료로 취급하거나 자동 착륙하지 않는다. 착륙은 `/zav/super/land` 서비스의 명시적인 요청이다.
- `super_planner.goal_yaw_en=true`가 필요하다. SUPER가 위치와 함께 yaw를 재계획한다. 경로 입력은 공간상의 목표이며 EPIC 원래 yaw 시간표를 그대로 추종하는 기능은 아니다.
- 점유된 목표의 보정은 Dasan에서 요청점으로부터 최대 0.3m다. 매번 보정된 점을 다시 보정해 목표가 누적 이동하던 동작을 수정했다. 새 장애물 때문에 원래 경로를 그대로 지나갈 수 없으면 SUPER의 실시간 회피가 우선한다.

현재 구현은 경로상의 lookahead 목표로 SUPER를 유도한다. 전체 polyline을 최적화기의 강제 경유 조건으로 넣은 기능은 아니며, 각 정점을 정확히 통과한다는 보장은 없다. 이 경계와 실제 통과 오차를 구분해서 검증한다.

추가 실험: `interior-trial05`는 30개 경유점/43.74m의 진행을 완료했지만 AUTO_LAND 거부로 전체 임무는 실패다. `interior-trial06`은 외부 경로 반복 전송·주행 중 suffix 교체(경로 revision 2)와 독립 yaw 입력을 실행했고, 복귀 끝 부근에 목표 보정/재계획 실패·heartbeat FAULT로 끝났다. EKF EV 측정 잡음 모드 0 시험도 착륙 문제를 해결한 것으로 채택하지 않는다. ULog에서 초기 정지 상태부터 PX4 추정 roll이 약 180°였음을 확인했으며, 초기화 원인을 조사 중이다. body truth를 제어에 넣거나 PX4 검사 기능을 끄지 않았다.


## 후속 수정과 회귀검사

초기 PX4 roll이 180°인 상태에서 EKF만 물리 안정 후 재시작하면 0°가 됐다(`interior-trial07/ekf-before.txt`, `ekf-after.txt`). ZAV_v1은 첫 HIL 센서 전달을 1초 늦춰 물리 spawn 접촉이 안정된 실제 센서부터 전달한다. 합성 가속도나 truth feedback을 넣지 않으며, 이후 lockstep timestamp 간격은 그대로다. 새 실행의 초기 roll 0°는 `interior-trial08/ekf-startup.txt`에서 확인했다. 다른 PX4 차량의 기본 backend 지연은 0이고 ZAV_v1 프리셋에서 1초를 지정한다.

명시적 탐사 yaw가 있을 때 SUPER 내부 yaw 중간점을 속도 방향으로 만들던 동작도 수정했다. 명시된 yaw로 최단 각도를 따라 보간하며, yaw 미지정 시에는 기존 진행 방향 정책을 사용한다. yaw 속도 제한이나 penalty는 변경하지 않았다. 전진/후진 독립 yaw와 ±π 연결을 실제 yaw optimizer C++ 검사로 확인한다.

`interior-trial08`은 초기 roll은 정상화했지만, 빠르게 접근한 복도 반전점 부근의 재계획 실패로 끝났다. 고정 4m preview 대신 제동 거리 기반 preview를 적용해 `interior-trial09`에서 다시 검증한다. 실패 기록을 성공으로 합치지 않는다.

```bash
source simulator/ros_setup.sh
colcon test --packages-select zav_super super_planner --event-handlers console_direct+
python3 simulator/tests/check_px4_lockstep.py
```

preview가 줄어도 이미 발행한 목표가 경로상의 뒤쪽으로 후퇴하지 않도록 수정했다. 왕복의 반전점을 넘어가는 실제 방향 전환은 허용하며, 진행 arc와 목표 arc는 각각 단조롭게 유지한다. 수정 후 기본 파일 임무는 `interior-trial13`에서 COMPLETED + LANDED로 확인했다. TRACKING 구간의 충격/관통 보고는 0이며 callback 오류와 누락도 0이다. 실제 a/j 피크는 약 5.92m/s² / 50.85m/s³다. 실제 body와 bag 기준선 차이는 p95 약 0.85m, 최대 약 1.09m이며 대부분 높이 차이다. LIO와 실제 body의 차이는 p95 약 0.19m이므로 기준선 차이를 모두 LIO 오차라고 해석하지 않는다. 회피 경로와 제어 추종의 기여를 더 분리하려면 실제 수신 다항식 기록이 필요하다. 기존 recorder의 reliable QoS 때문에 trial13에는 다항식 기록이 없었으며, 이후 sensor-data QoS로 수정했다.
