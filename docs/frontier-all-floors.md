# Frontier 층별 복도 방문 후 최상층 종료

요청한 순서는 **계단으로 한 층 상승 → 해당 층의 bag 복도 구간 방문 → 다음 층 상승**이다. 최상층에 도달하면 그곳에서 멈추고 착륙한다. 출발층으로 내려가는 복귀 경로는 없다. 복도 자체는 선택한 bag 복도 구간을 방문한 뒤 계단 쪽으로 돌아오는 형태이며, 이것을 삭제하는 계단 전용 경로와 구분한다.

현재 기본 경로는 **500.54m, 203개 일반 경유점, 하나의 연속 상위 Path**이다. 186m 축소 경로 대신 네 층의 기록된 긴 복도를 끝까지 방문하는 경로로 복원했다. trial30·31의 두 번 연속 실행에서 이 전체 경로를 203/203 경유점까지 완료하고 최상층 자연 바닥에 PX4 LANDED로 착륙했다. 중간 대기와 최종 착륙 전까지의 navigation 충격·관통은 모두 0건이다. 기록은 `log/frontier-20261118/completion-fix/trial30-acquisition-clock/completion-audit.json`에 있다. 출발점은 이전 검증된 계단 가까운 평지 `[-3.26924,-0.05305,0.16]`m, 이륙 목표 Z는 1.64114m다. bag 약 83.20초부터 최상층 관측 지점까지의 위치 기록을 바탕으로 한다. 기록된 7개 높이 구간의 목표 body Z는 대략 1.64, 5.26, 9.28, 13.28, 17.26, 21.21, 24.18m다. 실제 건물의 층 명칭을 추정한 숫자는 아니다. 상층부에서 bag이 계단참만 관측한 구간은 그 관측 경로만 방문하며 비관측 복도를 새로 만들지 않는다.

`frontier_mission.json`의 `return_to_start=false`, `stairs_only=false`, `continuous_waypoints=true`, `viewpoint_pause_indices=[]`가 현재 형태다. 중간 일반 경유점·계단참·윗층 도착·복도 코너를 모두 같은 Path 안에서 연속 통과한다. `corridor_trims=[]`이며 복도 길이를 자르지 않는다. `allow_blocked_loop_skip=false`로 복도 끝을 생략하는 우회 정책도 비활성화했다. 중간 강제 정지와 3초 대기는 제거했다. 최상층 마지막 끝점에서만 도착·정지를 확인한 뒤 시험 발행기가 PX4 LAND를 명시적으로 요청한다.

## 설정과 경로

`src/argo_super/config/common.yaml`은 확장 전과 바이트 단위로 같고 SHA-256은 `a757c64015713ec4107769578ed55661c0b40eb5a011a9c46fac090c5f38bea9`다. penalty, 최대 속도·가속도·저크, 지도/LIO/OMMPC 값은 바꾸지 않았다. 오프라인 경로는 손-held 센서 위치에 밀착하기보다 같은 복도의 여유가 큰 중앙을 선호하고 연속성을 고려해 선택했다. 이것은 비행 YAML의 trajectory penalty 변경이 아니다. 지도·카메라 색상·충돌 형상도 그대로다.

모든 최종 경로 선분은 같은 PCD 복셀에서 4cm 간격으로 최소 중심 여유 0.55m를 검사했다. 입력 생성의 6cm 샘플링과 최종 검사 사이에서 드러난 약 14μm 부족은 해당 경유점을 1mm 옮겨 재검사했으며 기준을 낮추지 않았다. 이 정적 여유가 실제 추종 오차까지 보장하는 것은 아니다.

최상층 종료점은 대략 `[-0.9829,3.61785,24.1817]`m다. 아래에서 평평한 자연 바닥 95개 점을 확인했으며 중앙 Z는 약 22.854m, 10~90백분위 높이 차이는 약 6.4mm다. 최상층에 새 가상 바닥이나 인공 플랫폼을 추가하지 않았다. PX4의 실제 landed·disarmed 피드백으로 착륙을 확인한다.

경로 소비자는 `/mission/exploration_path`의 world-frame `nav_msgs/Path`를 사용한다. XYZ와 yaw quaternion을 보존하며, EPIC 자체/ROS 1 polynomial bridge를 연결한 것은 아니다. 막힌 왕복 복도를 생략하는 시험 정책은 현재 비활성화했다. 일반 EPIC 경로 소비자는 온라인 LIO 세계 좌표의 XYZ/yaw를 그대로 받는다.

## 저장 bag 경로와 온라인 LIO 좌표 정합

저장된 bag 경로는 오프라인 지도 좌표이고 비행 제어와 SUPER는 온라인 Super-LIO 좌표를 쓴다. 긴 왕복 후 두 좌표가 어긋나면 지도에서 안전했던 고정 경유점도 온라인 점유 지도에서는 벽 안으로 판단될 수 있다. trial11에서는 복도 왕복 후 약 0.9m 위치 차이가 발생했다. 이 차이를 정답 pose로 제어하거나 복도를 줄여서 감추지 않는다.

`bag_map_alignment=true`인 bag 시험 발행기만 `/argo/super/cloud_world`의 실제 LiDAR 관측을 저장 PCD와 point-to-plane ICP로 정합한다. 온라인/저장 지도 좌표의 기울기 차이를 포함한 회전·평행이동(6DoF)을 구하고, 다음 XYZ/yaw 경로를 현재 LIO 좌표로 변환한다. 제어 odometry는 바꾸지 않으며 저장 PCD를 ROG-Map에 주입하지 않는다. 참조 지도 해시는 임무와 일치해야 한다. 정합은 충분한 평면 관측, 관측 가능성, 잔차·inlier·보정 크기 검사를 통과해야 한다.

이전 분할 Path 모드에서는 viewpoint 정지 시 정합했다. 현재 연속 모드는 초기 정합 후 이동 중에도 약 1초 간격으로 실제 점군을 정합한다. `/mission/bag_frame_correction`으로 기존 경로에 강체 좌표 변환만 적용하며, 경로 revision·segment·누적 진행 거리·preview 진행을 초기화하지 않는다. 새 Path는 보내지 않는다. 추가로 5초 이상 진전 없이 실제 정지한 경우에는 남은 순서 경로를 복구 정합할 수 있다. 이동 중 주기적으로 새 Path를 발행하는 동작은 되돌아감 회귀 때문에 제거했다. 활성 경로의 순서·진행 상태는 비행 중 유지한다. 이미 지난 segment만 제외하며 복도 끝·되돌아오는 가지·다음 계단의 모든 미방문 경유점을 보존한다. 정지 정체 복구는 구간당 최대 세 번이며, 해결되지 않으면 기존 60초 no-progress 착륙 보호가 동작한다. `aligned_remaining_path_republished` 로그에 남긴다. 이는 복도를 생략하는 기능과 다르다.

EPIC는 같은 온라인 지도 좌표로 viewpoint를 생성하므로 이 bag 참조 정합기가 필요하지 않다. 공통 `/mission/exploration_path` 계약은 그대로다. 실제 EPIC 노드/ROS 1 polynomial bridge를 연결한 시험은 아니다.

## 계산 오류 수정

- ROG-Map의 근처 셀 탐색은 격자 중심 기준으로 이웃을 정렬해 놓고, 실제 목표와의 거리가 반경 밖인 후보 하나를 만나면 전체 탐색을 종료했다. 목표가 격자 중심에서 벗어나면 이후의 유효한 반경 내 후보를 놓쳤다. 실제 거리 반경을 확인하고 삼각부등식으로 안전하게 탐색 범위를 제한한다. 입력·출력이 같은 벡터인 호출도 보존한다.
- 실패한 보정 출력은 NaN인데 FSM이 그 결과를 검사하지 않아 A* 끝점에 NaN을 전달했다. 실패 시 마지막 유한 목표를 유지하면서 충돌 회피 재계획은 계속한다.
- 목표가 일시적으로 거절됐고 기체가 정지해 있을 때는 같은 click goal을 1초 간격으로 다시 보낸다. 움직이는 동안 불필요하게 목표를 재발행하지 않고, 완료된 viewpoint hold를 해제하지 않는다.

- ROG-Map ROS 2 odometry callback이 위치·자세만 받고 선속도를 저장하지 않았다. 상태 벡터 초기화를 명시하고 world-frame 선속도를 저장해 실제 정지 판단과 guide-path 속도에 사용한다.
- 계획기가 완료 후 WAIT_GOAL에 있어도 기존 정지 궤적을 실행하는 동안 heartbeat를 보낸다. 종료된 zero-v/a/j 참조, 0.1초 이내 피드백, 끝점 거리 0.3m 이내, 온라인 끝점 비점유를 확인한다. 제어기의 heartbeat 1초, 독립 terminal speed 0.3m/s, freshness 보호는 그대로다.
- 남은 경로 길이가 braking lookahead보다 짧다는 이유만으로 동일한 모서리 목표를 매 tick 재발행하던 동작을 수정했다. 30cm 미만의 마지막 갱신도 정확한 끝점은 즉시 한 번 발행한다.

근처 셀의 반경 제한과 임무 정체/heartbeat/상태 보호를 완화한 것은 아니다. 이 수정은 특정 층의 예외 코드가 아니라 공통 탐색·경로 전달 코드에 적용했다.

## 시험 기록

기록 루트는 `log/frontier-20261118/all-floors/`다.

| 실행 | 경로 | 결과 |
|---|---|---|
| trial01-full | 575m, 상승 후 지상 복귀 후보 | 출발층에서 두 층 위 긴 복도에서 NaN 끝점/목표 정체. 충격·관통 없이 안전 착륙했으나 전체 실패. |
| trial02-recovery | 같은 긴 후보 + 보정 수정 초기판 | 복도 진입 중 충돌·heartbeat FAULT. 실패. |
| trial03-centered | 588m, 중앙 여유·모서리 정지, 지상 복귀 포함 | 상층 계단에서 충격 2건, 관통 0. 그 뒤 긴 복도 재계획 중 heartbeat FAULT. 실패. |
| trial04-ascent-only | 87m, 복도 제외 계단 전용 | 7개 관측 높이 도달, 최상층 자연 바닥 착륙, 공중 navigation 충격·관통 0. 계단 동작 검증이며 최종 요청인 층별 복도 방문을 대체하지 않는다. |
| trial05-floor-corridors | 500.54m, 각 층 복도 유지·최상층 종료 | 두 번째 상층 복도에서 접촉 후 heartbeat FAULT. 전체 실패. |
| trial06-corridor-observations | 500.31m, 복도 정지 관측점 추가 | 충격·관통 0이나 감속 중 계획기 완료 처리로 heartbeat FAULT. 실패. |
| trial07-corridor-settling | 동일 복도 경로 + 정지 완료 처리 초기판 | 접촉 0이나 첫 복도 진입 재계획 지연 중 heartbeat FAULT. 실패. |
| trial08-short-corridors | 234m, 먼 복도 일부 제외 | 첫 복도 방문 뒤 다음 계단 회전에서 벽 접촉 후 FAULT. 실패. |
| trial09-compact-corridors | 186.12m, 각 층 짧은 복도 방문 유지 | 26/26구간, 7개 관측 높이 도달, 최상층 LANDED. navigation 충격·관통 0. 현재 요청보다 복도가 짧아 기본값에서 제외. |
| trial10-full-length | 500.54m 복원, 생략 금지 | 첫 상층 도착 시 완료 대기 heartbeat 만료. 접촉 0, 실패. |
| trial11-terminal-liveness | 같은 긴 경로, 명령/속도 전달 수정 | 첫 복도 끝 왕복 완료 후 계단 목표가 점유로 거절됨. 60초 정체 후 안전 착륙, 접촉 0. |
| trial12-bag-map-alignment | 같은 긴 경로, 정지 viewpoint 정합 | 첫 복도 끝 왕복과 다음 층 상승 통과. 두 번째 복도 Y32m 부근에서 주행 중 좌표 차이로 정체, 안전 착륙. 접촉 0. |
| trial13-live-frame-recovery | 같은 긴 경로, 정지 정체 시 남은 경로 재정합 | 두 번째 복도에서 높이 오차가 커져 바닥 접촉. 실패. |
| trial14-rigid-live-alignment | 같은 긴 경로, 6DoF 정합 및 이동 중 남은 경로 갱신 | 앞선 세 복도 끝에 실제 도달. 세 번째 복도 귀환 중 접촉, backup 생성 실패 후 heartbeat FAULT. 14/26구간 완료, 최상층 실패. |
| trial15-reproduce | 동일 경로 | 첫 복도 끝 도달 전3.27m 되돌아감 재현. |
| trial16-fresh-progress | 최신 피드백 처리 초기판 | 첫 계단 접촉·FAULT로 실패. |
| trial17-stable-path | 이동 중 Path 재발행 제거 | 첫 복도 전진 역행0m. 두 번째는 로컬 polynomial의2.25m 후퇴가 남음. 세 번째 복도 귀환 접촉 후 LIO safety FAULT,14/26구간, 최상층 실패. |
| trial18-no-scripted-stops | 500.54m,203개 경유점의 연속 Path1개 | 첫 윗층/코너 정지 없이 통과. 두 번째 복도 접촉 후 heartbeat FAULT,65/203경유점, 최상층 실패. |

이전 분할 Path 모드의 trial14 기록은 `long-corridors/trial14-rigid-live-alignment/`다. `evaluation.json`의 `finite_ascent_complete=false`, `full-corridor-audit.json`의 `full_route_test_complete=false`가 결과다. 네 복도 끝 중 앞선 세 지점과 실제 body 위치의 최소 거리는 각각 약 **0.419m, 0.082m, 0.045m**였다. 네 번째 복도와 최상층에는 도달하지 못했다. 복도 길이 생략·왕복 루프 건너뛰기 이벤트는 0건이다.

첫 충격은 simulation 326.07초, 세 번째 복도 끝을 지난 귀환 Y37.76m 부근에서 발생했다. 이후 backup polytope 생성이 실패하고 341.13초에 controller heartbeat FAULT가 발생했다. navigation 구간 충격 보고 1,455건, 관통 보고 73건으로, 접촉 이후 반복 보고를 포함한다. 이후 주행 중 Path 재발행이 진행 초기화와 되돌아감을 유발하는 회귀도 확인해 제거했다. 자세한 기록은 [되돌아감 진단](frontier-backtracking.md)을 참고한다. 정합만으로 추종 안정성이 해결됐다고 할 수 없다. 해당 조건에서 최상층 완주를 검증한 결과는 없다.

이전 축소 경로 결과는 `trial09-compact-corridors/evaluation.json`의 `finite_ascent_complete=true`, `all_observed_levels_reached=true`, `completed_chunks=26`, `final_controller_state=LANDED`로 확인했다. 실제 최고 body Z는 24.194m, 최종 착륙 body Z는 22.995m였다. 출발층으로 내려오지 않았다.

이 성공은 먼 복도를 제외한 186m 경로 한 번의 결과다. 500m 전체 복도의 완주나 모든 환경의 안정성을 보장하지 않는다. 최상층 대기 시 실제 body XY는 목표와 약 0.6m 차이가 있었으며, 지도·LIO의 절대 위치 오차 문제는 남는다. 실제 속도·가속도·저크 한계 만족을 이 결과로 주장하지 않는다.

현재 연속 경로의 전체 완료는 `completion-audit.json`에서 원본 길이·203개 경유점·네 복도 끝·일곱 높이 구간·충격/관통·명시적 LAND·PX4 LANDED를 함께 검사한다. 이전 분할 경로의 `evaluation.json`은 당시 구간 수를 전제로 하므로 현재 판정에 그대로 쓰지 않는다. 외부 탐사 경로 소비자는 마지막에도 `WAITING_PATH`이므로 범용 그래프의 `mission_complete=false`를 EPIC 전체 탐사 완료로 바꾸지 않는다. 최종 시험의 모든 chunk 도착, 모든 층 높이 도달, 명시적 LAND 및 PX4 LANDED를 함께 검사한다.

## 실행 및 재생성

```bash
./run_super_sim.sh frontier
# [Ready] 이후 다른 터미널
./run_super.sh --scenario frontier --execute
```

전체 실측 복원은 [기존 Frontier 문서](frontier-navigation.md)를 따른다. 동일 복원 데이터에서 현재 경로는 다음 순서로 만든다.

```bash
source simulator/ros_setup.sh
python3 simulator/tools/super/dasan_route.py --scenario frontier \
  --output log/frontier-20261118/all-floors/frontier_centered_candidate.json \
  --anchor log/frontier-20261118/all-floors/baseline-mission.json \
  --clip-start --start-bag-seconds 82 --minimum-length 20 \
  --lateral-limit .65 --vertical-limit .6 --clearance .55 --connect-gaps \
  --offset-weight .2 --smoothness-weight 2 --yaw-policy path-heading
python3 simulator/tools/super/frontier_all_floors_route.py \
  --candidate log/frontier-20261118/all-floors/frontier_centered_candidate.json \
  --output log/frontier-20261118/all-floors/frontier_full_corridors.json \
  --turn-exit-holds --no-return --align-live-frame
python3 log/frontier-20261118/all-floors/apply_top_landing.py \
  log/frontier-20261118/all-floors/frontier_full_corridors.json
```

기본 생성은 중간 강제 정지 없는 연속 경로다. `--stop-at-viewpoints`를 명시할 때만 이전 정지 관측 모드로 생성한다. `--ascent-only`는 복도까지 빼는 별도 계단 시험 옵션이고 현재 요청의 기본 경로에는 사용하지 않는다. 위 기하 경로에서 `bag_map_alignment=true`를 설정한 JSON과 같은 NPZ를 `simulator/assets/super/frontier_mission.*`으로 반영했다. 같은 긴 파일은 `frontier_mission_all_floors.*`와 `frontier_mission_full_corridors.*`로 보존했다. 이전 186m 성공 경로는 `frontier_mission_compact_corridors.*`에 그대로 보존했다. 500m 복도 후보는 `frontier_mission_floor_corridors.*`, 234m 실패 후보는 `frontier_mission_short_corridors.*`, 87m 계단 전용 성공은 `frontier_mission_stairs_only.*`로 구분한다. 다른 출발점으로 변경할 때는 navigation과 Isaac을 종료하고 두 파일을 함께 바꾼 후 다시 시작한다.


## 중간 강제 정지 제거

사용자 요청에 따라 trial18부터 기존25개 `viewpoint_pause_indices`를 모두 비우고, 203개 경유점을 하나의 Path로 전달한다. 원래 관측점은 `observation_candidate_indices`로 기록만 남긴다. `viewpoint_wait_seconds=0`이며 길이500.53956m·XYZ/yaw 경로와 NPZ는 변경하지 않았다. 구간 마지막 속도0/hold를 제거하려고 단순 대기시간 변경 대신 Path 분할 자체를 제거했다. 정지점이 없어도 bag-map 정합 발행기가 실행되도록 launch도 연속 모드를 지원한다.

trial18에서 첫 윗층 도착점(vertex9)은 실제 body 속도0.795m/s, 복도 진입 코너(vertex12)는0.442m/s로 TRACKING 상태에서 통과했다. 두 지점 주위에0.5초 이상 실제 정지 구간은 없었다. 결과는 `log/frontier-20261118/continuous-waypoints/trial18-no-scripted-stops/continuous-audit.json`이다. 전체 층 완주 판정과 이 두 지점의 통과 판정은 구분한다. trial18 전체 시험은 두 번째 복도에서 접촉 후 sim96.29초에 heartbeat FAULT로 중단됐다. 당시203개 중65개 경유점 진행 상태였으며 최상층에는 도달하지 못했다. navigation 충격 보고493건·관통 보고13건으로 접촉 이후 반복 보고를 포함한다. 실제 장애물 대응·최적화 실패에 따른 정지/후퇴는 스크립트 강제 정지와 별개의 문제다.

현재 연속 프로필은 `frontier_mission_continuous.*`, 이전 정지 프로필은 `frontier_mission_viewpoint_stops.*`에도 보존했다. common YAML은 바이트 단위로 유지했고 임무 회귀21개가 통과했다.


## 현재 연속 경로의 전 구간 검토 (trial19·20)

같은500.54m/203경유점/중간정지0개와 동일common YAML로 Isaac·RViz를 켜고 두 번 더 실행했다. 전 구간 무충돌 완주는 두 실행 모두 실패했다. trial19는44.56초·경로진행55.5m에서 충격·관통 없이 백업 궤적/heartbeat 만료로 중단됐다. trial20은85.145초 두 번째 상층 복도 진입에서 최초 접촉이 발생했다. 최초 충돌 후1초 문맥을 수집한 뒤 진단 발행기가LAND를 요청했다. 최종LANDED는 전체 경로 완료가 아니다.

trial20 최초 접촉에서 SUPER 참조와 실제body는0.988m,추정body와 참조는0.845m,초기 지도 정합 좌표의 추정↔실제body는0.203m 차이였다. 실제 기체가 참조를 벗어나 모서리에 닿은 것이 확인되며 위치 추정 오차만으로는 설명되지 않는다. 충돌0.1초 전에도 실제↔참조 오차0.798m,실제/참조 X속도-3.94/-1.85m/s였다. 당시 일반 궤적 flag1이었다.

83.715초 재시작은 초기속도0인데 측정속도2.255m/s였고 참조 위치 전환 폭은1.071m였다. PlanFromRest의0속도 초기화와 이동 중 재시작은 검토 대상이나,유일 인과관계는 아직 확정하지 않았다. 제어·계획 로직과 파라미터는 이번 검토에서 변경하지 않았다.

상세 보고서: `log/frontier-20261118/full-route-review/review.html`. 그래프: `trial20-repeat/collision-analysis.png`,원자료: 각 trial의 `review-analysis.json`,완주 판정/불변 검사: `review-summary.json`. 진단에는 truth를 읽었지만 제어에 주입하지 않았다.


## 전 구간 충돌 원인 수정 및 완주 검증

검토 후 원인을 수정한 기록은 `log/frontier-20261118/completion-fix/`다. 같은 500.53956m/203경유점 경로와 바이트 단위로 같은 common YAML을 유지했다.

- 물리 위치·속도는 200Hz 상태인데 자세는 렌더용 USD에서 읽어 약 50ms 늦었다. Vehicle의 자세도 같은 Dynamic Control 물리 pose에서 읽는다. Mounted IMU가 이 일관된 물리 상태로 specific force와 회전율을 생성한다.
- Motion BVH가 꺼져 움직이는 LiDAR의 GMO 시작·끝 자세가 같았다. `SimulationApp(enable_motion_bvh=True)`로 실제 이동 중 점별 시각과 센서 움직임을 모델링한다. 이 설정은 [Isaac Sim 5.1 공식 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx.html#motion-bvh)의 기능을 사용한다.
- 렌더 콜백에서 읽은 GMO는 이전 완료 프레임인데 현재 전달 시각으로 RTX 시계를 변환했다. 이전 렌더 콜백의 물리 시각을 보존해 해당 프레임에 연결하고, 점별 실제 획득 시각을 유지한다. float32 시계 드리프트도 따라간다. 고정 30ms/50ms 튜닝값을 넣지 않았다. 수정 전 원본 점군의 지형 적합 시간 편향은 약 -30ms, Motion BVH만 켠 중간판은 -50ms였고 trial30 수정판은 대부분 0±10ms였다. 지형/ground truth 대조는 오프라인 진단에만 썼다.
- Super-LIO가 LiDAR 끝 시각과 다음 IMU 샘플 시각 사이의 적분을 누락하고 재전파 시각만 끝 시각으로 옮겼다. 경계 오른쪽 IMU를 큐에 유지한 채 보간하고, 실제 적분된 시각·샘플을 저장한다. 일정 회전 재현 테스트에서 수정 전 1초에 약 0.018rad 누적 누락을 확인했고 수정 후 테스트가 통과했다.
- 움직이는 상태의 PlanFromRest는 최신 실제 위치·속도로 초기화한다. 빈 공간의 실제 위치를 격자 중심으로 옮기거나 측정 속도를 0으로 바꾸지 않는다.
- 최적화가 같은 ROS 실행기를 오래 점유하면 이미 확정된 궤적의 heartbeat까지 밀렸다. 별도 실행기가 신선한 odometry와 FOLLOW_TRAJ 상태, 유효한 정지 끝점 궤적의 종료 시각 안에서만 heartbeat를 발행한다. 궤적 연장, 만료된 이동 궤적 인증, 제어기 1초 timeout 변경은 하지 않는다.

trial30은 simulation 275.405초에 203/203 경유점을 완료했고, 284.585초에 LANDED를 확인했다. navigation 검사 구간은 8.360~278.860초로 끝점 정지 대기도 포함한다. 충격·관통 0건, 복도 끝 실제 body 최소 거리는 순서대로 0.100/0.167/0.259/0.194m다. 7개 높이 구간과 최상층 종료점에 모두 도달했으며 최종 body Z는 22.995m로 지상 복귀가 아니다. 정답 pose는 센서 시각/실행 결과의 오프라인 검증에만 사용했다.

검증: Super-LIO ctest 2/2, SUPER ctest 2/2, argo_super ctest 2/2(임무 회귀 22개 포함), 물리 상태/IMU 수학/실제 센서 발행 및 획득 시계 회귀, 원본 지도 해시와 0.55m 정적 경로 여유 검사가 통과했다. 센서 시뮬레이션·계획/경로 전달 코드를 수정했으며 penalty·최대 속도/가속도/저크·LIO/지도/OMMPC YAML 값과 지형 형상은 변경하지 않았다.


최종 코드에서 디버그 출력을 제거하고 Isaac·RViz를 새로 실행한 trial31도 전체 완료했다. 272.205초에 203/203 경유점, 281.785초에 최상층 LANDED, navigation 7.810~275.980초 충격·관통 0건이다. 복도 끝 최소 body 거리는 0.151/0.137/0.281/0.148m다. 두 번의 완주 모두 단일 Path revision1을 유지했고, source chunk 생략이나 중간 hold는 없었다. 최종 통합 보고서는 `log/frontier-20261118/completion-fix/verification.html`, 기계 판정은 `verification-summary.json`이며 `verified_full_runs=2`, `complete=true`다. 첫 성공을 특정 실행의 초기 시계 위상에만 의존한 결과로 두지 않고 재시작 후 반복 검증했다.
