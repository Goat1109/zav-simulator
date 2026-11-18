# Dasan 확장 경로 시험

현재 기본 경로는 **116.83m 왕복, 58개 경유점**이다. 이전 43.74m 직선 왕복보다 약 2.7배 길고, 서로 다른 코너 두 곳을 왕복으로 통과한다. bag의 상층 약 **262.29–302.59초** 구간을 사용한다. 먼저 기록의 역방향으로 진행하고 같은 길로 돌아와 기록 방향을 따른다. 계단이나 다른 층으로 순간 이동하는 연결은 없다.

2026-11-18부터 기본/medium 경로는 `yaw_waypoints`를 모두 `null`로 두어 SUPER가 재계획한 궤적의 진행 방향으로 기수를 돌린다. 출발 yaw는 이륙 초기 자세에만 사용하며, EPIC 등 외부 경로가 명시적으로 주는 yaw는 계속 존중한다. 아래 trial04 완주 기록은 이 변경 전 고정 yaw 시험이다.

변경 후 Isaac/RViz 재시험(`log/dasan-20261118/yaw-forward/`)도 **116.83m 완주 + LANDED**였다. TRACKING 중 PhysX 충격·관통은 없었다. 기수가 진행 방향에 따라 회전함을 body truth로 확인했지만, 1m/s 이상 이동 시 진행 방향과의 오차 중앙값은 7.42°, 95백분위는 77.68°로 코너/반전 지연은 남는다. 순간적으로 진행 방향과 일치하는 보장은 아니다. 기존 yaw 설정(0.8rad/s), penalty 및 주행 한계는 변경하지 않았다. 감사 결과는 `yaw-audit.json`, `contact-audit.json`, 그래프는 `flight/report.html`에 보존했다.

출발점은 해당 상층 복도 내부 `[5.543, 31.076, 4.360]`m, 이륙 목표 Z는 `6.072`m다. **기존 초반층 출발점과 층·위치가 다르다.** 바닥 지지면·천장·양쪽 벽과 이륙 공간을 확인한 지점을 사용했다. 원본 지도/카메라 색상은 바꾸지 않았고, 추가 장애물 삭제도 하지 않았다. 출발 패드는 해당 위치에만 둔다.

## 실행

```bash
./run_super_sim.sh dasan
# Isaac의 [Ready] 이후 별도 터미널
./run_super.sh --scenario dasan --backend super_lio --profile common --execute
```

현재 `simulator/assets/super/dasan_mission.json`이 117m 경로를 가리킨다. 온라인 `dasan.yaml`의 모든 값은 확장 전과 동일하다. penalty, 속도·가속도·저크 한계, OMMPC 설정도 그대로다. 비교 중 사용한 Super-LIO 해상도 변경은 효과가 없어 되돌렸다. 감사 기록은 `log/dasan-20261117/long-route/parameters-audit.json`에 있다.

## 검증 결과

Isaac Sim과 RViz를 모두 켜고 시험했다. 기록은 `log/dasan-20261117/long-route/`에 있다.

| 실행 | 경로/추정기 | 결과 |
|---|---|---|
| trial01 | 306.23m / 기존 Super-LIO | 약 89m 진행 후 바닥 접촉·disarm. 추정 높이와 실제 body 높이 차이가 약 1.3m까지 증가했다. 실패다. |
| trial02 | 같은 306m / 점군 voxel 0.2m 비교 | 약 81m에서 재계획/backup 실패, heartbeat FAULT. 높이 차이 약 1.6m. 채택하지 않았다. |
| trial03-fastlivo | 같은 306m / FAST-LIVO2 비교 | 약 24m 이후 추정이 무너져 실패. 채택하지 않았다. |
| trial04-medium | **116.83m / 기존 Super-LIO 설정** | **COMPLETED + LANDED**. TRACKING 중 충격/관통 보고 0, contact callback 오류/누락 0. |

trial04의 시뮬레이션 body 최대 속도는 약 **6.73m/s**, a/j 피크는 약 **7.67m/s² / 50.21m/s³**다. 약 0.2초 Savitzky–Golay 필터를 쓴 실제 body norm이며 계획 한계와 구분한다. 한계 파라미터를 바꾸지는 않았지만 실제 추종 a/j가 설정값을 넘는 차이는 남아 있다. 117m의 단일 완주를 모든 환경이나 306m 전체의 성공으로 해석하지 않는다.

- `trial04-medium/flight/report.html`: 그래프와 완료 상태
- `trial04-medium/flight/samples.jsonl`, `kinematics.csv`: 실제·추정·수신 계획 기록
- `trial04-medium/contact-audit.json`: TRACKING 구간 접촉 감사
- `trial04-medium/landed-gui.png`: 실제 Isaac/RViz 화면
- `adopted-route-plan.png`: bag 경로, 채택한 구간과 실패 지점 비교

306m는 완주하지 못했으므로 기본으로 사용하지 않는다. 이번에는 원인 구간을 우회해 임의로 건물 밖을 달리는 대신, 그 이전의 연결된 bag 경로에서 반전하도록 잘랐다. 306m 실패가 곧 해당 실제 복도의 기하학적 통과 불가능을 증명하는 것은 아니다. 추정/재계획 문제를 포함한 현재 시스템의 실패 기록이다.

비교 해상도에 관한 정정: Super-LIO 파일 기본값은 0.5m지만 navigation 런치가 실제로는 voxel 0.15m / hash 0.3m를 적용한다. trial02의 0.2m 변경은 더 촘촘한 값이 아니며 개선으로 표시하지 않는다. 최종 설정은 원래 값으로 복구했다. FAST-LIVO2 common-profile 비교용 임시 변경도 복구했고, 재현용 패치는 `experimental-fastlivo-common.patch`에 보존했다.

## 선택·제외 기준

PCD voxel 경계를 기준으로 중심의 보수적 정적 여유를 최소 **0.55m** 유지한다. 오프라인 후보는 bag에서 가로 최대 0.65m, 높이 최대 0.6m 안에서 고르고, 비슷한 후보 중 bag 위치와 연속성을 우선한다. 이것은 새 경로를 만드는 기하학적 선택 기준이며 온라인 trajectory penalty를 바꾸는 것이 아니다.

초반층의 일부 짧은 단절은 bag 주변의 제한된 공간에서 연결을 찾아봤지만 이 여유 기준으로 확인되지 않아 제외했다. 후보를 다른 높이에 두어 벽의 반대쪽으로 넘어가지 않도록 수평 witness를 검사한다. 선택적으로 시도하는 짧은 연결은 모든 edge와 바닥/천장을 검사한다. 현재 채택된 상층 구간은 원래 연속 구간이며 실제로 사용된 보수 연결은 없다. 이 검색의 실패를 물리적 통과 불가능의 증명으로 쓰지 않는다.

실행 중 SUPER는 오프라인 PCD를 제어 피드백으로 읽지 않는다. 시뮬레이션 LiDAR와 LIO 피드백으로 회피·재계획하며 truth는 결과 진단에만 쓴다.

## 보존된 경로와 재생성

`simulator/assets/super/`에 다음 파일을 보존했다. 생성 자산은 Git에서 제외한다.

- `dasan_mission_short.json/.npz`: 이전 43.74m, 초반층 출발
- `dasan_mission_medium.json/.npz`: 현재 116.83m, 상층 출발
- `dasan_mission_long.json/.npz`: 실패를 보존한 306.23m 후보
- `dasan_mission_long_anchor.json`: 상층의 검사한 출발점·패드

**출발 층이 다른 경로로 바꿀 때는 기존 navigation과 Isaac을 종료한 뒤**, 선택한 JSON과 NPZ를 기본 이름으로 복사하고 다시 실행한다. `--mission`은 navigation 입력만 바꾸므로, 이것만으로 Isaac의 출발 위치가 바뀌지는 않는다.

```bash
# 예: 이전 짧은 회귀 경로로 복귀
cp simulator/assets/super/dasan_mission_short.json simulator/assets/super/dasan_mission.json
cp simulator/assets/super/dasan_mission_short.npz simulator/assets/super/dasan_mission.npz
# 다시 117m를 쓰려면 위 두 명령의 short를 medium으로 바꾼다.
```

현재 117m 재생성은 다음과 같다. `dasan_route.py`의 인자 없는 기본값은 이전 짧은 회귀 경로를 위한 값이므로, 확장 경로에는 아래 인자를 사용한다.

```bash
source simulator/ros_setup.sh
python3 simulator/tools/super/dasan_route.py \
  --output simulator/assets/super/dasan_mission_medium.json \
  --anchor simulator/assets/super/dasan_mission_long_anchor.json \
  --start-bag-seconds 170 --end-bag-seconds 320 --minimum-length 60 \
  --lateral-limit .65 --vertical-limit .6 --clearance .55 \
  --connect-gaps --reverse-section --offset-weight 2 --smoothness-weight 1 \
  --max-forward-length 60
```

`simulator/tests/test_dasan_route.py`는 관측된 코너 연결, 벽 관통 거부, 관측되지 않은 외부 공간 연결 거부를 검사한다. 기존 기본 재생성의 출발점·43.74m 경유점이 이전 결과와 동일한 것도 비교했다.
