# Frontier FAST-LIVO2 지도와 SUPER 주행

현재 기본 경로는 복도 길이를 자르지 않은 500m 층별 복도 끝 방문·최상층 종료 시험 경로다. 현재는 중간 강제 정지를 제거한 하나의 연속 Path를 쓴다. 센서·추정 시각과 계획 실행을 수정한 trial30·31에서 두 번 연속 500.54m/203경유점 전체 완료·최상층 자연 바닥 LANDED·navigation 충격/관통 0건을 확인했다. 이전 186m 성공 경로는 별도 보존했다. 상세 결과는 [전 층 경로 기록](frontier-all-floors.md)을 참고한다. 아래 78m 왕복은 이전 검증 기록이다.


원본은 `~/Bags/Frontier/scan_sensors_20260917_170833_0.db3`다. 약 600.50초, Livox LiDAR 5,987개, IMU 119,656개, RGB 이미지 6,005개다. 외부 위치·정답 지도는 없다.

## 복원과 지형

Dasan과 같은 FAST-LIVO2/카메라 보정으로 재생했지만 초기 설정은 약 64초부터 발산했다. 1배속도 해결하지 못했다. 오프라인 복원에서만 `vio.img_point_cov=100000000`으로 영상 측정 가중치를 낮추자 전체 597.42초의 유효 위치 추정이 완료됐다. LiDAR/IMU 기반 FAST-LIVO2 추정에 약한 영상 업데이트를 사용한 결과다. 온라인 Super-LIO·SUPER·OMMPC 파라미터를 바꾼 것이 아니다. 실패 결과는 별도 로그에 남겼고 지형에 쓰지 않았다.

`log/frontier-20261118/fastlivo-low-visual-weight/`가 채택한 복원이다. 실제 카메라 RGB를 같은 FAST-LIVO2 세계 점군에 전사했다. 지도는 717,380점, 직접 RGB가 있는 점은 653,377개(91.08%)다. 나머지는 회색이다. 원본 bag이나 건물 구조를 삭제하지 않았다. 3,785개 고립점만 기존 반경 지지 필터로 제외했다.

수평인 국소 평면 8,029개 중 일관된 7,466개에서 한 번의 1.33045° 전역 강체 회전을 구했다. 같은 회전·평행이동을 지도와 bag 경로 모두에 적용했다. 계단·층별 변형이나 높이 평탄화는 없다. bag 경로 길이는 약 663.15m, 센서 높이 변화는 약 24m다. 루프 폐합/절대 오차 보정은 없으며, 전체 복원 성공이 정확한 측량 지도라는 뜻은 아니다.

생성 자산은 `simulator/assets/super/frontier.pcd`, `frontier_rgb.pcd`, `frontier.usdc`, `frontier_recipe.json`, `frontier_trajectory.csv`, `frontier_color.npz`다. USD는 10cm 표면 복셀, 전역 가상 바닥 없이 25개 지형 mesh, 약 149만 노출 면과 이륙 패드를 사용한다. PCD와 USD 원본 해시는 일치한다. 최대 복셀 표면 근사 상한은 17.32cm다. 실내 화면 검토를 위해 viewport Camera Light를 켠다.

색상 PCD는 RViz에서 직접 열어 확인했다(`pcd-rviz-gui.png`). 실제 Isaac 지형과 RViz를 켠 상태에서 비행을 시험한다. `map-overview.png`는 점군/기록 경로, `stair-route.png`는 선택한 계단 경로를 보여 준다.

## 공통 설정

실행 설정은 모든 장면에서 `src/zav_super/config/common.yaml` 하나다. 이전 최종 `dasan.yaml`과 바이트 단위로 같다(SHA-256 `a757c64015713ec4107769578ed55661c0b40eb5a011a9c46fac090c5f38bea9`). 원래 공통값과 다른 YAML은 `src/zav_super/config_archive/20261118/`에 원본 그대로 백업했으며 manifest에 해시를 보존했다. penalty, 속도·가속도·저크, OMMPC, 지도/LIO 값은 Dasan 최종값 그대로다. 이륙 높이와 위치는 환경 임무에서 공급한다.

공통 경로 소비자는 먼 목표를 향한 직선이 상위 3D polyline을 map resolution(0.15m)보다 크게 자르지 않도록 preview를 제한한다. 이는 전역 지도나 truth를 제어에 넣지 않고 경유점의 공간 순서를 보존하는 규칙이다. 직선 경로에는 기존 braking lookahead를 유지한다. 모든 시나리오에 같은 규칙을 사용한다.

## 이전 78m 경로와 결과

이전 검증 기본값은 **77.87m / 32개 일반 경유점 / 5개 상위 경로 구간**이다. bag 위치 추정 기준 약 **82.10–124.80초**에서 가져왔다. 계단 전의 평평한 복도에서 이륙해 **한 층 약 4.2m 상승 → 상층 복도 왕복 → 계단 하강 → 같은 패드 착륙**을 실행한다. 전체 663m bag이나 더 높은 모든 층을 비행 검증한 것은 아니다.

이륙 body 위치는 `[-3.26924, -0.05305, 0.16000]`m, 초기 yaw는 `0.13288`rad, 이륙 목표 Z는 `1.64114`m다. 주변 바닥 162개 점과 바닥/천장/좌우 벽·이륙 기둥을 확인했다. 지도에 1×1m 지지 패드 하나만 추가했다. 경로 중심의 보수적 정적 여유는 최소 0.55m이며, 5cm 간격 재검사도 통과했다. 기체의 실제 추종 오차까지 보장하는 값은 아니다.

**trial07-near-stairs**를 기본 명령 그대로 실행했다. 5개 구간의 도착·정지와 다음 구간 재출발, 최종 `LANDED`까지 확인했다. 이륙 후 최초 TRACKING부터 LANDING 전까지 **정지 viewpoint 대기 시간을 포함해** PhysX 충격·관통 보고 0건이다. `evaluation.json`의 `finite_bag_test_complete=true`가 이 유한 bag 시험의 완료 판정이다. 외부 탐사 소비자는 마지막에도 `WAITING_PATH`이므로 범용 그래프의 `mission_complete=false`는 이후 EPIC 탐사 전체 완료를 선언하지 않는 의도된 구분이다.

실제 body 기준 최대 속도는 약 **4.43m/s**, 가속도/저크 피크는 **7.38m/s² / 84.83m/s³**다(약 0.2초 평활 미분). 설정 한계는 바꾸지 않았지만 실제 추종 a/j가 계획 한계를 넘는 차이는 남는다. 성공한 한 시험으로 모든 환경의 성능을 보장하지 않는다.

계단 중간참과 층 진입점에서만 상위 경로를 나누고 3초 관측 대기한다. 일반 경유점마다 정지하는 방식은 아니다. 긴 lookahead가 계단 순서를 자르는 문제와, 정지 요청 후 이전 SUPER 궤적이 다시 정지를 풀던 문제를 수정했다. 현재 정지는 새로운 click goal과 그 이후 생성된 궤적이 와야 풀리며, stale feedback·emergency stop 보호는 유지한다. 이 규칙은 모든 환경에서 동일하다.

별도로 **trial05-corridor**의 135.07m 복도 경로도 `COMPLETED + LANDED`, 주행 중 충격·관통 0건을 확인했다. `frontier_mission_corridor.json/.npz`로 보존했다. 실패한 109m 계단 후보는 `frontier_mission_stairs.json/.npz`, 채택한 78m는 `frontier_mission_near_stairs.json/.npz`로 보존한다. 출발점이 다르므로 경로를 바꿀 때 navigation/Isaac을 종료하고 JSON과 NPZ 둘 다 기본 `frontier_mission.*`으로 복사한 후 다시 켠다. `--mission`만 바꾸면 물리 기체 출발 위치는 바뀌지 않는다.

결과: `log/frontier-20261118/trial07-near-stairs/evaluation.json`, `contact-audit.json`, `flight/report.html`, `landed-isaac.png`, `landed-rviz.png`. 원본 기록과 실패 원인도 같은 로그 루트에 보존한다.

## 상위 viewpoint 경로 계약

`/mission/exploration_path`의 `nav_msgs/Path`를 사용한다. frame은 `world`, position은 유한한 XYZ(미터), orientation은 yaw(라디안)의 단위 quaternion이다. JSON 임무의 `viewpoints`에 x/y/z/yaw도 명시한다. 0 quaternion은 yaw 미지정이며, 주행 방향을 바라보게 한다. 명시 yaw는 보존한다.

같은 경로 재발행은 진행을 초기화하지 않는다. 새 경로는 현재 기체 근처 2m 이내에서 시작해야 한다. 보통 경유점은 멈추지 않고 통과한다. 실제 탐사 viewpoint에서 관측/정지가 필요하면 그 지점을 현재 Path의 끝으로 보낸다. 도착·정지 후 `WAITING_PATH`/native hold가 확인되면 다음 경로를 보낸다. 소비자는 외부 경로 완료만으로 착륙하지 않는다.

bag 시험 발행기의 `viewpoint_pause_indices`는 이 방법으로 계단 중간참과 층 진입점에서 구간을 나눈다. `--land-at-end`는 시험 발행기만 마지막에 착륙을 요청하는 옵션이다. 실제 EPIC 입력에는 시험 발행기를 함께 켜지 않는다. ROS 1 EPIC 자체나 그 position/yaw polynomial 변환 bridge까지 연결한 것은 아니며, 공간 viewpoint → SUPER → OMMPC → PX4의 ROS 2 입력 계약을 시험하는 구성이다.

## 실행

```bash
./run_super_sim.sh frontier
# [Ready] 이후 다른 터미널. 두 GUI가 열린다.
./run_super.sh --scenario frontier --execute
```

bag 임무에 정지 viewpoint가 있으면 런치가 상위 시험 발행기를 함께 실행한다. 실제 탐사 프로그램을 연결할 때는 다음을 사용한다.

```bash
./run_super.sh --scenario frontier --route-source external --execute
# EPIC bridge 또는 시험 발행기 중 하나만 /mission/exploration_path를 발행한다.
# 시험용:
source simulator/ros_setup.sh
export ROS_DOMAIN_ID=119
python3 simulator/tools/super/publish_bag_route.py --scenario frontier --land-at-end
```

## 실험 기록

- `trial00-inspection`: 최초 지형 로드/화면 점검. 실내 stage light가 가려지는 표시 문제를 발견했다.
- `trial01-stairs`: Camera Light 표시 변경 중 Python import scope 오류. 비행 전에 종료하고 수정했다.
- `trial02-stairs`: 전체 109m 경로를 한 번에 입력. 약 21.7m 지점에서 계단 진입 모서리 충돌. 실패.
- `trial03-stair-guidance`: 3D chord preview 검사 적용. 한 층 올라갔으나 약 31.9m에서 상층 천장 접촉/heartbeat fault. 실패.

- `trial04-viewpoints`: 109m를 5개 구간으로 분할. 첫 정지 뒤 이전 endpoint의 궤적이 hold를 다시 풀어 중간참 접촉/FAULT. 실패.
- `trial05-corridor`: 135.07m 복도 경로, 전체 완료·착륙 및 충격·관통 0건.
- `trial06-stairs-hold`: hold 해제 조건 수정 후 계단 상승/상층 복도/하강을 접촉 없이 통과했으나, 복귀 하층 연결 복도에서 충돌·FAULT. 109m 전체 성공이 아니다.
- `trial07-near-stairs`: 실패한 하층 연결부를 제외하고 계단 가까운 평지에서 출발하는 77.87m. 5개 경로 구간과 3초 대기/재출발, 최종 착륙, 공중 navigation 구간(정지 대기 포함) 충격·관통 0건. 채택.

## 재생성

```bash
source simulator/ros_setup.sh
python3 simulator/tools/super/map_dasan_fastlivo.py \
  --bag ~/Bags/Frontier --output log/frontier-new-reconstruction \
  --image-cov 100000000 --rate 2 --domain 75
python3 simulator/tools/super/merge_dasan_fastlivo.py log/frontier-new-reconstruction --name frontier
python3 simulator/tools/super/audit_reconstructed_map.py \
  log/frontier-new-reconstruction/frontier_raw.pcd --output log/frontier-level-audit.json
python3 simulator/tools/super/prepare_dasan_scene.py log/frontier-new-reconstruction \
  --scenario frontier --level-audit log/frontier-level-audit.json
python3 simulator/tools/super/dasan_route.py --scenario frontier \
  --output simulator/assets/super/frontier_mission.json \
  --clip-start --start-bag-seconds 82 --end-bag-seconds 125 --minimum-length 15 \
  --lateral-limit .65 --vertical-limit .6 --clearance .55 \
  --offset-weight 2 --smoothness-weight 1 --yaw-policy path-heading
```

채택한 동일 복원 데이터로 경로를 생성한 후 `viewpoint_pause_indices=[5,7,25,27]`, `viewpoint_wait_seconds=3.0`, `end_wait_seconds=3.0`을 설정한다. 새로 복원한 지도에서도 같은 점 번호나 출발 위치가 안전하다고 가정하면 안 되므로 정적 재검사와 GUI 비행이 필요하다.

USD는 `./run_super_sim.sh frontier`가 Isaac 환경 안에서 자동 생성/갱신한다. 일반 system Python 또는 Isaac venv만으로 `pcd_scene.py`를 실행하면 pxr 경로가 없어 실패할 수 있다.

검증은 `simulator/tests/check_measured_scene.py frontier`(지도/RGB/USD/패드 해시·bounds와 5cm 선분 여유), `test_dasan_route.py`(벽/미관측 외부 연결 거부), `src/zav_super/test/test_bag_mission.py`(16개, 3D viewpoint·yaw 보존·preview·경로 교체 등), `ctest --test-dir build/zav_super`로 수행했다.
