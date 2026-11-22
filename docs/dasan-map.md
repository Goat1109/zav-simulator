# Dasan bag 지도와 Isaac Sim 지형

현재 자산은 2026-11-17에 **FAST-LIVO2의 영상·LiDAR·IMU 융합으로 다시 복원한 지도**다. 초기 Super-LIO 복원 결과는 복도가 기울어지고 층 형상이 달라 채택을 철회했다. 이전에 실시한 로딩·정지 확인은 원본 복원 품질을 검증하기에 부족했다. 이전 파일은 `log/dasan-20261117/rejected-superlio-assets/`에 보존했다.

사용자의 정상 RViz 결과는 다른 환경의 FAST-LIVO2 결과다. 그 환경의 YAML/PCD는 제공받지 않았으므로 이 저장소 설정과 동일하다고 가정하지 않는다. 화면 유사성과 복도 평면을 검사했으며, 외부 기준 지도에 대한 절대 정확도를 검증했다고 표현하지 않는다. SUPER 주행 구성과 실험 상태는 [Dasan 주행 안내](dasan-navigation.md)에 기록한다.

## 현재 결과와 실행

`simulator/assets/super/`에 결과가 있다. 개인 bag에서 생성한 큰 자산은 Git에서 제외한다.

- `dasan.pcd`: 전체 LiDAR 관측의 XYZ 지도, 2,555,046점 (사람 흔적 후보 309점 제거 후)
- `dasan_rgb.pcd`: 동일 XYZ에 FAST-LIVO2가 생성한 실제 카메라 RGB를 옮긴 지도
- `dasan.usdc`: 색상과 정적 triangle mesh collision을 포함한 지형
- `dasan_trajectory.csv`: 지도와 동일한 강체 변환을 적용한 IMU pose 궤적
- `dasan_recipe.json`: 원본 해시, 복원 방식, 필터, 강체 좌표 변환, 출발 패드
- `dasan_color.npz`, `dasan_color.json`: RGB, 관측 마스크, 색상 출처와 해시
- `dasan.json`: 변환 해상도·메시 통계·원본/색상 해시

```bash
# Isaac Sim GUI와 PX4를 실행한다. 자동 이륙/주행은 없다.
./run_super_sim.sh dasan --overview

# 기체 주변 추적 카메라
./run_super_sim.sh dasan
```

한 번에 하나만 실행한다. 기존 tmux 대시보드를 종료한 뒤 재시작한다. 첫 실행에서는 충돌체 cooking과 셰이더 초기화가 필요하며 `[Ready] dasan`을 기다린다.

현재 지도를 별도 RViz에서 확인하려면 다음 명령을 사용한다. 진단용 `/dasan/inspection_map`만 발행하며 planner/controller를 실행하지 않는다.

```bash
source simulator/ros_setup.sh
ROS_DOMAIN_ID=73 python3 simulator/tools/super/view_dasan_pcd.py \
  --config log/dasan-inspect/map.rviz
```

## 잘못된 초기 복원의 수정

초기 Super-LIO 원본 PCD에서도 복도 방향 평면이 기울어졌다. USD 변환은 평행이동과 10cm voxel화만 수행했으므로 변환기에서 생긴 수 m 변형이 아니다. 같은 bag 마지막 측정 시각의 추정 높이도 초기 결과는 약 6.76m, 새 FAST-LIVO2 결과는 약 0.60m로 달랐다. 이 차이는 복원 단계가 달랐다는 증거이며 실제 마지막 위치의 외부 기준 오차는 아니다.

FAST-LIVO2의 `zenith.yaml`에 카메라를 켜고 `zenith_camera_pinhole.yaml`을 사용했다. planner/PX4 bridge 없이 별도 ROS domain에서 원본 bag을 읽었다. 원배속 첫 시도는 도구 세션 종료로 중단되어 비교용으로만 남겼다. 완주한 독립 재생은 2배속이며 원래 센서 header 시각을 그대로 사용했다. 결과 pose 7,484개, 전체 관측 점군 7,482개, RGB 점군 7,483개를 보존했다. 출력 pose 최대 stamp 간격은 약 0.301초, 전체 점군 최대 간격은 약 0.500초다. 모든 원본 센서 메시지가 손실 없이 처리됐다고 보장하지 않는다.

복도는 이 건물의 네 긴 구역에서 독립 평면으로 추정한다. 좁은 횡방향 계수보다 긴 방향의 경사를 사용하고, 충분한 지지점·길이·잔차 조건을 만족하는 평면의 중앙값으로 **지도 전체에 단 한 번의 1.1047° 강체 회전**을 적용했다. 같은 회전과 이동을 모든 지도 점과 궤적 pose에 적용했으며 층별 flattening이나 비선형 변형은 하지 않았다. 사람 흔적 후보 제거는 이후 별도 단계다. 강체 변환을 역으로 적용한 점과 원본 대응점 차이는 표본 검사에서 최대 약 8.1µm였다.

긴 방향 평면 경사의 절댓값 중앙값은 다음과 같다. 이 값은 자동 추정한 표면의 기울기 비교이며 외부 기준 지도에 대한 정확도나 모든 평면의 수평을 보장하지 않는다.

| 검사 구역 | 초기 Super-LIO | 현재 FAST-LIVO2 + 공통 정렬 |
|---|---:|---:|
| 서측 | 0.723° | 0.099° |
| 동측 | 0.584° | 0.349° |
| 남측 | 1.848° | 0.315° |
| 북측 | 3.481° | 0.063° |

북측 검사 평면의 긴 방향 높이 변화 중앙값은 약 1.372m에서 0.025m로 줄었다. 일부 평면에는 여전히 잔차가 있다. 현재 동측 최대 약 0.52m, 북측 한 검출 평면 최대 약 0.46m가 기록됐다. 이는 서로 다른 길이·높이의 검출 평면에 대한 값이며, 모든 실제 바닥을 정확히 분리한 측량 결과가 아니다. 따라서 외부 FAST-LIVO2 원본과 완전히 일치하거나 복원 오차가 없다고 표시하지 않는다.

## 지형과 실제 색상

전체 world 점군을 10cm voxel centroid로 합쳤으며, 반경 0.4m에서 자신 포함 이웃 5개 미만인 고립점을 제외했다. USD 표면은 10cm voxel 경계를 사용한다. 총 1,986,484개 표면 voxel, 91개 표면 메시와 1개 출발 패드, 5,748,562개 노출 표면 quad를 만들었다. 표본점에 대한 voxel 양자화 범위는 최대 약 17.3cm이며 실제 환경의 절대 오차 한계는 아니다.

색상은 이번 FAST-LIVO2의 native RGB world 점군에서 얻었다. 전체 점과 0.15m 이내 대응하는 native RGB를 사용한 점은 2,097,624개(약 82.1%)다. 나머지 457,731개는 회색이다. 이전 Super-LIO 궤적으로 투영한 색상은 재사용하지 않았다. PCD의 sRGB 바이트를 보존하고 USD에는 선형 RGB를 저장한다.

출발점은 현재 복도 내부의 연속 bag 구간에서 선택한다. 지도와 이동 경로 모두 같은 좌표계이며 `dasan_mission.json`의 출발점과 패드를 Isaac과 navigation이 함께 사용한다. 양쪽 벽·천장·바닥과 이륙 기둥의 공간을 확인했다. 이전 외부 시작점과 외부 우회 경로는 폐기했다.

사람 흔적 제거는 원본 사본을 보존하고 별도 후보에서 수행했다. 카메라에서 사람으로 검출되고, LiDAR의 여러 시각에서 빈 공간으로 확인되는 작은 비평면 성분만 제거했다. 조건은 검출 신뢰도 0.65 이상, 빈 공간 관측 8회 이상/20초 이상, 장애물 endpoint 관측 4회 이하다. 총 309점/11성분을 제거했다. 벽·큰 평면·불확실한 사람 흔적은 유지했으며 모든 사람을 제거했다고 보장하지 않는다. 검출 사진과 제거 좌표는 `log/dasan-20261117/people-filtered-assets/`에 있다. 위 메시/색상 개수는 제거 전 검증 기록이며 현재 정확한 통계는 `dasan.json`과 `dasan_color.json`을 따른다.

## 재생성

새 출력 디렉터리와 별도 후보 자산 디렉터리를 사용한다. 오래 걸리는 재생은 독립 터미널에서 실행해 도구 세션 중단에 영향을 받지 않도록 한다.

```bash
source simulator/ros_setup.sh
python3 simulator/tools/super/map_dasan_fastlivo.py \
  --output log/dasan-rebuild/reconstruction --rate 2
python3 simulator/tools/super/merge_dasan_fastlivo.py log/dasan-rebuild/reconstruction
OPENBLAS_NUM_THREADS=1 python3 simulator/tools/super/audit_dasan_corridors.py \
  log/dasan-rebuild/reconstruction/dasan_raw.pcd \
  --output log/dasan-rebuild/corridor-audit.json
python3 simulator/tools/super/prepare_dasan_scene.py log/dasan-rebuild/reconstruction \
  --assets log/dasan-rebuild/candidate-assets \
  --level-audit log/dasan-rebuild/corridor-audit.json
```

후보를 먼저 검사한 뒤 `simulator/assets/super/`에 설치한다. `pcd_scene.py`가 Isaac 내부에서 USD를 생성한다. 이 변환기는 현재 Dasan의 검사 구역과 초기 저층 바닥을 전제로 한다. 다른 bag에서 동일한 출발점이나 수평 정렬을 자동 적용한다고 가정하지 않는다.

원래 `map_dasan_bag.py`와 `color_dasan_scene.py`는 초기 Super-LIO 실험의 재현 도구다. 현재 채택 자산 생성 절차에는 사용하지 않는다.

## 검증 기록과 파라미터 상태

- `log/dasan-20261117/fastlivo-reconstruction-2x/`: 원본 설정, native 관측 fragments, 궤적, 처리 수와 stamp 간격
- `log/dasan-20261117/corridor-comparison.json`: 세 단계의 구역별 평면 수·경사·높이 변화
- `log/dasan-20261117/fastlivo-rigid-validation.json`: 강체 변환과 대응점·자산 해시 확인
- `log/dasan-20261117/fastlivo-rviz-candidate.png`: 현재 점군의 실제 RViz 확인
- `log/dasan-20261117/isaac-fastlivo-corrected-overview.png`: 현재 지형의 실제 Isaac GUI 확인
- `log/dasan-20261117/contacts-fastlivo-corrected.jsonl`: 현재 지형의 정지 접촉 확인

`simulator/tests/check_pcd_scene.py`는 음수 voxel 경계, 인접 voxel 내부면 제거, binary PCD 필드 순서, RGB와 충돌 기하 보존을 검사한다.

`src/argo_super/config/dasan.yaml`은 Dasan 실행기에 연결한 별도 설정이다. penalty·traj_opt 전체·속도·가속도·저크 제한·OMMPC/Super-LIO 설정은 기존 common과 같다. 기존 `common_super_lio.yaml` 및 `ommpc_high_speed.yaml`도 보존했다. 지도 설정과 주행 검증의 범위·실패는 [주행 안내](dasan-navigation.md)를 따른다.
