# ARGO 기체 선택과 argo_v2 등록값

기본 기체는 `argo_v1`이다. `argo_v2`는 별도 CAD·물리 USD·프로파일·센서 외부 보정·PX4 airframe·RViz 메시로 등록했다. 한 번에 한 기체를 실행하며 두 기체 동시 비행을 추가한 것은 아니다.

## 실행

```bash
# 터미널 1: 원하는 기체와 장면
./run_super_sim.sh high_speed --vehicle argo_v2
# 기존 기체
./run_super_sim.sh high_speed --vehicle argo_v1
# 일반 창고 시뮬레이터와 Isaac 단독 실행도 같은 옵션 지원
./run_argo_sim.sh --vehicle argo_v2
./run_isaac_sim.sh --vehicle argo_v2

# 터미널 2: 실행 중인 기체·ROS domain·센서 보정 디렉터리를 자동 상속
./run_super.sh --scenario high_speed --execute
# 명시하면 시뮬레이터와 같은 기체인지 검사
./run_super.sh --scenario high_speed --vehicle argo_v2 --execute
./run_fastlivo.sh --vehicle argo_v2
./run_fastlio.sh --vehicle argo_v2
./run_superlio.sh --vehicle argo_v2
```

장면은 `warehouse`, `high_speed`, `dense`, `mixed`, `dasan`, `frontier` 모두 지원한다. `--overview`와 `--vehicle`은 함께 사용할 수 있다. 기체를 바꿀 때는 기존 시뮬레이터와 LIO/주행 실행을 Ctrl+C로 종료한다. 실행 중인 기체와 다른 선택을 같은 세션에 연결하려 하면 오류로 거부한다. `ARGO_VEHICLE=argo_v2` 환경변수도 지원한다. 기존 safety fault 실험은 동일한 센서/제어 토픽을 사용하므로 선택된 기체에서 기존 명령으로 실행한다.

## 파일 분리

| 파일 | 역할 |
|---|---|
| `simulator/config/vehicles/argo_v1.json` | 기존 v1 값. 물리 USD와 motor/PX4 값은 보존 |
| `simulator/config/vehicles/argo_v2.json` | v2의 모든 기체별 입력값 |
| `simulator/vehicle_profiles.py` | 기체 선택, 센서 장착 역산, 주행 설정 overlay, PX4 geometry 계산 |
| `simulator/pegasus/simulator/logic/vehicles/multirotors/argo_v2.py` | v2 preset. v1의 공통 모터/센서 backend 재사용 |
| `simulator/pegasus/simulator/assets/Robots/argo_v2/argo_v2_cad.usdc` | 홈의 CAD 원본을 바이트 그대로 복사 |
| 같은 폴더의 `argo_v2.usda` | 5 rigid body, 4 회전 관절, 명시적 질량/관성, 27 CAD 충돌체 |
| `simulator/config/vehicles/argo_v2_calibration/` | v2 LiDAR↔카메라 외부 보정과 공통 카메라 내부 보정 |
| `simulator/px4/10021_isaac_argo_v2` | v1 공통 설정을 읽고 v2 geometry/hover만 override |
| `src/argo_super/meshes/argo_v2.stl` | 외부 형상과 프로펠러로 만든 RViz 표시 메시 |
| `simulator/config/vehicles/argo_v2_audit.json` | 전체 숫자, CAD 부품별 변환/경계, 주축 quaternion, 합성 관성, SHA-256 |

시뮬레이터의 원래 CAD는 내부 전자부품까지 유지한다. RViz STL은 표시 부담을 줄이기 위해 외부 형상·모터·센서·프로펠러 156,790개 삼각형만 사용한다. 물리 충돌과 질량/관성은 이 STL로 계산하지 않는다.

## 좌표와 단위

body는 X 전방, Y 좌측, Z 위의 FLU다. 원점은 제공된 CAD 원점이며 Mid360 밑면의 Z=0 평면이다. CAD에서 센서 중심축은 X≈−0.602829 mm, Y≈−0.288330 mm에 있다. 원점을 임의로 그 중심축으로 옮기지 않았다. 제공한 COM 좌표를 그대로 사용하고 CAD 기하를 0.001배 하여 m 단위로 참조했다.

변환: g→kg는 10⁻³, mm→m는 10⁻³, g·mm²→kg·m²는 10⁻⁹. 대각 성분만 남기지 않고 전체 대칭 관성행렬을 고유분해하여 USD의 `diagonalInertia`와 `principalAxes`로 저장한다.

## 질량과 무게중심

| 항목 | 적용값 |
|---|---|
| base | 2.538596 kg |
| 프로펠러 1개 | 0.009138 kg |
| 합계 | **2.575148 kg** |
| base COM, body 기준 m | `[0.00038, -0.000967, -0.047093]` |
| 전체 COM, body 기준 m | `[0.000374606228458, -0.000953274270838, -0.047740351944]` |

프로펠러 COM (body 기준 m):

| 구분 / 모터 index | COM |
|---|---|
| FR / rotor0 | `[0.098397, -0.098712, -0.0927]` |
| RL / rotor1 | `[-0.098397, 0.098712, -0.0927]` |
| FL / rotor2 | `[0.098397, 0.098712, -0.0927]` |
| RR / rotor3 | `[-0.098397, -0.098712, -0.0927]` |

v1은 기존 설계를 재현하기 위해 nominal 제어/플래너 질량 1.99 kg, 실제 USD base 1.99 kg + rotor 0.005 kg × 4 구조를 그대로 보존했다. v2는 base+프로펠러 합계 2.575148 kg를 제어기와 플래너에도 일치시켰다.

## COM 기준 관성행렬 (kg·m²)

base:

```text
[0.025, 5.823584e-05, -5.3201116e-05]
[5.823584e-05, 0.02349, 5.959254e-06]
[-5.3201116e-05, 5.959254e-06, 0.03528]
```

프로펠러 각 1개:

```text
[1.8146831e-05, 4.33e-19, -4.1e-19]
[4.33e-19, 1.8146836e-05, 0]
[-4.1e-19, 0, 3.6219041e-05]
```

USD에 저장된 주축 값·방향 quaternion·각 rotor의 관절 기준 local COM 및 평행축 정리로 합성한 전체 관성은 audit JSON에 전부 수록했다.

## 모터축과 PX4

모터 순서와 회전 방향은 v1과 같이 FR, RL, FL, RR 및 `[-1, -1, +1, +1]`이다. CAD 프로펠러 그룹의 회전축을 관절 원점으로 사용했다. 제공한 COM과 CAD 축이 최대 약 1.206 mm 다르므로 둘을 같은 값으로 덮어쓰지 않고 rotor local COM 오프셋으로 표현했다.

| 모터 | CAD 축, body FLU m | PX4 위치, 전체 COM 기준 FRD m |
|---|---|---|
| FR / 0 | `[0.0983971715558, -0.09928833002, -0.0892]` | `[0.0980225653274, 0.0983350557492, 0.041459648056]` |
| RL / 1 | `[-0.0996028284442, 0.09871166998, -0.0892]` | `[-0.0999774346726, -0.0996649442508, 0.041459648056]` |
| FL / 2 | `[0.0983971715558, 0.09871166998, -0.0892]` | `[0.0980225653274, -0.0996649442508, 0.041459648056]` |
| RR / 3 | `[-0.0996028284442, -0.09928833002, -0.0892]` | `[-0.0999774346726, 0.0983350557492, 0.041459648056]` |

| 파라미터 | v1 → v2 |
|---|---|
| PX4 autostart / target | `10020 / isaac_argo` → `10021 / isaac_argo_v2` |
| MPC_THR_HOVER 및 OMMPC hover_throttle | `0.33` → `0.473424657359` |
| 추력 계수 | `5.4935e-6 N/(rad/s)²`, 동일 |
| 반토크 계수 | `9.888e-8 N·m/(rad/s)²`, 동일 |
| PX4 CA_ROTOR0..3_KM | `[+0.05,+0.05,−0.05,−0.05]`, 동일 |
| backend input scaling / armed idle | `1937.4 / 155 rad/s`, 동일 |
| rotor min/max | `0 / 2000 rad/s`, 동일 |
| backend update rate | `250 Hz`, 동일 |
| 선형 항력 | `[0.5,0.3,0]`, 동일 |

호버 명령은 기존 backend의 `ω = 1937.4u + 155`, `4kω² = mg`를 역산했다. 모터 계수와 PID/EKF 파라미터는 변경하지 않았다. 기존 KM=0.05와 물리 추력/반토크 비율 약 0.018의 차이도 요청대로 보존했다.

## 센서

| 항목 | v2 적용값 |
|---|---|
| LiDAR 위치, body m | `[-0.000602828839902, -0.000288330019988, 0.0465107092138]` |
| LiDAR Euler ZYX | `[0,0,0]°` (v1은 pitch +22°) |
| 카메라 위치, body m | `[0.106321071556, -0.000288330019988, -0.0213]` |
| 카메라 Euler ZYX | `[0, 0, 0]`° |
| IMU 위치, body m | `[0.0103971711601, 0.02300166998, 0.0906307092138]` |
| IMU Euler ZYX | `[0, 0, 0]`° |
| camera resolution / K / distortion | v1과 동일, `640×512`; 전체 행렬/계수 audit JSON |
| 물리/IMU / 렌더·LiDAR·RGB | `200 Hz / 20 Hz`, 동일 |
| 센서 노이즈, 내부 LiDAR–IMU 외부 보정 | v1 값 유지 |

LiDAR 가상 측정 원점은 CAD `MeshInstance_139` 광학 돔의 경계 중심, 카메라 가상 광학 원점은 `MeshInstance_160` 전방 렌즈 평면 중심으로 정했다. 실제 내부 투영 중심이나 IMU die를 CAD에서 확인한 값으로 주장하지 않는다. 특히 IMU 기준점은 v1의 `extrinsic_T=[−0.011,−0.02329,−0.04412]`와 변환 관례를 그대로 유지해 역산한 가상 좌표다.

카메라의 CAD 배치가 달라졌으므로 v1 Rcl/Pcl을 그대로 쓰지 않았다. v2는 정면 카메라에 대해 `Rcl=[[0,−1,0],[0,0,−1],[1,0,0]]`이며 Pcl은 두 센서의 body 위치 차이를 optical 좌표로 바꿔 생성했다. 카메라 내부 보정은 복사하여 유지했다. 시뮬레이터·FAST-LIVO·FAST-LIO·Super-LIO·PX4 bridge·SUPER adapter·EPIC adapter가 동일한 선택값을 읽는다.

## 충돌·spawn·플래너

| 항목 | 적용값 |
|---|---|
| CAD 전체 경계 m | `[-0.207602836,−0.207288337,−0.124199998]` ~ `[0.206397178,0.206711677,0.060097100]` |
| 외곽 크기 | 약 `414 × 414 × 184.297 mm` |
| 충돌 형상 | frame/guard/sensor/motor 등 27 CAD 부품. 넓은 프레임/가드는 convex decomposition |
| contact / rest offset | `0.002 / 0 m`, v1과 동일 |
| spawn Z 보정 | 기존 장면·mission spawn에 `+0.0291999979019 m` |
| robot_r | `0.4 m` 유지. 더 큰 실험 여유값이 있으면 축소하지 않음 |
| flatness.mass / controller mass | `1.99` → `2.575148 kg` |
| 속도·가속도·저크 제한, penalty, 제어 gain | 기존값 유지 |

spawn 보정은 v1 시각/물리 형상 최저점 −0.095 m와 v2 최저점 −0.124199998 m의 차이다. 동일한 초기 바닥 간격을 보존한다. 경로·지도·mission 파일은 수정하지 않는다. 기록의 `effective_config.yaml`에는 원본 common/실험 YAML 위에 적용한 기체 질량·충돌 여유·추력 환산값이 저장된다. `metadata.json`에는 기체명, 전체 profile, calibration 경로, 실제 IMU mount가 함께 저장된다.

## 재생성과 검증

### 다른 PC의 뷰 설정

`simulator/tools/runtime/chase_camera.py`와 `simulator/argo_sim.py`에 Perspective 자유 시점과 추적 카메라 분리, 줌, 벽 회피, 전체 기체 프레이밍이 구현되어 있어 pull 후 재실행하면 적용된다. `simulator/config/viewport.json`에는 현재 PC의 카메라 이동 속도·회전·관성·조작 설정만 추려 저장했으며 시뮬레이터 실행 때 적용한다. 사용자 전체 Isaac 설정이나 PC 경로·창 배치는 복사하지 않는다. 마지막에 마우스로 변경한 카메라 위치·방향은 저장된 stage 자료가 없어 포함하지 않았다. 렌더링 센서 카메라의 내부/외부 보정값은 이 파일로 바꾸지 않는다.

USD/pxr 및 SciPy가 로드되는 Isaac Python에서 다음 파일을 실행한다. 등록된 산출물도 저장소에 포함되어 있어 일반 실행 때 재생성할 필요는 없다.

1. `simulator/tools/runtime/build_argo_v2_asset.py` — 물리 USD 생성
2. `simulator/tools/super/export_argo_mesh.py --vehicle argo_v2` — RViz 메시 생성
3. `python3 simulator/tools/runtime/generate_argo_v2_airframe.py` — PX4 override 생성
4. `simulator/tools/runtime/audit_argo_v2.py` — 전체 수치와 파일 hash 기록

검증 결과:

- 기존 portable/source/센서/lockstep/프로세스 종료 검사 통과.
- 제어·ROS·profile 회귀 테스트 67개 통과.
- 정적 USD 검사: 5 rigid body, 4 관절, 27 충돌체, SI 단위, 전체 관성/COM, 이동한 프로펠러의 CAD world transform 확인.
- 무제어 2초 추력 균형 검사: 초기 gravity-only tick의 속도를 분리하여 수직 속도 변화 약 −0.000172 m/s 확인. 자세 제어를 포함한 비행 검증은 아래의 실제 PX4 시험에서 수행.
- v2 bridge 검사: odometry 80개 수신, IMU 장착각 제거, 중복 발행 차단, 정상 종료 확인.
- 여섯 시나리오의 v2 launch 구성과 적용 질량 확인.
- high_speed 장면에서 Super-LIO→SUPER/OMMPC→PX4를 연결해 이륙, 약 3초 READY 정지 비행, 착륙, 자동 시동 해제를 확인. 정지 구간 추정 높이 1.490036~1.506499 m (602 샘플).
- 전체 숲 왕복·Dasan/Frontier 전 구간을 v2로 완주 검증한 것은 아니다. 실기체 튜닝/비행 검증도 포함하지 않는다.

로컬 시험 산출물: `log/vehicle-validation/argo_v2_physics.json`, `log/vehicle-validation/argo_v2-smoke/{report.html,metadata.json,effective_config.yaml,smoke_result.json,samples.jsonl}`. 시험용 시뮬레이터와 주행 프로세스는 종료했다.
