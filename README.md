# ZAV Simulator

Isaac Sim **5.1.0** + Pegasus + PX4 **v1.14.3** 기반 GPS 없는 드론 시뮬레이터입니다. MID-360 형태의 LiDAR, 카메라와 장착 IMU를 FAST-LIVO2에 입력하고, 추정 odometry를 ROS 2 노드를 통해 PX4 EKF2로 전달합니다.

이 저장소의 `src/`에 ROS 패키지를 포함합니다. Pegasus 소스는 `simulator/pegasus/`에서 직접 관리합니다. PX4·Micro XRCE-DDS Agent만 설치 스크립트가 고정된 Git 커밋으로 내려받습니다. Isaac Sim 자체, 가상환경, 빌드 결과, 개인 BSON·로그는 포함하지 않습니다.

SUPER의 모든 시나리오는 이제 `src/zav_super/config/common.yaml` 하나를 사용한다. 내용은 Dasan 최종 파라미터 그대로이며, 이전 YAML은 `src/zav_super/config_archive/20261118/`에 해시와 함께 보존했다. `--profile common`은 기존 명령 호환용으로 유지하며 기본값도 common이다.

Frontier의 현재 시험 경로는 계단 상승 → 층별 긴 복도 끝 방문을 반복하는 500m 경로이며 지상 복귀는 없다. 현재 중간 강제 정지 없는 하나의 Path를 사용한다. 센서·추정 시각과 계획 실행을 수정한 trial30·31에서 두 번 연속 500.54m/203경유점 전체를 완료하고 최상층 자연 바닥 착륙을 확인했다. navigation 충격·관통 0건이며 common YAML과 중간 강제 정지 없는 경로를 유지했다. 이전 186m 성공 경로는 별도 보존했다. [전 층 경로/시험 기록](docs/frontier-all-floors.md), [FAST-LIVO2 지도 복원](docs/frontier-navigation.md)을 참고한다.

## 지원 환경

| 환경 | 실행 구성 | 확인 범위 |
| --- | --- | --- |
| Ubuntu 22.04 x86_64 + NVIDIA GPU | Linux Isaac Sim + ROS 2 Humble | 배포본에서 GUI·센서·LIO→EKF 위치 확인. 상세 검증은 [검증 기록](docs/verification.md) 참조 |
| Windows 11 22H2 이상 + WSL2 Ubuntu 22.04 | Windows Isaac Sim + WSL ROS 2/PX4, mirrored networking | Windows GPU 실행, 센서·FAST-LIVO·EV, 이륙·위치 이동·터미널 조종·착륙 확인. [검증 기록](docs/verification.md) 참조 |

Isaac Sim의 [GPU·드라이버 요구사항](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)을 충족해야 합니다. 설치 스크립트가 ROS 2와 컴파일 의존성을 설치하므로 인터넷·sudo 권한·충분한 여유 디스크가 필요합니다. 다른 Ubuntu 버전, ARM, Windows 10, WSL 기본 NAT를 동일 지원 환경으로 가정하지 않습니다.

## 처음 설치하기

개발 브랜치는 **`ZAVSIM_Fix`**입니다(대소문자 구분). 친구에게 전달할 전체 절차와
필요 파일 목록은 **[설치·공동 개발 안내](docs/installation.md)**를 참고하세요.

GitHub 조직의 비공개 저장소 접근 권한이 있는 계정으로 Git 인증을 먼저 설정합니다. 아래 저장소를 clone합니다.

```bash
sudo apt-get update
sudo apt-get install -y git software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
git clone --branch ZAVSIM_Fix --single-branch https://github.com/ZenithFuturesLab/zav-simulator.git ~/zenith_ws
cd ~/zenith_ws
```

`gh auth login`이 표시하는 일회용 코드와 주소를 사용해 브라우저에서 GitHub 로그인을 완료합니다. `ZenithFuturesLab` 접근 권한이 있는 계정이어야 합니다.

이미 `~/zenith_ws`가 있으면 다른 빈 경로로 clone하세요. 기존 폴더에 덮어쓰거나 기존 작업을 삭제하지 마세요.

**Ubuntu:** Isaac Sim에서 제공하는 `python.sh` 또는 Isaac Sim 5.1.0이 설치된 Python 3.11 가상환경의 Python을 지정합니다. 일반 `/usr/bin/python3`은 사용하지 않습니다.

```bash
./setup.sh --platform native --isaac-python /absolute/path/to/isaac-sim/python.sh
```

pip 방식 설치라면 다음처럼 지정합니다.

```bash
./setup.sh --platform native --isaac-python /absolute/path/to/isaac_env/bin/python
```

**Windows + WSL2:** 먼저 [WSL2 설치 안내](docs/windows-wsl2.md)에 따라 mirrored networking과 Windows Isaac Sim 실행을 확인하고, WSL 터미널에서 실행합니다.

```bash
./setup.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
```

실행 전 설치 단계를 확인하려면 `./scripts/bootstrap.sh --dry-run`을 사용합니다. 전체 옵션은 `--help`로 확인합니다. 작업 수 기본값은 2이며 `ZAV_BUILD_JOBS=4 ./scripts/bootstrap.sh ...`처럼 조절할 수 있습니다.

## 실행하기

설치가 끝나면 새 터미널 네 개에서 아래 명령을 각각 실행합니다. `./run_*.sh`는 clone한 저장소 루트에서 실행합니다. 설치 시 홈 디렉토리에도 호환 실행기가 만들어집니다.

```bash
# 터미널 1: Isaac Sim + PX4 + DDS Agent
zav_sim
# 터미널 2: FAST-LIVO2 + LIO → PX4 ROS 2 bridge
./run_fastlivo.sh
# 터미널 3: 비행 관리자
ros2 run zav_control flight_manager
# 터미널 4: 센서 RViz
./run_mid360_rviz.sh
```

새 터미널 환경 등록 없이 실행하려면 각 터미널에서 `source /path/to/zenith_ws/simulator/ros_setup.sh`를 먼저 실행하고, 시뮬레이터는 저장소 루트의 `./run_zav_sim.sh`로 실행합니다. 브리지 Python을 따로 실행할 필요가 없습니다.

LiDAR·IMU만 사용하는 Super-LIO로 교체하려면 FAST-LIVO를 종료하고 `./run_superlio.sh`를 실행합니다. 설치·빌드·입출력 호환 계약은 [Super-LIO 안내](docs/super-lio.md)를 참고하세요.

첫 실행은 Isaac shader 캐시와 확장 로딩 때문에 오래 걸릴 수 있습니다. 비행 명령 전에 LiDAR·IMU·카메라와 EV odometry 수신을 [검증 절차](docs/verification.md)로 확인하세요. 시뮬레이터 대시보드에서 Ctrl+C로 종료합니다. 종료기는 동일 사용자 PX4·DDS Agent 및 Windows `kit.exe`도 정리하므로 다른 시뮬레이터와 동시에 실행하지 마세요.

FAST-LIVO는 실행한 터미널에서 Ctrl+C로 종료합니다. `[LIO] All owned processes stopped`가 출력된 뒤 재실행하세요. 실행기가 만든 mapper·LIO bridge·relay를 정리한 후 재실행 잠금을 해제합니다. RViz가 먼저 실행해 소유한 공유 relay는 유지합니다. 잠금 파일을 직접 삭제하지 마세요. 이전 mapper가 남아 있으면 새 실행을 거부하고 PID를 표시합니다.

## SUPER 공식 숲 벤치마크

기존 **zav_sim + FAST-LIVO + PX4**로 공식 숲을 비행하려면 `./scripts/run_zav_forest.sh dense` 또는 `high_speed`를 사용합니다. 플래너·자동 왕복 임무 실행은 [ZAV/PX4 숲 비행 안내](docs/zav-forest-benchmarks.md)를 참고하세요.

공개 OMMPC를 PX4 각속도 제어에 연결한 비교 구성은 [OMMPC 숲 비교 안내](docs/ommpc-forest-comparison.md)를 참고하세요. 기존 PX4 제어기는 기본값으로 유지합니다.

기체 408×408×245.5mm 기준 충돌체·지도 팽창 점검과 실측형상 실험 옵션은 [ZAV footprint 점검](docs/zav-footprint-audit.md)에 정리했습니다.

원본 지도·목표·속도 설정으로 dense와 high-speed 예제를 실행하려면 `./scripts/run_super_benchmark.sh dense` 또는 `./scripts/run_super_benchmark.sh high_speed`를 사용합니다. 공식 `perfect_drone_sim`을 사용하는 독립 실험이며, 실행 절차와 PX4 추종과의 차이는 [SUPER 공식 벤치마크 안내](docs/super-official-benchmarks.md)에 정리했습니다.

개방→밀집→개방 혼합 맵은 동일한 SUPER/Super-LIO 공통 파라미터로 왕복합니다.

터미널 1에서 Isaac Sim GUI를 실행합니다.

```bash
cd ~/zenith_ws
./run_super_sim.sh mixed
```

`[Ready] mixed` 출력 후 터미널 2에서 RViz2와 자동 왕복 임무를 실행합니다.

```bash
cd ~/zenith_ws
./run_super.sh --scenario mixed --backend super_lio --profile common --execute
```

공통 설정은 `src/zav_super/config/common.yaml`이며 현재는 Dasan 최종값으로 통합했습니다.
숲의 과거 맵 구성·GUI 검증·성공 및 실패 주행 결과는
[혼합 맵 안내](docs/super-mixed-map.md)에 정리했습니다.

## Dasan 실측 bag 지도

카메라 RGB를 포함한 Dasan 복원 지형은 `./run_super_sim.sh dasan --overview`로
전체를 볼 수 있습니다. 기체 주변 시점은 `./run_super_sim.sh dasan`입니다.
PCD·USD·복원 궤적·카메라 색상과 재생성 절차는 [Dasan 지도 안내](docs/dasan-map.md)에 있습니다.
현재 기본 bag 경로는 약 117m 왕복입니다. [확장 경로와 시험 결과](docs/dasan-long-route.md)를 참고하세요.
외부 탐사 경로의 연속 추종·갱신 입력과 복도 내부 bag 주행 구성·실험 결과는 [Dasan 주행 안내](docs/dasan-navigation.md)에 있습니다.

## Frontier 주행과 여섯 가지 문제 해결

Frontier bag은 FAST-LIVO2로 복원하고 실제 카메라 RGB를 입힌 PCD를 Isaac Sim 지형으로 변환했습니다.
Frontier 실행에 필요한 최종 지도·임무·텍스처는 분할 압축 파일로 포함했습니다. 원본 bag과 복원 중간 파일·시험 로그는 제외합니다.
`./run_super_sim.sh frontier`의 첫 실행에서 자동으로 복원하며, Git LFS 설치는 필요하지 않습니다.
`./run_super_sim.sh frontier`는 실제 카메라 텍스처를 입힌 연속 표면 지형
`simulator/assets/super/frontier_visual/frontier_textured_flight.usda`를 기본으로 사용합니다.
화면·LiDAR·충돌 판정에 새 지형을 사용하며, 기존 500.54m 경로와 `common.yaml`은 유지합니다.
새 지형에서 203개 경유점 도착과 주행 중 충격·관통 0건을 확인했습니다.
최상층 착륙면의 방향을 수정하고 1m × 1m, 두께 2cm 패드를 추가한 뒤,
최상층만 따로 시험해 두 번 연속 PX4 착륙 감지·자동 시동 해제를 확인했습니다.
수정 후 전체 500m 경로를 재주행한 결과는 아닙니다. [최상층 착륙 검증](docs/frontier-landing.md)을 참고하세요.
trial30·31의 착륙까지 완료한 기록은 기존 복셀 지형에서의 결과입니다.
복원·자산 생성 절차는 [Frontier 지도 안내](docs/frontier-navigation.md), 최종 경로와 검증 결과는
[전 층 주행 기록](docs/frontier-all-floors.md)을 참고하세요.

자산 준비 후 터미널 두 개에서 실행합니다. 두 번째 명령은 Isaac의 `[Ready]` 출력 이후 실행하며 RViz도 함께 엽니다.

```bash
# 터미널 1: Isaac Sim + PX4
./run_super_sim.sh frontier
# 터미널 2: Super-LIO → SUPER → OMMPC → PX4, RViz 및 bag 기반 상위 경로
./run_super.sh --scenario frontier --execute
```

전 구간 시험 중 다음 여섯 가지 문제를 확인하고 수정했습니다.

1. **물리 상태와 자세의 시각 불일치**
   위치·속도는 200Hz 물리 상태를 읽으면서 자세는 20Hz 렌더링용 USD 값을 읽어 약 50ms 늦었습니다.
   IMU의 specific force와 회전율 계산에 같은 시각의 상태가 들어가도록 자세도 Dynamic Control의 물리 pose에서 읽습니다.
2. **LiDAR 획득 시각과 센서 움직임의 불일치**
   렌더 콜백에서 읽은 이전 완료 프레임의 점군에 현재 전달 시각을 붙였고, Motion BVH도 꺼져 있었습니다.
   Motion BVH를 켜고 해당 렌더 프레임의 시각으로 RTX 시계를 변환해 점별 획득 시각을 보존합니다.
   고정 30ms/50ms 보정값 대신 실제 렌더 시각을 사용하며 작은 시계 드리프트도 반영합니다.
3. **Super-LIO의 스캔 경계 IMU 적분 누락**
   LiDAR 스캔 끝이 IMU 샘플 사이에 있으면 일부 운동 적분이 누락됐습니다.
   경계 양쪽 IMU를 보간해 정확한 스캔 끝까지 적분하고, 다음 구간에 필요한 샘플은 큐에 유지합니다.
   일정 회전 재현 테스트에서 수정 전 1초 동안 약 0.018rad가 누락되는 것을 확인했고 수정 후 테스트가 통과했습니다.
4. **이동 중 재계획의 초기 상태 오류**
   `PlanFromRest`가 움직이는 기체의 초기 속도를 0으로 두거나 빈 공간의 실제 위치를 격자 중심으로 옮기는 경우가 있었습니다.
   최신 측정 위치·속도로 초기화하고, 실제 위치가 비점유이면 그 위치를 그대로 사용합니다.
5. **계획 계산으로 인한 실행 heartbeat 지연**
   같은 ROS 실행기에서 최적화가 오래 걸리면 이미 확정된 궤적의 heartbeat까지 지연됐습니다.
   확정 궤적의 heartbeat를 별도 실행기로 분리했습니다. 신선한 odometry, 주행 상태, 정지 가능한 끝점과
   궤적 유효 시간 안에서만 발행하며, 기존 제어기의 1초 timeout은 유지합니다.
6. **지도 정합 시 경로 진행 초기화와 되돌아감**
   이동 중 전체 Path를 다시 보내면 진행 상태가 초기화돼 이미 지난 경유점을 향하는 경우가 있었습니다.
   정합 결과는 기존 경로의 강체 좌표 보정으로 전달하고, 경로 revision·segment·누적 진행 거리는 유지합니다.
   미방문 복도 끝과 다음 계단 경유점을 보존합니다.

수정 후 Isaac Sim과 RViz를 켜고 **500.54m·203개 경유점을 두 번 연속 완주**했습니다.
긴 복도 4곳과 기록된 높이 구간 7곳을 방문한 뒤 최상층 자연 바닥에서 PX4 `LANDED`를 확인했습니다.
두 실행 모두 최초 주행부터 최종 착륙 전까지의 충격·관통은 0건이며 끝점 대기도 검사에 포함했습니다.
복도 단축·경로 생략·중간 강제 정지 없이 실행했고, `common.yaml`과 penalty·최대 속도·가속도·저크는 유지했습니다.
이는 기록된 Frontier 경로의 SITL 검증 결과이며 실제 EPIC 노드나 실기체를 연결한 시험은 포함하지 않습니다.

## LIO 이상 감시와 착륙

`./run_fastlivo.sh`는 짧은 측정 간격에서 위치·자세가 갑자기 튀는 운동 연속성만 감시합니다.
정상 비행에서는 기압계 융합을 끄고, 점프 확정 시 odometry와 외부 setpoint를 차단한 뒤
기압계 융합을 켜고 실제 융합·수직 상태를 확인해 PX4에 착륙을 요청합니다.
수신 지연·긴 측정 간격·LiDAR 보정 신선도·EKF2 융합 상태는 착륙 발동 기준에서 제거했습니다.
기본 임계값, 긴 간격의 비교 생략, 위치/자세 점프 실험 및 높이 추정 의존성은
[LIO safety 실험 안내](docs/lio-safety.md)를 확인하세요.

## FAST-LIVO odometry CSV 저장

시뮬레이터와 `./run_fastlivo.sh`를 실행한 상태에서 별도 터미널로 실행합니다.

```bash
./record_odometry.sh
# 저장 경로 지정
./record_odometry.sh --output log/odometry/flight.csv
# IMU 전파 odometry를 저장할 경우
./record_odometry.sh --topic /LIVO2/imu_propagate
```

기본 토픽은 `/aft_mapped_to_init`, 기본 파일은 `log/odometry/fastlivo_날짜_시간.csv`입니다.
Ctrl+C로 저장을 종료합니다. 기존 파일은 덮어쓰지 않으며 수신한 행마다 flush합니다.
CSV에는 메시지의 측정 시각(`stamp_sec`, `stamp_nanosec`, `timestamp_s`), 프레임,
위치(m), 자세 쿼터니언(x/y/z/w), 선속도(m/s), 각속도(rad/s)를 기록합니다.
좌표 변환 없이 원본 값을 저장하므로 pose는 `frame_id`, twist는 `child_frame_id` 기준입니다.
시뮬레이션의 측정 시각은 실제 날짜/시간과 다릅니다. 각속도는 발행된 값 그대로이며,
FAST-LIVO에서 채우지 않은 필드는 0일 수 있습니다. 기록기를 실행한 뒤 수신한 메시지만 저장합니다.

CSV를 저장하는 동안 별도 터미널에서 실시간 그래프를 실행할 수 있습니다. ROS 환경 설정은 필요 없습니다.

```bash
python3 simulator/tools/runtime/plot_odometry.py
# 파일을 직접 지정하거나 갱신 간격(ms)과 표시할 최대 샘플 수 조정
python3 simulator/tools/runtime/plot_odometry.py log/odometry/flight.csv --interval 200 --max-points 10000
```

파일을 생략하면 가장 최근 `fastlivo_*.csv`를 선택한 뒤 해당 파일을 계속 읽습니다.
위치 XYZ, 자세 roll/pitch/yaw(도), 선속도 XYZ, XYZ 3차원 궤적을 표시합니다.
3차원 궤적은 마우스로 드래그해 회전할 수 있으며 세 축의 거리 비율을 동일하게 유지합니다.
시간축은 첫 메시지 기준 측정 경과 시간이며, 자세는 쿼터니언에서 ZYX Euler로 변환합니다.
matplotlib이 없으면 `sudo apt install python3-matplotlib`로 설치합니다. 그래프 창을 닫으면 종료됩니다.

## 개발할 때

```text
simulator/
  zav_sim.py                 # 물리 주기·센서 설정·장면·메인 루프
  slam_test_scene.py         # SLAM 검증 장면
  pegasus/simulator/logic/   # 기체·센서·PX4/ROS 백엔드 (실제로 import하는 코드)
  pegasus/simulator/assets/  # 기체 USD·LiDAR USD
  config/                   # 공통 Pegasus 설정
  tools/                    # 센서 relay·FAST-LIVO 실행·RViz
  scripts/                  # Ubuntu/WSL Isaac·PX4 실행
src/                        # FAST-LIVO2 등 ROS 패키지
scripts/                    # 최초 설치·빌드·검증
.zav.env                    # 이 컴퓨터의 Isaac 경로·플랫폼 (Git 제외)
```

**Python 수정은 저장 후 재실행하면 적용됩니다.** 실행기가 저장소의
`simulator/zav_sim.py`를 직접 실행하고 같은 저장소의 Pegasus 코드를 import합니다.
설치 폴더로 복사하거나 deploy 명령을 실행하지 않습니다. Windows Isaac도 WSL의
동일 파일을 UNC 경로로 읽습니다. 기존 `src/PegasusSimulator`는 더 이상 사용하지 않습니다.

친구와 공유할 때는 수정한 원본을 commit/push하고 친구는 `git pull` 후 재실행합니다.
C++/ROS 메시지를 바꿨을 때만 `./scripts/build.sh`로 빌드합니다.
의존성 버전을 바꿨다면 최초 설치 명령을 다시 실행합니다.

최초 설치는 `./setup.sh`로 시작할 수 있습니다. Ubuntu/WSL을 자동 감지하고
일반적인 Isaac 경로를 찾습니다. 다른 위치에 설치했다면 위의 `--isaac-python` 또는
`--isaac-windows-python` 옵션을 지정하세요. 기존 checkout에서 업데이트한 경우에는
`python3 scripts/configure.py`를 한 번 실행해 이전 셸 등록 경로를 갱신하세요.

## 설정과 구조

- [Ubuntu 설치 상세](docs/ubuntu22.04.md), [Windows/WSL2](docs/windows-wsl2.md)
- [파라미터 적용 순서와 현재 프로파일](docs/parameters.md)
- [검증·문제 해결](docs/verification.md), [GitHub 업로드](docs/publishing.md)
- `simulator/`: 실행 코드·센서·기체 USD·Pegasus 소스·PX4 airframe
- `src/`: FAST-LIVO2, vikit, Sophus, Livox 메시지, PX4 메시지, `zav_control`
- `dependencies.lock.json`: 외부 소스 고정 버전
- `.zav.env`: 설치기가 만드는 컴퓨터별 경로. Git에 올리지 않음

`simulator/current_ekf_mpc.params`는 현재 재현 프로파일을 사람이 읽기 쉽게 정리한 파일입니다. 실제 기본값은 `simulator/px4/10020_isaac_zav`에서 설정합니다. 이 프로파일은 SITL 전용입니다.

각 외부 코드의 라이선스는 해당 파일과 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 따릅니다.
