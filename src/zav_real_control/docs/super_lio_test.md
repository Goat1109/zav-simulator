# Super-LIO / Mid360s 지상 비교

공식 저장소: https://github.com/Liansheng-Wang/Super-LIO (ros2 브랜치)
고정한 초기 커밋: `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`

## 빌드 (argo_dev 컨테이너 내부)

```bash
source /opt/ros/humble/setup.bash
source /root/argo_ws/install/setup.bash
cd /root/argo_ws
# 새 환경에서만 의존성 설치:
# apt-get update && apt-get install -y libgoogle-glog-dev libtbb-dev
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select basic super_lio zav_real_control --executor sequential \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=119
```

## 단독 실행

기존 FAST-LIVO launch를 종료하고 프로펠러 제거·비무장 상태에서 시작한다.
먼저 bridge 없이 출력·축을 확인한다. 시작 후 초기화 동안 기체를 정지한다.

```bash
ros2 launch zav_real_control sensors_super_lio.launch.py
```

드라이버가 이미 실행 중이면 중복 실행을 피한다:

```bash
ros2 launch zav_real_control sensors_super_lio.launch.py start_driver:=false
```

RViz를 함께 켜려면 `rviz:=true`를 붙인다. 이때 다운샘플된 클라우드 출력도 켜진다.
성능 비교에서는 RViz를 끈다. 카메라는 사용하지 않는다.

## 출력 확인 (별도 터미널, 동일 setup/ROS_DOMAIN_ID)

```bash
ros2 topic hz /super_lio/odom
ros2 topic hz /super_lio/imu/odom
ros2 topic echo /super_lio/odom --once
```

- `/super_lio/odom`: LiDAR 업데이트 후 IMU 원점 pose와 월드 좌표계 속도.
- `/super_lio/imu/odom`: IMU 콜백의 전파 pose. 메시지 주기가 빠르다는 것이
  LiDAR 보정 주기 또는 연산시간이 짧다는 뜻은 아니다. upstream의 twist/공분산은 FC용으로 검증하지 않았다.
- 연산시간: `lio.eva.timer: true`로 실행 후 Ctrl+C 종료하면
  Undistort/DownSample/Observe/UpdateMap 평균 처리시간(ms)이 로그에 출력된다.
  upstream에 processing_time 발행 함수는 있으나 현재 처리 경로에서 호출하지 않는다.
- `/super_lio/cloud_world`: `rviz:=true`에서 출력.
- `/super_lio/tf`: 별도 TF 스트림. 기존 FAST-LIVO TF와 분리.

## 비교 순서

1. 정지 상태 1분: pose 흔들림, 출력 공백과 CPU/RSS 기록.
2. 자로 30cm 상승 → 잠시 유지 → 원위치: Z 크기·부호·복귀 오차 비교.
3. 전후/좌우 이동, 90도 yaw 후 복귀: 축과 회전 방향 확인.
4. 같은 장소·동작으로 FAST-LIVO LIO와 각각 실행해 비교.
   동시에 실행하면 CPU 경쟁 때문에 성능 비교가 왜곡될 수 있다.
5. 10분 실행으로 출력률, 최대 공백, RSS 증가 비교.

설정: `config/super_lio_mid360s.yaml`. LiDAR/IMU 토픽은 기존과 동일하고
blind=0.5m 및 LiDAR-in-IMU 외부변환은 현 zenith.yaml에서 가져왔다.
Super-LIO 공식 Mid360 기본값은 Z=+0.04412m이지만 현재 FAST-LIVO의 값은
Z=-0.04412m이므로 현 장비 설정을 사용했다. 이 값은 실물/교정 데이터로 추가 검증해야 한다.
나머지 추정기 파라미터는 공식 Mid360 기본값을 기준으로 한다.

기본값 `adapt_odometry:=true`는 `/super_lio/imu/odom`의 월드 기준 선속도를
IMU 기준으로 변환하고 기존 bridge 입력 `/zav/lio/odometry`를 발행한다.
측정 시각과 IMU 기준 각속도는 보존하고 frame을 camera_init/mid360_imu로 명시한다.
기존 bridge가 동일한 장착 회전과 lever-arm을 한 번만 보정한다.
LiDAR 보정 시각과 IMU 측정 시각 차이 또는 LiDAR 보정 수신 공백이 0.25초를 넘으면
전달을 멈춘다. 이후 FC/bridge의 기존 EV 손실 처리를 따른다.
이 제한은 순수 IMU 전파가 새로운 LiDAR 보정으로 오인되는 것을 막는다.
같은 출력에 다른 추정기 publisher가 감지되면 변환 출력을 차단한다.
FAST-LIVO와 Super-LIO를 동시에 같은 bridge에 연결하지 않는다.
고립 비교만 원하면 `adapt_odometry:=false`를 추가한다.

연결 확인 명령(모든 터미널에 같은 setup과 ROS_DOMAIN_ID):

```bash
# 터미널 1: super_lio alias도 동일
ros2 launch zav_real_control sensors_super_lio.launch.py
# 터미널 2: 기존 bridge alias. FC로 실제 EV/실패 대응 명령 전송 활성화.
ros2 launch zav_real_control rc_flight.launch.py publish:=true
# 터미널 3: 기존 state alias
ros2 run zav_real_control inspect_status
```

이 명령 자체는 시동 명령이 아니다. estimator/adapter는 비무장 상태에서 먼저
시작하고 정상 데이터 전달을 확인한다. 무장 중 estimator/adapter를 재시작하면
원점이 달라질 수 있으므로 금지한다. 브리지의 공분산과 FC guard 파라미터는
이번 작업에서 변경하지 않았다. 실물에서 30cm 각 축 이동·yaw/roll/pitch·정지
드리프트·지연 및 비무장 EV 손실 동작을 확인한 뒤 비행 여부를 판단한다.
소프트웨어 연결 완료는 비행 준비 완료를 의미하지 않는다.
종료는 Ctrl+C.

## 도입 검증

- argo_dev / Humble / aarch64에서 basic, super_lio, zav_real_control 빌드 성공.
- 별도 ROS_DOMAIN_ID=217에서 합성 정지 IMU·실내 평면 포인트 입력으로
  KF/맵 초기화, LiDAR odometry 101개·IMU odometry 875개 발행 및 정상 종료 확인.
- pose 값은 모두 유한값. 합성 데이터에서 최대 위치 노름은 0.236m로,
  이 검사는 입력/출력 연결 확인용이며 실물 센서의 정확도 검증은 아니다.
- 실측 이동, 외부변환, 지연 및 FC 연동은 아직 검증하지 않았다.
