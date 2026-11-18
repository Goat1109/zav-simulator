# EPIC 연동 검토: 경로와 시간 궤적

2026-11-17, 공식 EPIC `d73c3150e57d669ac21bcad5c859f52c2827c9ba`를 읽고 현재 저장소와 비교했다. 사본은 `log/epic-interface-review`이며 EPIC을 빌드하거나 비행 입력으로 연결하지 않았다.

**현재 bag 경유점 추종은 EPIC의 시간 궤적 추종과 동등하지 않다.** EPIC 결과를 XYZ 점 또는 click goal로만 바꾸면 속도·가속도·yaw·시작 시각의 의미가 사라진다. 중간점마다 정지시키거나 끝점만 남기는 방식은 미래 연동 기준으로 채택하지 않는다.

## 확인한 EPIC 출력

공식 소스:

- [메시지 PolyTraj](https://github.com/Robotics-STAR-Lab/EPIC/blob/d73c3150e57d669ac21bcad5c859f52c2827c9ba/src/local_planner/Utils/traj_utils/msg/PolyTraj.msg)
- [직렬화: planner_manager.cpp](https://github.com/Robotics-STAR-Lab/EPIC/blob/d73c3150e57d669ac21bcad5c859f52c2827c9ba/src/local_planner/minco_planner/src/planner_manager.cpp#L722)
- [발행 토픽/FSM](https://github.com/Robotics-STAR-Lab/EPIC/blob/d73c3150e57d669ac21bcad5c859f52c2827c9ba/src/global_planner/exploration_manager/src/fast_exploration_fsm.cpp#L207)
- [궤적 평가/제어 명령: traj_server.cpp](https://github.com/Robotics-STAR-Lab/EPIC/blob/d73c3150e57d669ac21bcad5c859f52c2827c9ba/src/local_planner/minco_planner/src/traj_server.cpp)

| 항목 | EPIC | 현재 SUPER/OMMPC 연결 |
|---|---|---|
| ROS | ROS1/catkin, README 기준 Noetic | ROS2 Humble |
| 위치 궤적 | `/planning/trajectory`, `traj_utils/PolyTraj`, 7차 | `/planning_cmd/poly_traj`, `mars_quadrotor_msgs/PolynomialTrajectory`, 7차 |
| yaw 궤적 | `/planning/yaw_trajectory`, 같은 메시지, 5차, x 계수에 yaw | 같은 통합 메시지 내 yaw 계수, decoder는 현재 7차만 허용 |
| 시간 | `start_time`, 구간별 `duration` | `start_wt_pos`, `start_wt_yaw`, 구간별 시간 |
| 갱신 식별 | `drone_id`, `traj_id` | `trajectory_id`, position/yaw/type bit |
| 생존 확인 | 별도 `/planning/heartbeat` | 통합 메시지의 HEART_BEAT bit |
| 좌표계 명시 | PolyTraj에는 Header/frame_id 없음 | `header.frame_id == world` 필요 |

EPIC은 자체 local planner가 생성한 궤적을 출력한다. 탐사 시 yaw는 진행 방향과 다를 수 있으므로 `atan2(vy,vx)`로 대체해서는 안 된다.

## 권장 연결과 SUPER의 역할

EPIC이 만든 **위치·yaw 시간 궤적 자체를 따라야 한다면**:

```text
EPIC position/yaw polynomial + heartbeat
  → ROS1/ROS2 전달 및 검증/변환 adapter
  → 동일한 timed polynomial controller 입력
  → OMMPC → PX4 → 기체
```

이 모드의 궤적 생성자는 EPIC이다. SUPER planner를 동시에 같은 controller 토픽의 writer로 두지 않는다. SUPER planner를 우회한다는 점을 숨기지 않으며, 현재 Dasan SUPER 실험을 이 모드의 검증으로 표시하지 않는다.

**EPIC 탐사 기능과 SUPER local planner를 모두 사용하려면** 연결 위치를 더 위로 올려야 한다. EPIC의 탐사 목표/전역 경로(예: `global_tour_`, `path_next_goal_`)를 넘기고 SUPER가 실제 실행 궤적을 다시 생성하도록 한다. 그러면 EPIC 원래 시간 궤적을 그대로 추종하는 것이 아니며, 탐사 yaw 목표를 SUPER 재계획 시각에 맞춰 전달하는 추가 인터페이스가 필요하다. 두 planner를 직렬로 붙이고 원래 위치/yaw/time가 보존된다고 가정하지 않는다.

bag 실험에서는 시간 없는 여러 XYZ 경유점을 `PolylineFollower`에 주고, 진행 순서를 유지하면서 앞쪽 경로를 보간한 목표를 SUPER에 전달한다. 중간점 정지 없이 통과하는 기능 검증용이다. 현재 click-goal frontend는 전체 polyline을 하드 제약으로 받지 않으므로 모든 경유점을 정확히 통과하거나 EPIC의 시간 계획을 보존한다고 보장하지 않는다. 향후 EPIC 입력을 이 클래스에 억지로 끼워 넣지 않는다.

## adapter 구현 시 필요한 사항

1. 위치 7차는 계수와 구간 시간을 유지한다. yaw 5차는 7차 계수 행렬의 상위 두 차수에 0을 붙여 정확히 표현할 수 있다. 계수 방향은 양쪽 evaluator와 표본 p/v/a/j/yaw/yaw-rate로 검증해야 한다. 임의 재피팅은 하지 않는다.
2. position/yaw를 동일 drone·traj ID와 start_time으로 맞춘 뒤 전환한다. 서로 다른 재계획 세대가 섞이거나 한쪽만 늦게 도착하는 경우를 처리한다. 현재 controller callback은 POSITION_TRAJ 없는 yaw-only 메시지를 적용하지 않으므로 단순 topic remap으로 충분하지 않다.
3. ROS time/시뮬레이션 time와 wall/monotonic time를 구분한다. 시작 시각을 수신 시각으로 덮어쓰지 않는다. heartbeat는 실제 EPIC 메시지의 신선도를 검사해야 하며 adapter 자체 timer로 거짓 생존 신호를 만들지 않는다.
4. 입력 좌표계를 설정/검증한다. EPIC 메시지에는 frame_id가 없으므로 world라고 추정해서는 안 된다. 위치·미분·yaw에 같은 변환과 단위를 적용한다.
5. 재계획 전환 시 p/v/a와 yaw/yaw-rate의 연속성, 과거/미래 궤적, 양쪽 궤적 만료, 기체 한계 위반을 확인한다. 현재 OMMPC의 yaw/body-rate 제한은 실제 추종 한계다. 임의 yaw 궤적을 완벽히 따라간다고 보장하지 않는다.
6. 시간 없는 waypoint 입력과 시간 있는 trajectory 입력을 명시적으로 구분한다. 연속 탐사 중 궤적 한 조각이 끝났다고 임무 완료/착륙으로 처리하지 않는다. 이륙/탐사 종료/중지/착륙은 별도 상태로 둔다.

필요 검증은 ROS 메시지 변환 전후의 수치 평가 일치, ID 쌍/갱신 순서, 시간 점프·지연·heartbeat 단절, ±π yaw 연결, 서로 다른 position/yaw 구간 길이, 곡선·연속 재계획·최종 정지에 대한 실제 Isaac/PX4 실험이다. 현재는 소스 검토와 bag waypoint 추종 단위검사 단계이며 EPIC 호환 완료가 아니다. penalty와 속도·가속도·저크 한계를 바꾸는 작업은 하지 않았다.

## 채택한 방향과 현재 구현

사용자가 **EPIC 상위 탐사 경로 → SUPER → OMMPC → PX4**를 선택했다. 따라서 EPIC 최종 다항식 궤적을 controller에 직접 연결하는 안은 이번 구현 대상이 아니다.

ROS2 수신 계약 `/mission/exploration_path` (`nav_msgs/Path`, world XYZ 및 선택적 orientation yaw)를 추가했다. 동일 경로 재전송 시 진행 보존, 현재 위치 부근에서 시작하는 새 경로 교체, 국소 진행 투영, 제동 거리 기반의 움직이는 lookahead, ±π yaw 보간, 외부 경로 완료 시 WAITING_PATH 상태를 구현했다. `goal_yaw_en`을 켜서 SUPER의 yaw 재계획 입력으로 전달한다. 입력 경로의 시간표를 강제하지 않는다.

이 인터페이스는 향후 EPIC의 상위 `path_next_goal_` 등에서 경로와 탐사 방향을 추출하는 adapter의 목적지다. **ROS1 EPIC 본체와의 실제 연결은 아직 하지 않았다.** source bridge에서 frame 변환·발행 시점·yaw 의미와 경로 갱신 계약을 맞추는 검증이 남아 있다. 현재 bag 발행기는 외부 입력의 대역이며 EPIC 자체의 탐사 성능 검증이 아니다. 실행 명령과 GUI 시험 기록은 [Dasan 주행 안내](dasan-navigation.md)를 따른다.

구체적인 EPIC 연결 후보는 `fast_exploration_manager.cpp`의 `ed_->path_next_goal_` 위치 경로와 선택된 viewpoint의 `yaw_`/`local_data_.end_yaw_`다. 위치 경로만 추출해서 yaw가 자동으로 포함된다고 가정하지 않는다. EPIC의 local planner와 traj_server 발행은 이 SUPER 모드에서 controller writer로 사용하지 않아야 한다.


ROS2 수신부의 실제 검증은 Dasan GUI에서 수행했다. trial10/11은 경로 재전송·실시간 suffix 교체·왕복·명시적 착륙이 성공했고, trial11은 정지 확인 hold 후 8초 대기도 성공했다. preview 목표가 뒤로 후퇴하는 경우를 수정한 뒤 trial13 기본 파일 임무가 COMPLETED + LANDED로 종료됐다. 이 결과는 동일 설정의 모든 환경 성공이나 EPIC 본체 탐사 성공을 뜻하지 않는다. 실제 고도 편차/가속도·저크 수치는 Dasan 안내에 남겼다.
