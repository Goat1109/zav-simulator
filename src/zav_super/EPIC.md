# EPIC → SUPER hardware integration

Source: [zav-simulator / ZAVSIM_Fix](https://github.com/ZenithFuturesLab/zav-simulator/tree/ZAVSIM_Fix/src/epic_ros2), commit `36bf6ef5a5a13bb7e002d0f82a5c46953afb2cfa`. Only `epic_ros2` was imported. License and source hashes are in `epic_ros2/LICENSE` and `epic_ros2/UPSTREAM.json`. Native mapping, frontier selection and external guide generation remain upstream code; the external manual reset also accepts LAND so a new, explicit RC7 session is possible.

## Build and operate

Inside `argo_dev`, run `scripts/build_super_hardware.sh`, then source `install/setup.bash`. The web server needs a restart to load its updated Python backend; restarting it does not stop detached ROS jobs.

1. On the web page choose **gym**, **frontier whole**, or **frontier single floor**, then run **EPIC + SUPER 자율탐사** and **SUPER 브리지** while disarmed and landed before takeoff. A stationary ground pose locks the profile alignment; an airborne launch cannot acquire it. The profile is fixed for that launch; stop and relaunch on the ground to change it.
2. Take off manually in Position and establish a stationary hover. Enter actual Offboard with the existing RC8 procedure. Match the selected profile's flight height band.
3. Keep RC7 LOW or HIGH for at least 0.3 s, then place it in the middle (1400–1600 µs) for at least 0.3 s. It starts only with healthy SUPER Offboard, fresh state/cloud/RC7, fresh airborne confirmation, EPIC WAIT_TRIGGER, and a normal battery above 20%.
4. Move RC7 out of the middle to stop exploration and request a current-position hold. RC8 LOW remains the pilot's existing mode takeover. A failed start, stopped session or recovered link never restarts just because RC7 is still in the middle; move it outside and back in again.

CLI equivalent (start the existing SUPER bridge separately):

```bash
ros2 launch zav_super super_navigation.launch.py execute:=true epic_enabled:=true epic_profile:=gym
# epic_profile:=frontier_whole | frontier_single_floor
```

## Profiles and coordinates

| Web label | Original file | Source viewing Z band | Hardware profile viewing Z band |
| --- | --- | --- | --- |
| gym | gym-epic-full-local.yaml | 0.3–1.5 m | 0.3–1.5 m |
| frontier whole | frontier_bounds.yaml | unspecified | 0.3–9.5 m |
| frontier single floor | frontier-first-floor-sim.yaml | 1.2–1.9 m | 1.2–1.9 m |

The three original YAML files are preserved byte for byte. Their boundary geometry is aligned to the robot's initial ground position and heading, as requested. Gym and frontier whole use profile reference `(0,0,0)`, yaw 0°. Frontier single floor uses its documented simulation reference `(-3.26924,-0.05305,0.18920)`, yaw -172.3865°. Thus the offset first-floor boxes include the real launch pose after alignment, rather than rejecting an aircraft whose LIO starts at the origin.

`epic_frame_adapter` captures one stationary, disarmed, confirmed-ground pose with a fresh FC heartbeat. It applies a rigid yaw/translation transform to odometry, world velocity and registered cloud XYZ before EPIC, then the inverse to EPIC guides before SUPER. Body angular velocity and sensor mounting stay unchanged. The controller checks guides and local trajectories in that same fixed profile frame, plus SUPER's physical world-height limits. It also watches the transform heartbeat: a missing/changed transform stops exploration, and a changed transform requires a ground relaunch. The anchor never resets just because the pilot leaves Offboard or disarms.

For first floor the viewing band is about **1.01–1.71 m above the initial pose**, because the source reference is at Z=0.18920 m. The viewing band is managed by native EPIC; measured departure/return connectors can transition outside it while staying within the exploration boundary and physical height guards.
SUPER's virtual ceiling is 10 m and requested optimizer speed is 1 m/s. EPIC hardware guide costing also uses 1 m/s. Native profile limits are capped using the common ground plus 0.2 m and ceiling minus 0.5 m; the controller independently enforces those world-height guards after coordinate conversion. `frontier whole` includes source survey surfaces up to 30 m, but this launch cannot fly above 9.5 m.

## Data flow and FSM

`/argo/super/{odom,cloud_world}` → fixed profile-frame adapter → `/epic/profile_{odom,cloud}` → EPIC external planner → `/epic/profile_path` → inverse frame adapter → `/mission/exploration_path` → hardware controller → `/zav/super/goal` → SUPER polynomial trajectory → OMMPC → existing single MAVLink bridge.

The EPIC launch starts `exploration_node` and its TSP worker; the hardware integration also starts the coordinate adapter; no simulator, trajectory server, command adapter, FC link, arm, takeoff, or landing command is started. EPIC's unused takeoff/land output is isolated on `/epic/unused_takeoff_land`. EPIC state markers are remapped to `/epic/state`. Raw `/livox/imu` timestamps are the independent freshness reference. Planning runs at nice 10 with two OpenMP threads.

The controller publishes `/zav/epic/status` at 10 Hz: native FSM, profile, start/stop reason, blocked condition, RC7 freshness/edge readiness and current guide leg. The web subscriber marks this stale independently of FC telemetry. Controller status identifies the mission as `epic`, accepted by the existing bridge schema.

The initial hardware implementation follows the guide vertices in order, settling at each vertex before advancing. New previews are buffered rather than resetting a moving trajectory. A preview can replace the remaining route only at a stop, while fresh and connected to the current position. This is conservative stop-and-go navigation, not the simulator's continuous guide tracking.

## Failure behavior

- Profile-transform loss (0.6 s), a changed anchor, RC7 loss (0.5 s), non-middle PWM, unhealthy control/feedback, missing airborne confirmation, low/unknown battery, or measured boundary exit ends the EPIC session. EPIC's manual-reset topic is asserted, cached routes are cleared, and an EPIC mission is replaced by a current-position OMMPC hold request. Hold is possible only while the existing controller and feedback remain healthy.
- EPIC state silence beyond 1.5 s, native CAUTION/LAND, or 5 s without a new guide while waiting stops the session. FINISH/completion holds; it does not automatically land. EPIC may supply its native checked return guide before completion; it is subject to the same bounds checks.
- Guide checks reject stale/replayed/pre-session messages, wrong frame, nonfinite coordinates, invalid quaternion, oversized path, disconnected departure, long segments and segments crossing gaps between boundary boxes. An invalid live guide ends the session. SUPER trajectories are sampled against the exploration bounds before admission.
- Existing SUPER heartbeat, plan deadline, solver, odometry reset/jump, tracking and EV/FC failsafes remain in force. A failed controller cannot guarantee hover: the existing bridge/PX4 fallback owns that case. The UI reports actual FC mode independently.
- Forward/Patrol commands are blocked during EPIC. Explicit RTH can preempt it, then follows the existing Home/landing rules. Web disconnect does not terminate flight nodes.

The bounds check is a command admission/monitoring guard, not a proof that the aircraft can brake before every boundary or obstacle. Physical flight validation and measured stopping clearance remain necessary.

## Validation

`test/epic_gate.cpp` checks switch edges, debounce, loss/recovery, rejected starts and exact segment containment in a union of boxes. `test/epic_pipeline.py` exercises the ROS controller contract using synthetic inputs in a network-disabled container without serial devices. Frame unit tests cover first-floor alignment, inverse guide/odometry transforms, world velocity, body angular velocity and padded little/big-endian point clouds. The ROS contract test also runs through the real frame adapter, including refusal to anchor in flight and transform loss. Native EPIC tests and existing SUPER/web tests are also run. An additional device-free test runs the actual EPIC node with the actual profile adapter on a synthetic room, using a 90° frame rotation and translation, and verifies that its generated guide returns to hardware world coordinates. These tests do not establish real-aircraft safety or a full PX4 flight simulation.
