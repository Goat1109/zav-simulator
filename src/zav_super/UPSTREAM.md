# Hardware SUPER provenance

Source requested by the user:
https://github.com/Goat1109/zav-simulator/tree/argo_v2

Locked commit: `ef2788ff1f99431e67b79093ac3b7b0bfe269a0b`.

`upstream_manifest.json` records upstream and local SHA-256 for each imported
file. `config/common.yaml` originates from that commit's
`src/argo_super/config/common.yaml` and contains local hardware tuning;
`config/argo_v1.json` is identical to
`simulator/config/vehicles/argo_v1.json`. The vehicle JSON is reference metadata;
the hardware wrapper uses the existing calibrated `hardware.yaml` mount.

Imported `src/SUPER` and `src/ommpc_core` algorithm sources are unchanged, with
these local adaptations:

* `src/SUPER/super_planner/include/ros_interface/ros2/fsm_ros2.hpp`: the committed
  trajectory heartbeat's child node inherits the parent's `use_sim_time` instead
  of forcing simulated time. Hardware uses system ROS time; the simulator clock
  remains supported when explicitly selected by a simulator parent.
* `src/ommpc_core/CMakeLists.txt`: link the pinned OSQP static library rather than
  requiring a global runtime shared-library installation. Solver C++ is unchanged.
* The hardware copy of `trajectory.hpp` retains polynomial decoding and verified
  terminal-stop logic, and removes unused Isaac rotor/thrust inversion.

* Planner latency protection (2026-11-24): removed an unused second backup
  optimization; added cooperative steady-clock deadlines and iteration limits
  to L-BFGS and its trajectory callers. Backup optimization shares a 30 ms
  budget between at most two initial candidates, with jerk-aware stopping-time
  initialization. Existing corridor and dynamics acceptance checks remain.
* Initial plans share a 100 ms cycle budget and are checked again immediately
  before publication. Expired results retain the goal in GENERATE_TRAJ and are
  retried from refreshed robot state. Replans share 90% of replan_forward_dt.
  These are cooperative compute budgets, not hard real-time guarantees; an
  individual cost evaluation or OS pause can overrun a deadline, after which
  the result is rejected.

* Endpoint convergence (2026-11-25): the hardware configuration shares a 0.10 m
  position and 0.10 m/s speed arrival threshold between planner and controller.
  Polynomial `debug_info` carries the map-adjusted goal and the original request
  timestamp; the wrapper rejects mismatched requests and excessive goal shifts.
  A stopping reference near its accepted goal keeps its endpoint while converging
  for up to 3 seconds, rather than immediately generating a short correction.
  Arrival must remain stable for 1 second. New goals preempt convergence; stale
  localization or occupied endpoints are not certified. Timeout, distant backup
  stops and non-stopping references retain recovery planning. The convergence
  option defaults to disabled for other SUPER configurations. OMMPC gains and
  trajectory optimization limits are unchanged.

New hardware code is in `zav_super`, `zav_real_control/super_adapter.py` and
`super_gate.py`. The launch/bridge/state integration is also local. No simulator
arm/takeoff/land, PX4 DDS agent, vehicle spawning, ground truth, clock publisher,
is imported into this path. EPIC exploration is now available as a separate
`epic_ros2` package from the same simulator branch; see [EPIC hardware integration](EPIC.md)
and its own `UPSTREAM.json`. Hardware exploration changes the common maximum
speed to 1 m/s and virtual ceiling to 10 m; OMMPC gains remain unchanged.

OSQP dependency: https://github.com/osqp/osqp, v0.6.3,
`0dd00a578cf1c2691c5c379965d504c75bf6cfad`, including its pinned QDLDL submodule.
The build script installs it only under this workspace's `.deps/install`.

Keep the imported per-file copyright/license notices and
`src/SUPER/COPYING`, `COPYING.LESSER`, and `UPSTREAM.md`. The latter describes
the simulator repository's earlier changes relative to hku-mars/SUPER. This
hardware port does not override third-party terms or assert a new license for
the upstream OMMPC core.
