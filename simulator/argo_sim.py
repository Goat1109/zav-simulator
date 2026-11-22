#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Argo v1/v2 simulation with shared PX4 SITL and Pegasus sensor backends.

Aircraft-specific mass, inertia, geometry and mounts live in
simulator/config/vehicles/argo_v1.json and argo_v2.json.
Physics/IMU: 200 Hz. Rendering and graphical sensors: 20 Hz.
"""

import os
import sys
from pathlib import Path
# Resolve project paths from this script, independent of the launch directory.
_SIMULATOR_DIR = Path(__file__).resolve().parent
_WORKSPACE_DIR = _SIMULATOR_DIR.parent
sys.path.insert(0, str(_SIMULATOR_DIR))

# [ARGO 변경] Windows에만 필요한 내부 ROS DLL 경로입니다. Linux 라이브러리는 실행기가 설정합니다.
# Windows python.bat sets ISAAC_PATH but does not add the ROS 2 runtime DLLs.
# This must happen before SimulationApp/bridge startup, including its child check process.
_ros_dll_handles = []
if sys.platform == "win32":
    _isaac_root = os.environ.get("ISAAC_PATH")
    if not _isaac_root:
        raise RuntimeError("Run this script with Isaac Sim python.bat (ISAAC_PATH is missing).")
    _ros_lib = os.path.join(
        _isaac_root, "exts", "isaacsim.ros2.bridge", "humble", "lib"
    )
    if not os.path.isdir(_ros_lib):
        raise FileNotFoundError(f"Isaac Sim internal ROS 2 libraries not found: {_ros_lib}")
    os.environ["ROS_DISTRO"] = "humble"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["PATH"] = _ros_lib + os.pathsep + os.environ.get("PATH", "")
    _ros_dll_handles.append(os.add_dll_directory(_ros_lib))
    print(f"[ROS2] Windows internal Humble runtime: {_ros_lib}", flush=True)

from isaacsim import SimulationApp

# 정상 작동하던 기존 설정으로 복원
simulation_app = SimulationApp({"headless": os.environ.get("ARGO_HEADLESS", "0") == "1", "sync_loads": False, "enable_motion_bvh": True})

# 초기화 중 확장 모듈 로딩 충돌 여부 확인
for _ in range(5):
    simulation_app.update()

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import get_active_viewport

print("[INIT] RTX extension enabling...", flush=True)
enable_extension("isaacsim.sensors.rtx")

for _ in range(5):
    simulation_app.update()

print("[INIT] ROS2 extension enabling...", flush=True)
enable_extension("isaacsim.ros2.bridge")

for _ in range(5):
    simulation_app.update()

print("[INIT] ROS2 importing...", flush=True)
import rclpy
print("[INIT] ROS2 OK", flush=True)

import carb.input
import carb.settings
import omni.appwindow
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
import omni.isaac.core.utils.prims as prim_utils

import numpy as np
from scipy.spatial.transform import Rotation

# Pegasus Simulator 핵심 API
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotors import ArgoV1, ArgoV2
from vehicle_profiles import load_profile, sensor_calibration
VEHICLE = load_profile()
from pegasus.simulator.params import ASSET_PATH, SIMULATION_ENVIRONMENTS

# 장착 위치는 body 기준 미터, 방향은 Z/Y/X 순서 degree.
# 측정 거리·채널·스캔 속도는 USD의 OmniLidar 속성을 사용합니다.
# [ARGO 변경] 기본 기체/추력은 유지하고 센서 획득 주기만 정수 물리 스텝에 맞춥니다.
PHYSICS_HZ = 200.0  # Mid360 IMU rate; exactly 10 physics ticks per render frame.
RENDER_HZ = 20.0
# SLAM_TEST_SCENE = os.environ.get("PEGASUS_SLAM_SCENE", "1") != "0"  # 선택적 SLAM 검증 장면.
SLAM_TEST_SCENE = False

# [ARGO 변경] 새 센서는 이 설정을 ArgoV1에 주입합니다. 기존 PX4 센서 목록은 교체하지 않습니다.
MID360_CONFIG = {
    "usd_path": str(Path(ASSET_PATH) / "Sensors" / "Mid-360s.usda"),
    "sensor_prim_path": "/World/Example_Rotary",
    # Aircraft-specific virtual scan origin and body mounting orientation.
    "position": VEHICLE["lidar_position"],
    "orientation": VEHICLE["lidar_orientation"],  # Z/Y/X degrees
    "frequency": RENDER_HZ,
    "show_render": False,
    "publish_laserscan": False,
    "publish_method": "python",
    "tcp_port": 17441,
    "livox_tcp_port": 17445,
    "livox_scan_lines": 4,
}

# Use the same selected calibration as every estimator and the PX4 bridge.
_sensor_calibration = sensor_calibration(VEHICLE)
MID360_CONFIG["livox_scan_lines"] = _sensor_calibration["scan_lines"]

FRONT_CAMERA_CONFIG = {
    # Relative pose is determined by FAST-LIVO Rcl/Pcl, not CAD mesh bounds.
    "position": _sensor_calibration["camera_position"],
    "orientation": _sensor_calibration["camera_orientation"],
    "resolution": _sensor_calibration["resolution"],
    "intrinsics": _sensor_calibration["intrinsics"],
    "distortion_coefficients": _sensor_calibration["distortion"],
    "frequency": RENDER_HZ,  # Must divide RENDER_HZ; currently one image per render frame.
    "depth": False,
    "publish_method": "python",
    "tcp_port": 17443,
}

MID360_IMU_CONFIG = {
    "position": _sensor_calibration["imu_position"],
    "orientation": _sensor_calibration["imu_orientation"],
    "update_rate": PHYSICS_HZ,  # One measurement per physics tick, independent of rendering.
    "frame_id": "mid360_imu",
    "topic": "mid360/imu",
    "tcp_port": 17444,
    "gyro_noise_density": 0.0,  # Ideal model; not calibrated Livox noise parameters.
    "accel_noise_density": 0.0,
}

class ARGOPlaneApp:
    def __init__(self):
        # 1. 시뮬레이션 타임라인 인터페이스 획득
        self.timeline = omni.timeline.get_timeline_interface()
        
        # 2. 물리/IMU 200 Hz, 센서 렌더링 20 Hz로 설정합니다.
        self.pg = PegasusInterface()
        self.pg._world_settings = {
            **self.pg._world_settings,
            "physics_dt": 1.0 / PHYSICS_HZ,
            "rendering_dt": 1.0 / RENDER_HZ,
        }
        # Limit wall-clock rendering too, rather than only increasing simulated dt.
        settings = carb.settings.get_settings()
        # Share viewport navigation preferences without copying machine/user state.
        import json
        viewport_settings = json.loads((_SIMULATOR_DIR / "config/viewport.json").read_text())["settings"]
        for key, value in viewport_settings.items():
            settings.set(key, value)
        settings.set("/app/runLoops/main/rateLimitEnabled", True)
        settings.set("/app/runLoops/main/rateLimitFrequency", int(RENDER_HZ))
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        print(f"[Rates] physics={PHYSICS_HZ:g} Hz, render={RENDER_HZ:g} Hz, "
              f"RGB={FRONT_CAMERA_CONFIG['frequency'] if FRONT_CAMERA_CONFIG else 0:g} Hz", flush=True)

        # 3. 초경량 기본 평면 바닥(Ground Plane) 및 완충 물리 물성 설정
        #    - 원리 (충돌 역학 / Rule 2 상세 주석):
        #      restitution=0.0 (완전 비탄성 충돌): 바닥 충돌 에너지를 흡수하여 착륙 시 기체가 위로 튀는 현상(Bouncing) 원천 방지
        #      static_friction=0.8, dynamic_friction=0.6: 접지 시 미끄러짐 방지 및 안정적인 그립 확보
        # print("[Terrain] 충격 흡수 완충 평면(Damped Ground Plane) 생성 중...")
        # self.world.scene.add_default_ground_plane(
        #     static_friction=0.8,
        #     dynamic_friction=0.6,
        #     restitution=0.0
        # )

        scenario = os.environ.get("ARGO_SUPER_SCENE", "warehouse")
        if scenario in ("high_speed", "dense", "mixed", "dasan", "frontier"):
            from tools.super.pcd_scene import add_super_scene
            add_super_scene(self.world, scenario)
        elif scenario == "warehouse":
            # Full Warehouse 환경 로드
            warehouse_path = SIMULATION_ENVIRONMENTS["Full Warehouse"]
            print(f"[Terrain] Loading Full Warehouse: {warehouse_path}", flush=True)
            self.pg.load_asset(warehouse_path, "/World/layout")

            # Pump Kit before creating sensors/resetting physics so referenced assets
            # and textures can finish loading while the window remains responsive.
            import time
            loading_started = time.monotonic()
            next_report = loading_started
            for _ in range(10):
                simulation_app.update()
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                simulation_app.update()
                now = time.monotonic()
                if now >= next_report:
                    print(f"[Terrain] Loading assets: {omni.usd.get_context().get_stage_loading_status()}", flush=True)
                    next_report = now + 5.0
                if now - loading_started > 300:
                    raise RuntimeError(f"Full Warehouse asset loading timed out: {warehouse_path}")
            from pxr import Usd, UsdGeom
            layout = self.world.stage.GetPrimAtPath("/World/layout")
            mesh_count = sum(
                1 for prim in Usd.PrimRange(layout)
                if prim.IsA(UsdGeom.Mesh)
            )
            if mesh_count == 0:
                raise RuntimeError(f"Full Warehouse contains no meshes: {warehouse_path}")
            print(f"[Terrain] Full Warehouse loaded ({mesh_count} meshes)", flush=True)

        else:
            raise ValueError(f"Unknown ARGO_SUPER_SCENE: {scenario}")

        # 시각적 가시성을 위한 직사광(Distant Light) 생성
        stage = self.world.stage
        if not stage.GetPrimAtPath("/World/defaultDistantLight"):
            prim_utils.create_prim(
                "/World/defaultDistantLight",
                "DistantLight",
                attributes={
                    "inputs:intensity": 3000.0,
                    "inputs:angle": 0.53,
                    "inputs:color": (1.0, 1.0, 1.0)
                }
            )

        # Indoor roofs block the distant stage light. Use the viewport's camera
        # light for inspection; mesh colors, collision and LiDAR stay unchanged.
        if scenario in ('dasan','frontier') and os.environ.get('ARGO_HEADLESS','0')!='1':
            from omni.kit import commands as viewport_commands
            viewport_commands.execute('SetLightingMenuMode',lighting_mode='camera')

        # A plane alone cannot constrain horizontal LiDAR odometry.
        # Set PEGASUS_SLAM_SCENE=0 when using an existing structured environment.
        if SLAM_TEST_SCENE:
            from slam_test_scene import add_slam_test_scene
            add_slam_test_scene(stage)

        # 4. ARGO 커스텀 쿼드콥터 스폰 (ArgoV1 공식 프리셋 클래스 사용)
        #    - 내부적으로 argo_v1.usd 자동 바인딩
        #    - PX4 MAVLink Backend (TCP 4560) 자동 장착
        #    - PX4 북쪽(NED North) 정렬을 위한 Yaw=+90도 자동 적용
        print(f"[Spawn] {VEHICLE['name']} 프리셋으로 기체 생성 중...")
        spawn=[0.0, -50.0, 0.10] if scenario in ("high_speed", "dense", "mixed") else [0.0,0.0,0.10]
        spawn_orientation=None
        if scenario in ('dasan','frontier'):
            import json
            recipe_path=_SIMULATOR_DIR/f'assets/super/{scenario}_recipe.json'
            mission_path=Path(os.environ.get('ARGO_SUPER_MISSION_FILE',
                str(_SIMULATOR_DIR/f'assets/super/{scenario}_mission.json'))).expanduser().resolve()
            recipe=json.loads((mission_path if mission_path.exists() else recipe_path).read_text())
            if os.environ.get('ARGO_SUPER_MISSION_FILE'):
                import hashlib
                if not mission_path.is_file() or recipe.get('map_sha256') != hashlib.sha256(
                        (_SIMULATOR_DIR/f'assets/super/{scenario}.pcd').read_bytes()).hexdigest():
                    raise ValueError('Custom spawn mission is missing or does not match the measured map')
            spawn=recipe['spawn']
            spawn_orientation=Rotation.from_euler('z',recipe['spawn_yaw']).as_quat()
        spawn=list(spawn)
        spawn[2]+=VEHICLE['spawn_z_offset']
        vehicle_class=ArgoV2 if VEHICLE['name']=='argo_v2' else ArgoV1
        self.drone = vehicle_class(
            stage_prefix="/World/quadrotor",
            vehicle_id=0,
            init_pos=spawn,
            init_orientation=spawn_orientation,
            lidar_config=MID360_CONFIG,
            camera_config=FRONT_CAMERA_CONFIG,
            imu_config=MID360_IMU_CONFIG,
        )
        print("[LiDAR] PointCloud2: /drone0/mid360/pointcloud (frame: mid360)")

        # True mounted-IMU pose, in the same simulation clock as FAST-LIVO input.
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        self._scenario_type = String
        self._scenario_name = scenario
        self._truth_type = Odometry
        self._truth_node = rclpy.create_node("argo_ground_truth")
        self._scenario_pub = self._truth_node.create_publisher(String, "/argo/sim/scenario", 1)
        self._truth_pub = self._truth_node.create_publisher(
            Odometry, "/drone0/ground_truth/odometry", 100)
        self._imu_offset = np.asarray(MID360_IMU_CONFIG["position"])
        self._imu_rotation = Rotation.from_euler(
            "ZYX", MID360_IMU_CONFIG["orientation"], degrees=True)

        # Optional observation only: no collision shape, controller or rate change.
        self._contact_diagnostics = None
        contact_path = os.environ.get("ARGO_CONTACT_REPORT_PATH")
        if contact_path:
            from tools.super.contact_diagnostics import ContactDiagnostics
            self._contact_diagnostics = ContactDiagnostics(
                self.world.stage, self.drone._stage_prefix,
                lambda: self.world.current_time, contact_path)
            print(f"[Contacts] Buffered PhysX reports: {contact_path}", flush=True)


        # 5. Separate chase camera: full-body framing and wall-aware close-up.
        self.zoom_scale = 1.0
        self._chase_camera_path = "/World/ChaseCamera"
        self._camera_view_initialized = False

        # Omniverse 마우스 스크롤 이벤트 구독 등록
        self._app_window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._mouse = self._app_window.get_mouse()
        self._mouse_sub = self._input.subscribe_to_mouse_events(self._mouse, self._on_mouse_event)

        # 6. 물리 시뮬레이션 환경 리셋 (Articulation 및 관절 초기화)
        print("[INIT] Resetting world...", flush=True)
        self.world.reset()
        # Keep the user-controlled Perspective camera separate from the tracker.
        from pxr import Usd, UsdGeom, Gf
        from omni.physx import get_physx_scene_query_interface
        from tools.runtime.chase_camera import ChaseCameraRig
        self._chase_camera = UsdGeom.Camera.Define(self.world.stage, self._chase_camera_path)
        self._chase_camera.CreateHorizontalApertureAttr(36.)
        self._chase_camera.CreateClippingRangeAttr(Gf.Vec2f(.03, 10000.))
        visual = self.world.stage.GetPrimAtPath(f"{self.drone._stage_prefix}/body/visual_mesh")
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(visual).ComputeAlignedRange()
        size = np.asarray(bounds.GetSize())
        rotation = Rotation.from_quat(self.drone.state.attitude)
        self._camera_body_offset = rotation.inv().apply(
            np.asarray(bounds.GetMidpoint()) - np.asarray(self.drone.state.position))
        self._chase_rig = ChaseCameraRig(get_physx_scene_query_interface(),
                                       self.drone._stage_prefix,
                                       np.linalg.norm(size) * .5 + .04,
                                       size[2] * .5)
        # Align the initial view with an edge of the takeoff pad, rather than
        # its independently authored vehicle spawn yaw. Preserve this relative
        # offset while following later turns; do not rotate the vehicle or map.
        self._camera_yaw_offset = 0.
        pad_prim = self.world.stage.GetPrimAtPath("/World/layout/floor")
        if pad_prim and pad_prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(pad_prim).GetPointsAttr().Get()
            if points is not None and len(points) >= 2:
                transform = UsdGeom.Xformable(pad_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                edge = np.asarray(transform.Transform(Gf.Vec3d(*points[1]))) - np.asarray(
                    transform.Transform(Gf.Vec3d(*points[0])))
                if np.linalg.norm(edge[:2]) > 1.e-6:
                    pad_yaw = np.arctan2(edge[1], edge[0])
                    initial_rotation = (Rotation.from_quat(spawn_orientation)
                                        if spawn_orientation is not None
                                        else Rotation.from_euler("z", 90., degrees=True))
                    initial_forward = initial_rotation.apply([1., 0., 0.])
                    initial_yaw = np.arctan2(initial_forward[1], initial_forward[0])
                    aligned_yaw = pad_yaw + round((initial_yaw - pad_yaw) / (np.pi / 2.)) * (np.pi / 2.)
                    self._camera_yaw_offset = float(aligned_yaw - initial_yaw)
        self.update_chase_camera()
        print("=" * 60)
        print(f"[Ready] {scenario} 시뮬레이션 및 3인칭 줌 추적 카메라 준비 완료!")
        print("=" * 60)

    def _on_mouse_event(self, event, *args, **kwargs):
        """
        마우스 휠(Scroll) 입력을 감지하여 3인칭 카메라와 드론 사이의 거리를 실시간 조절합니다.
        - 휠 위로(Scroll Up, y > 0): zoom_scale 감소 -> 드론에 가까워짐 (Zoom In)
        - 휠 아래로(Scroll Down, y < 0): zoom_scale 증가 -> 드론에서 멀어짐 (Zoom Out)
        """
        viewport = get_active_viewport()
        if (event.type == carb.input.MouseEventType.SCROLL
                and viewport is not None
                and str(viewport.camera_path) == self._chase_camera_path):
            scroll_y = event.scroll.y
            if scroll_y > 0:
                self.zoom_scale = max(0.35, self.zoom_scale * 0.88)
            elif scroll_y < 0:
                self.zoom_scale = min(4.5, self.zoom_scale * 1.12)
        return True

    def update_chase_camera(self):
        """Update the dedicated tracker without writing the free Perspective view."""
        if os.environ.get('ARGO_SCENE_VIEW') == 'overview':
            # Set the initial overview once; subsequent mouse navigation is free.
            if self._camera_view_initialized:
                return
            if self._scenario_name == 'frontier':
                if not hasattr(self,'_measured_overview'):
                    import json
                    recipe=json.loads((_SIMULATOR_DIR/f'assets/super/{self._scenario_name}_recipe.json').read_text())
                    low,high=np.array(recipe['bounds_min']),np.array(recipe['bounds_max'])
                    center=(low+high)/2.;span=max(float(np.max(high-low)),20.)
                    self._measured_overview=(center+span*np.array([-1.3,-1.5,1.]),center)
                self.pg.set_viewport_camera(*self._measured_overview)
            elif self._scenario_name == 'dasan':
                self.pg.set_viewport_camera([-60.,-95.,80.], [30.,-10.,6.])
            else:
                self.pg.set_viewport_camera([65.,-80.,65.], [0.,0.,1.5])
            self._camera_view_initialized = True
            return
        pos = self.drone.state.position
        quat = self.drone.state.attitude

        if pos is None or quat is None or np.linalg.norm(quat) < 1e-6:
            return

        # 기체 정면 벡터 [1, 0, 0]을 월드 좌표계로 변환 후 수평면(XY) 투영
        forward_world = Rotation.from_quat(quat).apply([1.0, 0.0, 0.0])
        horiz_norm = np.hypot(forward_world[0], forward_world[1])

        if horiz_norm > 1e-6:
            u_horiz = np.array([forward_world[0] / horiz_norm, forward_world[1] / horiz_norm, 0.0])
        else:
            u_horiz = np.array([0.0, 1.0, 0.0])

        u_horiz = Rotation.from_euler("z", self._camera_yaw_offset).apply(u_horiz)

        viewport = get_active_viewport()
        if viewport is None:
            return
        if (self._camera_view_initialized
                and str(viewport.camera_path) != self._chase_camera_path):
            return
        width, height = viewport.get_texture_resolution()
        aspect = max(width, 1) / max(height, 1)
        center = np.asarray(pos) + Rotation.from_quat(quat).apply(self._camera_body_offset)
        cam_eye, cam_target, focal = self._chase_rig.update(
            center, u_horiz, self.zoom_scale, aspect, self.world.current_time)
        self._chase_camera.GetFocalLengthAttr().Set(float(focal))
        self._chase_camera.CreateVerticalApertureAttr(36. / aspect)
        set_camera_view(eye=cam_eye, target=cam_target,
                        camera_prim_path=self._chase_camera_path,
                        viewport_api=viewport)
        if not self._camera_view_initialized:
            # Start in chase view, leaving Perspective at a useful initial pose.
            self.pg.set_viewport_camera(cam_eye.tolist(), cam_target.tolist())
            viewport.camera_path = self._chase_camera_path
            self._camera_view_initialized = True

    def publish_ground_truth(self):
        state = self.drone.state
        rotation = Rotation.from_quat(state.attitude)
        position = np.asarray(state.position) + rotation.apply(self._imu_offset)
        attitude = (rotation * self._imu_rotation).as_quat()
        msg = self._truth_type()
        stamp_ns = round(float(self.world.current_time) * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = "isaac_world"
        msg.child_frame_id = "mid360_imu"
        for axis, value in zip(("x", "y", "z"), position):
            setattr(msg.pose.pose.position, axis, float(value))
        for axis, value in zip(("x", "y", "z", "w"), attitude):
            setattr(msg.pose.pose.orientation, axis, float(value))
        # Twist is expressed in the mounted IMU frame (Odometry convention).
        velocity = np.asarray(state.linear_velocity) + rotation.apply(
            np.cross(state.angular_velocity, self._imu_offset))
        imu_world = rotation * self._imu_rotation
        for axis, value in zip(("x", "y", "z"), imu_world.inv().apply(velocity)):
            setattr(msg.twist.twist.linear, axis, float(value))
        for axis, value in zip(("x", "y", "z"), self._imu_rotation.inv().apply(state.angular_velocity)):
            setattr(msg.twist.twist.angular, axis, float(value))
        self._truth_pub.publish(msg)
        self._scenario_pub.publish(self._scenario_type(data=self._scenario_name))

    def run(self):
        """시뮬레이션 메인 루프 (물리 스텝 및 렌더링 갱신)"""
        self.timeline.play()
        try:
            while simulation_app.is_running():
                self.world.step(render=True)
                self.publish_ground_truth()
                self.update_chase_camera()
                if self._contact_diagnostics is not None:
                    self._contact_diagnostics.flush()
        finally:
            if self._contact_diagnostics is not None:
                self._contact_diagnostics.close()
            self.timeline.stop()
            simulation_app.close()

if __name__ == '__main__':
    app = ARGOPlaneApp()
    app.run()
