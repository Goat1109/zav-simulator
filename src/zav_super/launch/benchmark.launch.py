"""SUPER + LIO adapter + ROS 2 OMMPC. Isaac runs in its own process."""
import os
from pathlib import Path
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, EmitEvent, LogInfo, ExecuteProcess
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share=Path(get_package_share_directory('zav_super'))
    workspace=Path(os.environ['ZAV_WS_DIR'])
    scenario=LaunchConfiguration('scenario').perform(context)
    backend=LaunchConfiguration('backend').perform(context)
    rviz=LaunchConfiguration('rviz').perform(context).lower() in ('true','1')
    execute=LaunchConfiguration('execute').perform(context).lower() in ('true','1')
    profile=LaunchConfiguration('profile').perform(context)
    common_config=LaunchConfiguration('common_config').perform(context)
    if scenario in ('mixed','dasan') and profile!='common':
        raise RuntimeError('mixed/dasan requires profile:=common')
    if common_config and profile!='common':
        raise RuntimeError('common_config requires profile:=common')
    if profile=='common' and backend!='super_lio':
        raise RuntimeError('common profile currently supports only super_lio')
    planner_config=controller_config=share/'config/common.yaml'
    if profile=='common':
        planner_config=controller_config=(Path(common_config).expanduser().resolve()
            if common_config else share/'config/common.yaml')
        if not planner_config.is_file():
            raise RuntimeError(f'Configuration file not found: {planner_config}')
    controller_parameters=(yaml.safe_load(controller_config.read_text())['zav_ommpc_px4']['ros__parameters']
                           if profile=='common' else str(controller_config))
    sys.path.insert(0,str(workspace/'simulator/pegasus/simulator/logic/sensors'))
    from fastlivo_calibration import load_calibration
    calibration=load_calibration(os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR',str(workspace/'src/FAST-LIVO2/config')),[.01215,0.,.11503],[0.,22.,0.])
    # Reuse exactly the existing mapper, external vision bridge and jump safety.
    lio=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(workspace/'simulator/tools/rviz/fastlivo_sim.launch.py')),
        launch_arguments={'backend':backend,
                          'navigation_profile':'true',
                          'navigation_config':str(planner_config) if profile=='common' else '',
                          'camera':'True' if backend=='fastlivo' else 'False','rviz':'False','safety':'True'}.items())
    mission_path=None
    auto_viewpoints=False
    adapter_parameters={'scenario':scenario,'imu_position':calibration['imu_position'],'imu_orientation':calibration['imu_orientation']}
    if scenario=='warehouse':
        adapter_parameters['spawn']=[0.,0.,.10]
    route_source=LaunchConfiguration('route_source').perform(context)
    use_epic=route_source=='epic'
    if use_epic:
        route_source='external'
    planner_values=yaml.safe_load(planner_config.read_text())
    mission_parameters={'use_sim_time':True,'execute':execute,'route_source':route_source,
        'path_chord_error_m':float(planner_values['rog_map']['resolution']),
        'path_max_acc':float(planner_values['traj_opt']['boundary']['max_acc']),
        'path_max_jerk':float(planner_values['traj_opt']['boundary']['max_jerk']),
        'path_lookahead_max_m':float(planner_values['super_planner']['planning_horizon'])}
    if route_source=='external' and not planner_values['super_planner']['goal_yaw_en']:
        raise RuntimeError('External exploration paths require super_planner.goal_yaw_en=true to preserve yaw goals')
    if scenario in ('dasan','frontier'):
        import json,hashlib
        path=Path(LaunchConfiguration('mission').perform(context) or workspace/f'simulator/assets/super/{scenario}_mission.json').expanduser().resolve()
        if not path.is_file():raise RuntimeError(f'Generate the clearance-checked {scenario} mission first')
        mission_path=path
        mission=json.loads(path.read_text())
        source=workspace/f'simulator/assets/super/{scenario}.pcd'
        if mission['map_sha256']!=hashlib.sha256(source.read_bytes()).hexdigest():
            raise RuntimeError(f'{scenario} mission and current physical map differ; regenerate the mission')
        adapter_parameters.update(spawn=mission['spawn'],spawn_yaw=mission['spawn_yaw'])
        controller_parameters=dict(controller_parameters,takeoff_height=float(mission['takeoff_height']))
        mission_parameters['mission_file']=str(path)
        auto_viewpoints=execute and route_source=='bag' and (
            bool(mission.get('viewpoint_pause_indices')) or mission.get('continuous_waypoints',False))
        if auto_viewpoints:mission_parameters['route_source']='external'
    def critical(): return [EmitEvent(event=Shutdown(reason='Critical SUPER navigation node exited'))]
    actions=[LogInfo(msg=f'SUPER planner config: {planner_config}'),lio,
        Node(package='zav_super',executable='lio_adapter.py',parameters=[adapter_parameters],output='screen',on_exit=critical()),
        Node(package='super_planner',executable='fsm_node',parameters=[{
            'use_sim_time':True,'zav_mission_gate':True,'config_name':str(planner_config)}],output='screen',on_exit=critical()),
        Node(package='zav_super',executable='ommpc_px4',parameters=[{'use_sim_time':True},controller_parameters],output='screen',on_exit=critical()),
        Node(package='zav_super',executable='benchmark_mission.py',parameters=[mission_parameters],output='screen')]
    if auto_viewpoints:
        actions.append(ExecuteProcess(cmd=[sys.executable,str(workspace/'simulator/tools/super/publish_bag_route.py'),
            '--scenario',scenario,'--mission',str(mission_path),'--land-at-end'],output='screen'))
    if use_epic:
        epic_share=Path(get_package_share_directory('epic_ros2'))
        epic_args={'config':str(epic_share/'config/isaac_building_explore.yaml'),
                   'external_local_planner':'true'}
        if scenario=='frontier':
            epic_args['scene_config']=str(epic_share/'config/frontier_bounds.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(epic_share/'launch/isaac_epic.launch.py')),
            launch_arguments=epic_args.items()))
        if execute:
            actions.append(ExecuteProcess(name='epic_start_after_takeoff',
                cmd=['bash',str(workspace/'simulator/tools/super/start_epic.sh')],
                output='screen'))
    rviz_config=share/'rviz'/('super_epic.rviz' if use_epic else 'super.rviz')
    if rviz: actions.append(Node(package='rviz2',executable='rviz2',prefix=['bash '+str(workspace/'simulator/tools/rviz/gui_exec.sh')],arguments=['-d',str(rviz_config)],parameters=[{'use_sim_time':True}],output='screen'))
    telemetry_dir=LaunchConfiguration('telemetry_dir').perform(context)
    if telemetry_dir:
        mission_args=['--mission',str(mission_path)] if mission_path else []
        actions.insert(0,ExecuteProcess(cmd=[sys.executable,str(workspace/'simulator/tools/super/flight_telemetry.py'),
            '--output',telemetry_dir,'--config',str(planner_config),'--scenario',scenario,'--backend',backend,*mission_args],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value='high_speed',choices=['warehouse','high_speed','dense','mixed','dasan','frontier']),
        DeclareLaunchArgument('backend',default_value='super_lio',choices=['super_lio']),
        DeclareLaunchArgument('profile',default_value='common',choices=['common']),
        DeclareLaunchArgument('common_config',default_value='',
                             description='Optional shared YAML path for isolated common-profile experiments'),
        DeclareLaunchArgument('mission',default_value='',description='Measured-scene clearance-checked bag-derived route JSON'),
        DeclareLaunchArgument('route_source',default_value='bag',choices=['bag','external','epic'],
                             description='Bag benchmark, external path publisher, or built-in EPIC exploration'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('execute',default_value='false'),
        DeclareLaunchArgument('telemetry_dir',default_value=''),
        OpaqueFunction(function=setup)])
