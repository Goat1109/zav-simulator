"""Hardware SUPER + OMMPC. Optional EPIC guides; no simulator, UART, automatic arm or takeoff."""
from pathlib import Path
import hashlib
import json
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share=Path(get_package_share_directory('zav_super'));control=Path(get_package_share_directory('zav_real_control'))
    arg=lambda n:LaunchConfiguration(n).perform(context)
    estimator=arg('estimator');execute=arg('execute').lower() in ('true','1')
    source=Path(arg('common_config')).expanduser().resolve()
    values=yaml.safe_load(source.read_text())
    # Keep common-file tuning and map its I/O to the hardware adapter.
    values['fsm']['click_goal_topic']='/zav/super/goal'
    values['rog_map']['ros_callback'].update(cloud_topic='/argo/super/cloud_world',odom_topic='/argo/super/odom')
    values['rog_map']['visualization']['frame_id']='world'
    run=Path(tempfile.mkdtemp(prefix='zav-super-hardware-'))
    effective=run/'common.yaml';effective.write_text(yaml.safe_dump(values,sort_keys=False))
    (run/'provenance.json').write_text(json.dumps({'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'vehicle':'argo_v1','estimator':estimator,'execute':execute},indent=2))
    actions=[LogInfo(msg=f'SUPER common config: {effective}; estimator={estimator}; execute={execute}')]
    if estimator=='super_lio':
        sensor=yaml.safe_load((control/'config/super_lio_mid360s.yaml').read_text())
        sensor['/**']['ros__parameters'].update(values.get('super_lio',{}).get('ros__parameters',{}))
        sensor['/**']['ros__parameters'].update({'lio.output.map':True,'lio.output.dense':True})
        sensorpath=run/'super_lio.yaml';sensorpath.write_text(yaml.safe_dump(sensor))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(control/'launch/sensors_super_lio.launch.py')),
            launch_arguments={'config':str(sensorpath),'planner_cloud':'true','rviz':'false','start_driver':arg('start_driver')}.items()))
    elif estimator in ('lio','livo'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(control/'launch/sensors_lio.launch.py')),
            launch_arguments={'camera':'true' if estimator=='livo' else 'false','start_driver':arg('start_driver')}.items()))
    cloud=arg('cloud_topic') or ('/super_lio/cloud_world' if estimator=='super_lio' else '/cloud_registered')
    frame=arg('cloud_frame') or ('world' if estimator=='super_lio' else 'camera_init')
    parameters=values['argo_ommpc_px4']['ros__parameters']
    # Legacy simulator-only switches intentionally not passed to the hardware wrapper.
    parameters={k:v for k,v in parameters.items() if k not in ('use_px4_landing','allow_terminal_hold')}
    # Planner and mission supervisor must judge the same accepted goal/limits.
    parameters.update({key:float(values['fsm'].get(key,default)) for key,default in (
        ('goal_position_tolerance',.1),('goal_speed_tolerance',.1),('goal_snap_max_distance',.3))})
    epic_enabled=arg('epic_enabled').lower()=='true'
    epic_parameters={'epic_enabled':epic_enabled}
    if epic_enabled:
        epic_share=Path(get_package_share_directory('epic_ros2'))
        profiles={'gym':'gym-epic-full-local.yaml','frontier_whole':'frontier_bounds.yaml',
                  'frontier_single_floor':'frontier-first-floor-sim.yaml'}
        profile=arg('epic_profile')
        if profile not in profiles:raise ValueError('Unknown EPIC profile')
        epic_config=epic_share/'config'/profiles[profile]
        config=yaml.safe_load(epic_config.read_text())['/**']['ros__parameters']
        floor=float(values['rog_map']['virtual_ground_height'])+.2
        ceiling=float(values['rog_map']['virtual_ceil_height'])-.5
        flight_min=max(floor,float(config.get('flight_height.min_z',.3)))
        flight_max=min(ceiling,float(config.get('flight_height.max_z',ceiling)))
        if flight_min>=flight_max:raise ValueError('EPIC flight band conflicts with SUPER height limits')
        epic_parameters.update(epic_frame_required=True,epic_profile=profile,epic_floor=floor,epic_ceiling=ceiling,
            epic_flight_min=flight_min,epic_flight_max=flight_max,
            epic_bounds_lower=[float(v) for i in range(config['box_num']) for v in config[f'box_{i}.down']],
            epic_bounds_upper=[float(v) for i in range(config['box_num']) for v in config[f'box_{i}.up']])
        reference=[-3.26924,-.05305,.18920] if profile=='frontier_single_floor' else [0.,0.,0.]
        reference_yaw=-172.3865 if profile=='frontier_single_floor' else 0.
        provenance=json.loads((run/'provenance.json').read_text())
        provenance['epic']={'profile':profile,'source':str(epic_config),'sha256':hashlib.sha256(epic_config.read_bytes()).hexdigest(),
            'reference_position':reference,'reference_yaw_deg':reference_yaw,'flight_min':flight_min,'flight_max':flight_max,
            'relative_to_ground_start':True,'max_vel':1.}
        (run/'provenance.json').write_text(json.dumps(provenance,indent=2))
        actions.append(Node(package='zav_real_control',executable='epic_frame_adapter',output='screen',parameters=[{
            'use_sim_time':False,'reference_position':reference,'reference_yaw_deg':reference_yaw}]))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(epic_share/'launch/hardware.launch.py')),
            launch_arguments={'config':str(epic_config),'flight_min':str(flight_min),'flight_max':str(flight_max)}.items()))
        actions.append(LogInfo(msg=f'EPIC profile={profile}; file={epic_config}; world flight Z={flight_min}..{flight_max}m'))
    actions.extend([
        Node(package='zav_real_control',executable='super_adapter',output='screen',parameters=[{
            'use_sim_time':False,'config_file':arg('hardware_config'),'odom_topic':arg('odom_topic'),'cloud_topic':cloud,'cloud_frame':frame}]),
        Node(package='super_planner',executable='fsm_node',output='screen',parameters=[{
            'use_sim_time':False,'argo_mission_gate':False,'config_name':str(effective)}]),
        Node(package='zav_super',executable='super_ommpc',output='screen',parameters=[parameters,epic_parameters,{
            'use_sim_time':False,'execute':execute,'distance_m':float(arg('distance')),
            'auto_takeoff':arg('auto_takeoff').lower()=='true','takeoff_height':float(arg('takeoff_height'))}])])
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('zav_super'));control=Path(get_package_share_directory('zav_real_control'))
    return LaunchDescription([
        DeclareLaunchArgument('estimator',default_value='super_lio',choices=['super_lio','lio','livo','external']),
        DeclareLaunchArgument('epic_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('epic_profile',default_value='gym',choices=['gym','frontier_whole','frontier_single_floor']),
        DeclareLaunchArgument('execute',default_value='false'),
        DeclareLaunchArgument('auto_takeoff',default_value='false'),
        DeclareLaunchArgument('takeoff_height',default_value='0.4'),
        DeclareLaunchArgument('distance',default_value='10.0'),
        DeclareLaunchArgument('common_config',default_value=str(share/'config/common.yaml')),
        DeclareLaunchArgument('hardware_config',default_value=str(control/'config/hardware.yaml')),
        DeclareLaunchArgument('start_driver',default_value='true'),
        DeclareLaunchArgument('odom_topic',default_value='/zav/lio/odometry'),
        DeclareLaunchArgument('cloud_topic',default_value=''),
        DeclareLaunchArgument('cloud_frame',default_value=''),
        OpaqueFunction(function=setup)])
