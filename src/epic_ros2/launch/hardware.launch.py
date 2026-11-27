"""Exploration only: no simulator clock, trajectory server, FC transport or takeoff."""
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arg=lambda key:LaunchConfiguration(key).perform(context)
    return [Node(package='epic_ros2',executable='exploration_node',name='epic_exploration',
        prefix='nice -n 10',additional_env={'OMP_WAIT_POLICY':'PASSIVE','OMP_NUM_THREADS':'2'},
        parameters=[{'global_planning.right_hand_priority':True,
                     'global_planning.connected_wall_traversal':False,
                     'global_planning.right_hand_corridor_box':-1},
            arg('config'),{'use_sim_time':False,'fsm.external_local_planner':True,
            'odometry_topic':'/epic/profile_odom','cloud_topic':'/epic/profile_cloud',
            'exploration.state_reference_imu_topic':'/livox/imu','exploration.planning_workers':2,
            'lidar_perception.odom_twist_in_world':True,
            'exploration.tsp_dir':tempfile.mkdtemp(prefix='zav-epic-tsp-'),
            'MaxVelMag':float(arg('max_vel')),'MaxAccMag':float(arg('max_acc')),'MaxJerkMag':float(arg('max_jerk')),'flight_height.min_z':float(arg('flight_min')),
            'flight_height.max_z':float(arg('flight_max')),
            'fsm.hardware_height_guard':True,'fsm.world_floor':float(arg('world_floor')),
            'fsm.world_ceiling':float(arg('world_ceiling')),'fsm.height_margin':float(arg('height_margin'))}],
        remappings=[('/mission/exploration_path','/epic/profile_path'),('/waypoint_generator/waypoints','/epic/trigger'),
            ('/planning/state','/epic/state'),('/planning/heartbeat','/epic/heartbeat'),
            ('/planning/land_requested','/epic/land_requested'),
            ('/px4ctrl/takeoff_land','/epic/unused_takeoff_land'),
            *[('/planning/'+key,'/epic/'+key) for key in
                ('trajectory','yaw_trajectory','static','replan')]],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config'),
        DeclareLaunchArgument('max_vel',default_value='1.0'),
        DeclareLaunchArgument('max_acc',default_value='1.5'),
        DeclareLaunchArgument('max_jerk',default_value='3.0'),
        DeclareLaunchArgument('flight_min',default_value='.3'),
        DeclareLaunchArgument('flight_max',default_value='9.5'),
        DeclareLaunchArgument('world_floor',default_value='-.4'),
        DeclareLaunchArgument('world_ceiling',default_value='9.5'),
        DeclareLaunchArgument('height_margin',default_value='.03'),OpaqueFunction(function=setup)])
