"""Hardware Mid360s + existing FAST-LIVO config, with opt-in FC odometry output."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fast=Path(get_package_share_directory('fast_livo'))
    livox=Path(get_package_share_directory('livox_ros_driver2'))
    image=ParameterValue(PythonExpression(['1 if "',LaunchConfiguration('camera'),
                                          '".lower() in ("true","1") else 0']),value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('start_driver',default_value='true'),
        DeclareLaunchArgument('camera',default_value='false',description='Enable camera driver and visual fusion; requires verified calibration'),
        DeclareLaunchArgument('start_camera',default_value='true',description='Set false when the camera driver is already running in the same ROS domain'),
        DeclareLaunchArgument('livo_config',default_value=str(fast/'config/zenith.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(livox/'launch_ROS2/msg_MID360s_launch.py')),
                                 condition=IfCondition(LaunchConfiguration('start_driver'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('mvs_ros2_driver'))/'launch/zenith_camera.launch.py')),
            condition=IfCondition(PythonExpression([
                '"',LaunchConfiguration('camera'),'".lower() in ("true","1") and "',
                LaunchConfiguration('start_camera'),'".lower() in ("true","1")']))),
        Node(package='fast_livo',executable='fastlivo_mapping',name='laserMapping',output='screen',
             parameters=[LaunchConfiguration('livo_config'),str(fast/'config/zenith_camera_pinhole.yaml'),
                         {'uav.imu_rate_odom':True,'uav.px4_odom_en':True,'uav.gravity_align_en':True,
                          'common.img_en':image,'common.img_reliable':True,
                          'pcd_save.pcd_save_en':False,'evo.pose_output_en':False}])])
