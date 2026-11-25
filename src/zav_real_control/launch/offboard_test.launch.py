"""Preview is the default. This launch never starts a bridge or opens an FC link."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test',default_value='go_front',
            choices=['go_front','go_right','go_back','go_left','waypoint_square','waypoint_circle']),
        DeclareLaunchArgument('execute',default_value='false',description='Explicitly publish live test requests'),
        DeclareLaunchArgument('config',default_value=str(Path(get_package_share_directory('zav_real_control'))/'config/offboard_tests.yaml')),
        Node(package='zav_real_control',executable=LaunchConfiguration('test'),output='screen',
             parameters=[LaunchConfiguration('config'),{'execute':ParameterValue(LaunchConfiguration('execute'),value_type=bool)}]),
    ])
