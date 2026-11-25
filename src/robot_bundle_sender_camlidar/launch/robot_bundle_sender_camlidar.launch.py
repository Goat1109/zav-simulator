from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_params = PathJoinSubstitution([
        get_package_share_directory('robot_bundle_sender_camlidar'), 'config',
        ['drone_', LaunchConfiguration('mode'), '.yaml'],
    ])
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fast_livo',
                              choices=['fast_livo', 'super_lio'],
                              description='Select the drone parameter preset'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(package='robot_bundle_sender_camlidar', executable='bundle_sender_node', name='robot_bundle_sender_camlidar', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
