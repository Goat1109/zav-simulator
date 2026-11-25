import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('argo_dehaze')
    default_config = os.path.join(pkg_share, 'config', 'dehaze_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to dehaze configuration YAML file'
    )

    dehaze_node = Node(
        package='argo_dehaze',
        executable='argo_dehaze_node',
        name='argo_dehaze_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        dehaze_node
    ])
