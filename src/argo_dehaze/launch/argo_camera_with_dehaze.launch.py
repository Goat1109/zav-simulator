import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dehaze_pkg = get_package_share_directory('argo_dehaze')
    mvs_pkg = get_package_share_directory('mvs_ros2_driver')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mvs_pkg, 'launch', 'zenith_camera.launch.py')
        )
    )

    dehaze_node = Node(
        package='argo_dehaze',
        executable='argo_dehaze_node',
        name='argo_dehaze_node',
        parameters=[os.path.join(dehaze_pkg, 'config', 'dehaze_params.yaml')],
        output='screen'
    )

    return LaunchDescription([
        camera_launch,
        dehaze_node
    ])
