"""EV bridge + conditional baro/LAND. Pilot flies RC Position mode; no offboard node."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default=str(Path(get_package_share_directory('zav_real_control'))/'config/dds_legacy.yaml')
    parameters=[{'config_file':LaunchConfiguration('config'),
                 'publish_enabled':ParameterValue(LaunchConfiguration('publish'),value_type=bool)}]
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=default),
        DeclareLaunchArgument('publish',default_value='false',description='Enable EV and failsafe writes after bench verification'),
        Node(package='zav_real_control',executable='lio_bridge',parameters=parameters,output='screen'),
        Node(package='zav_real_control',executable='baro_link',parameters=parameters,output='screen')])
