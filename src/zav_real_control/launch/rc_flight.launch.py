"""One UART for EV/control. Automatic takeoff is disabled unless explicitly enabled."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default=str(Path(get_package_share_directory('zav_real_control'))/'config/hardware.yaml')
    parameters=[{'config_file':LaunchConfiguration('config'),
                 'offboard_source':LaunchConfiguration('offboard_source'),
                 'offboard_enabled':ParameterValue(LaunchConfiguration('offboard'),value_type=bool),
                 'automatic_takeoff':ParameterValue(LaunchConfiguration('automatic_takeoff'),value_type=bool),
                 'takeoff_hover_thrust':ParameterValue(LaunchConfiguration('takeoff_hover_thrust'),value_type=float),
                 'takeoff_height':ParameterValue(LaunchConfiguration('takeoff_height'),value_type=float),
                 'publish_enabled':ParameterValue(LaunchConfiguration('publish'),value_type=bool)}]
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=default),
        DeclareLaunchArgument('offboard_source',default_value='legacy',choices=['legacy','super']),
        DeclareLaunchArgument('offboard',default_value='false',description='Enable RC8 Offboard hover stream and test requests'),
        DeclareLaunchArgument('automatic_takeoff',default_value='false'),
        DeclareLaunchArgument('takeoff_hover_thrust',default_value='0.0'),
        DeclareLaunchArgument('takeoff_height',default_value='0.4'),
        DeclareLaunchArgument('publish',default_value='false',description='Enable EV and failsafe writes after bench verification'),
        Node(package='zav_real_control',executable='mavlink_bridge',parameters=parameters,output='screen')])
