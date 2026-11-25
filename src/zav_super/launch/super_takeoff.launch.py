"""Explicit web-confirmed ground start. Never launched by the web server at boot."""
from pathlib import Path
import json
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    super_share=Path(get_package_share_directory('zav_super'))
    control=Path(get_package_share_directory('zav_real_control'))
    hover=json.loads((super_share/'config/automatic_takeoff.json').read_text())['hover_thrust']
    if type(hover) not in (int,float) or not math.isfinite(hover) or not .1<=hover<=.8:
        raise ValueError('Invalid aircraft automatic takeoff hover_thrust')
    height=LaunchConfiguration('height')
    return LaunchDescription([
        DeclareLaunchArgument('height',default_value='0.4'),
        # Explicit aircraft setting. The bridge still validates the FC read-back.
        DeclareLaunchArgument('hover_thrust',default_value=str(hover)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(super_share/'launch/super_navigation.launch.py')),
            launch_arguments={'execute':'true','auto_takeoff':'true','takeoff_height':height}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(control/'launch/rc_flight.launch.py')),
            launch_arguments={'publish':'true','offboard':'true','offboard_source':'super',
                              'automatic_takeoff':'true','takeoff_height':height,
                              'takeoff_hover_thrust':LaunchConfiguration('hover_thrust')}.items())])
