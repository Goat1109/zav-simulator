"""Super-LIO with optional EV adapter and independent camera driver."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own = Path(get_package_share_directory('zav_real_control'))
    livox = Path(get_package_share_directory('livox_ros_driver2'))
    super_lio = Path(get_package_share_directory('super_lio'))
    topics = ['odom', 'imu/odom', 'robo/odom', 'path', 'cloud_world',
              'robo/cloud_world', 'body/cloud_pose', 'world/cloud_pose',
              'processing_time', 'global_map']
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Enable camera driver for recording; Super-LIO does not fuse images'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Set false when the camera driver is already running in the same ROS domain'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=str(super_lio / 'rviz/lio.rviz')),
        DeclareLaunchArgument('planner_cloud', default_value='false',description='Publish world cloud for SUPER without RViz'),
        DeclareLaunchArgument('adapt_odometry', default_value='true',
                              description='Publish existing bridge-compatible IMU odometry'),
        DeclareLaunchArgument('config', default_value=str(own / 'config/super_lio_mid360s.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(livox / 'launch_ROS2/msg_MID360s_launch.py')),
            condition=IfCondition(LaunchConfiguration('start_driver'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('mvs_ros2_driver')) / 'launch/zenith_camera.launch.py')),
            condition=IfCondition(PythonExpression([
                '"', LaunchConfiguration('camera'), '".lower() in ("true","1") and "',
                LaunchConfiguration('start_camera'), '".lower() in ("true","1")']))),
        Node(package='super_lio', executable='super_lio_node', name='super_lio_node',
             output='screen', parameters=[LaunchConfiguration('config'),
                         {'lio.output.map': ParameterValue(PythonExpression(['"',LaunchConfiguration('rviz'),'".lower() in ("true","1") or "',LaunchConfiguration('planner_cloud'),'".lower() in ("true","1")']), value_type=bool)}],
             remappings=[('/lio/' + t, '/super_lio/' + t) for t in topics]
                        + [('/tf', '/super_lio/tf'), ('/tf_static', '/super_lio/tf_static')]),
        Node(package='zav_real_control', executable='super_lio_adapter',
             output='screen', condition=IfCondition(LaunchConfiguration('adapt_odometry'))),
        Node(package='rviz2', executable='rviz2', name='super_lio_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             remappings=[('/lio/' + t, '/super_lio/' + t) for t in topics]
                        + [('/tf', '/super_lio/tf'), ('/tf_static', '/super_lio/tf_static')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
