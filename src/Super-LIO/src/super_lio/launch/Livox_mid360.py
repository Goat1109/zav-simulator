import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    # 출력 토픽 이름을 밖에서 바꿀 수 있게 한다(기본값은 원래 이름이라 기존 사용에 영향 없음).
    # 예) ros2 launch super_lio Livox_mid360.py \
    #       pose_topic:=/handheld/pose cloud_topic:=/handheld/point_cloud
    declare_pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='/lio/robo/odom',
        description='remap target for /lio/robo/odom')
    declare_cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/lio/cloud_body',
        description='remap target for /lio/cloud_body (deskewed cloud in body frame)')

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[config_yaml],
        remappings=[
            ('/lio/robo/odom',  LaunchConfiguration('pose_topic')),
            ('/lio/cloud_body', LaunchConfiguration('cloud_topic')),
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_pose_topic_arg)
    ld.add_action(declare_cloud_topic_arg)
    ld.add_action(super_lio_node)
    ld.add_action(rviz2_node)

    return ld