from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ndt_localizer = FindPackageShare('ndt_localizer')
    model_path = PathJoinSubstitution([pkg_ndt_localizer, 'urdf', 'lexus.urdf'])

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='/base_link'),
        DeclareLaunchArgument('topic_name', default_value='vehicle_model'),
        DeclareLaunchArgument('offset_x', default_value='1.2'),
        DeclareLaunchArgument('offset_y', default_value='0.0'),
        DeclareLaunchArgument('offset_z', default_value='0.0'),
        DeclareLaunchArgument('offset_roll', default_value='0.0'),
        DeclareLaunchArgument('offset_pitch', default_value='0.0'),
        DeclareLaunchArgument('offset_yaw', default_value='0.0'),
        DeclareLaunchArgument('model_path', default_value=model_path),
        DeclareLaunchArgument('gui', default_value='False'),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[
                {'robot_description': PathJoinSubstitution([LaunchConfiguration('model_path')])},
                {'use_gui': LaunchConfiguration('gui')},
            ],
        ),
    ])
