from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_ndt_localizer = get_package_share_directory('ndt_localizer')
    return LaunchDescription([
        DeclareLaunchArgument('input_initial_pose_topic', default_value='/initialpose', description='Initial position topic to align'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered/global', description='robot odom topic'),
        DeclareLaunchArgument('submap_size_xy', default_value='50.0'),
        DeclareLaunchArgument('submap_size_z', default_value='20.0'),
        DeclareLaunchArgument('map_switch_thres', default_value='25.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('pcd_path', default_value=os.path.join(pkg_ndt_localizer, 'map/map.pcd')),
        DeclareLaunchArgument('map_topic', default_value='/points_map'),
        # RViz node (optional, comment out if not needed)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', os.path.join(pkg_ndt_localizer, 'cfgs/jaguar.rviz')],
        #     output='screen',
        # ),
        Node(
            package='ndt_localizer',
            executable='map_loader',
            name='map_loader',
            output='screen',
            parameters=[
                {'pcd_path': LaunchConfiguration('pcd_path')},
                {'map_topic': LaunchConfiguration('map_topic')},
                {'robot_pose_topic': LaunchConfiguration('odom_topic')},
                {'init_pose_topic': LaunchConfiguration('input_initial_pose_topic')},
                {'submap_size_xy': LaunchConfiguration('submap_size_xy')},
                {'submap_size_z': LaunchConfiguration('submap_size_z')},
                {'map_switch_thres': LaunchConfiguration('map_switch_thres')},
                {'roll': LaunchConfiguration('roll')},
                {'pitch': LaunchConfiguration('pitch')},
                {'yaw': LaunchConfiguration('yaw')},
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'z': LaunchConfiguration('z')},
            ],
        ),
    ])
