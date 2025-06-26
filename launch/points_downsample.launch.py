from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sync', default_value='false'),
        DeclareLaunchArgument('node_name', default_value='voxel_grid_filter'),
        DeclareLaunchArgument('points_topic', default_value='/cloud'),
        DeclareLaunchArgument('output_log', default_value='false'),
        DeclareLaunchArgument('leaf_size', default_value='1.0'),

        Node(
            package='ndt_localizer',
            executable=LaunchConfiguration('node_name'),
            name=LaunchConfiguration('node_name'),
            parameters=[
                {'points_topic': LaunchConfiguration('points_topic')},
                {'output_log': LaunchConfiguration('output_log')},
                {'leaf_size': LaunchConfiguration('leaf_size')},
            ]
        ),
    ])
