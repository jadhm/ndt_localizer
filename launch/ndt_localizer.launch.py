from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ndt_localizer',
            executable='ndt',
            name='ndt_localizer',
            output='screen',
            parameters=[{
                # Add parameters here as needed, e.g. 'param_name': value
            }]
        ),
        Node(
            package='ndt_localizer',
            executable='mapLoader',
            name='map_loader',
            output='screen',
            parameters=[{
                # Add parameters here as needed
            }]
        ),
        Node(
            package='ndt_localizer',
            executable='voxel_grid_filter',
            name='voxel_grid_filter',
            output='screen',
            parameters=[{
                # Add parameters here as needed
            }]
        ),
    ])
