from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ndt_localizer = get_package_share_directory('ndt_localizer')

    return LaunchDescription([
        DeclareLaunchArgument('input_sensor_points_topic', default_value='/filtered_points', description='Sensor points topic'),
        DeclareLaunchArgument('input_odom_topic', default_value='/odom', description='filtered odometry topic'),
        DeclareLaunchArgument('input_initial_pose_topic', default_value='/initialpose', description='Initial position topic to align'),
        DeclareLaunchArgument('input_map_points_topic', default_value='/points_map', description='Map points topic'),
        DeclareLaunchArgument('output_pose_topic', default_value='ndt_pose', description='Estimated self position'),
        DeclareLaunchArgument('output_pose_with_covariance_topic', default_value='ndt_pose_with_covariance', description='Estimated self position with covariance'),
        DeclareLaunchArgument('output_diagnostics_topic', default_value='diagnostics', description='Diagnostic topic'),
        DeclareLaunchArgument('path_file', default_value=os.path.join(pkg_ndt_localizer, ''), description='Mapping trajectory as height map'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Vehicle reference frame'),
        DeclareLaunchArgument('trans_epsilon', default_value='0.05', description='The maximum difference between two consecutive transformations in order to consider convergence'),
        DeclareLaunchArgument('step_size', default_value='0.1', description='The newton line search maximum step length'),
        DeclareLaunchArgument('resolution', default_value='3.0', description='The ND voxel grid resolution'),
        DeclareLaunchArgument('max_iterations', default_value='50', description='The number of iterations required to calculate alignment'),
        DeclareLaunchArgument('converged_param_transform_probability', default_value='3.0', description=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Includes (converted to Python launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ndt_localizer, 'launch/jaguar.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ndt_localizer, 'launch/static_tf.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ndt_localizer, 'launch/map_loader.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ndt_localizer, 'launch/points_downsample.launch.py')),
        ),

        Node(
            package='ndt_localizer',
            executable='ndt_localizer_node',
            name='ndt_localizer_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'initialpose': LaunchConfiguration('input_initial_pose_topic')},
                {'base_frame': LaunchConfiguration('base_frame')},
                {'path_file': LaunchConfiguration('path_file')},
                {'trans_epsilon': LaunchConfiguration('trans_epsilon')},
                {'step_size': LaunchConfiguration('step_size')},
                {'resolution': LaunchConfiguration('resolution')},
                {'max_iterations': LaunchConfiguration('max_iterations')},
                {'converged_param_transform_probability': LaunchConfiguration('converged_param_transform_probability')},
            ],
            remappings=[
                ('points_raw', LaunchConfiguration('input_sensor_points_topic')),
                ('odom', LaunchConfiguration('input_odom_topic')),
                ('pointcloud_map', LaunchConfiguration('input_map_points_topic')),
                ('ndt_pose', LaunchConfiguration('output_pose_topic')),
                ('ndt_pose_with_covariance', LaunchConfiguration('output_pose_with_covariance_topic')),
                ('diagnostics', LaunchConfiguration('output_diagnostics_topic')),
            ],
        ),
    ])
