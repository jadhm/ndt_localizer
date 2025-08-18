from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ndt_localizer = FindPackageShare('ndt_localizer')
    share = get_package_share_directory("ndt_localizer")

    urdf_path = PathJoinSubstitution([pkg_ndt_localizer, 'urdf', 'jaguar.urdf'])
    # local_ekf_yaml = PathJoinSubstitution([pkg_ndt_localizer, 'cfgs', 'local_ekf.yaml'])
    local_ekf_yaml = str(PathJoinSubstitution([share, "cfgs", "local_ekf.yaml"]).perform({}))
    # global_ekf_yaml = PathJoinSubstitution([pkg_ndt_localizer, 'cfgs', 'global_ekf.yaml'])
    global_ekf_yaml = str(PathJoinSubstitution([share, "cfgs", "global_ekf.yaml"]).perform({}))

    return LaunchDescription([
        # Robot State Publisher with xacro
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro ', urdf_path, ' --inorder'])
        #     }],
        # ),
        # Local EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[local_ekf_yaml],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/local'),
            ],
        ),
        # Global EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_global',
            output='screen',
            parameters=[global_ekf_yaml],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/global'),
            ],
        ),
        # NavSat Transform Node
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     parameters=[{
        #         'wait_for_datum': True,
        #         'datum': [50.7894803, -0.9132029, 0.0],
        #         'zero_altitude': True,
        #         'magnetic_declination_radians': 0.0,
        #         'yaw_offset': 3.05,
        #     }],
        #     remappings=[
        #         ('/imu/data', '/imu/data'),
        #         ('/gps/fix', '/gnss'),
        #         ('/odometry/filtered', '/odometry/filtered/global'),
        #     ],
        #     # respawn is not natively supported in ROS 2 launch, use lifecycle or external tools if needed
        # ),
    ])
