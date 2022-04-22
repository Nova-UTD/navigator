import os
from os import path

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir, "param")
    parameter_file = parameters = [
        (path.join(param_dir, "mapping", "lio_sam.param.yaml"))]
    launch_path = os.path.realpath(__file__)
    launch_dir = os.path.dirname(launch_path)

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))

    state_pub = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[os.path.join(launch_dir, "data", "hail_bopp.urdf")]
    )

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud', '/lidar_front/velodyne_points')],
        output='screen'
    )

    odom_bl = launch_ros.actions.Node(
        package='tf2_ros',  # Fuse the map and odom frames
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 1.0 odom base_link'.split(' '),
        parameters=[parameter_file],
        output='screen'
    )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'velodyne']
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        mapping,
        tf,
        graphbasedslam,
        odom_bl,
        state_pub
    ])
