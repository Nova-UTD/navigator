import os
from os import path

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud','/lidar_fused')],
        output='screen'
        )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','velodyne']
        )


    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
        )

    urdf_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=["/home/wheitman/navigator/data/hail_bopp.urdf"]
    )

    lidar_fusion = launch_ros.actions.Node(
        package='lidar_fusion',
        name='lidar_fusion_node',
        executable='lidar_fusion_node',
        remappings=[
            ('/lidar_front', '/lidar_front/velodyne_points'),
            ('/lidar_rear', '/lidar_rear/velodyne_points'),
            ('/lidar_fused', '/lidar_fused')
        ]
    )

    gnss_parser = launch_ros.actions.Node(
    package = 'gnss_parser',
    executable = 'gnss_parser',
    remappings = [
        ("/sensors/gnss/odom", "gnss_odom"),
        ("/serial/gnss", "serial_incoming_lines")])


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        mapping,
        lidar_fusion,
        tf,
        urdf_publisher,
        gnss_parser,
        graphbasedslam,
            ])