from os import path, environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

def generate_launch_description():
    # PERCEPTION
    # nodes
    # lidar_front_filter = Node(
    #     package='point_cloud_filter_transform_nodes',
    #     namespace="lidar_front",
    #     executable='velodyne_cloud_node_exe',
    #     parameters=[("/opt/params/"+environ["param_name"])],
    #     remappings=[("topic", "points_raw")],
    #     arguments=["--model", "vlp16"])

    lidar_front_filter = Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        namespace = "lidar_front",
        parameters=[("/opt/params/"+environ["front_param_name"])],
        remappings = [("points_in", "/lidar_front/points_raw")]
    )
    lidar_rear_filter = Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        namespace = "lidar_rear",
        parameters=[("/opt/params/"+environ["rear_param_name"])],
        remappings = [("points_in", "/lidar_rear/points_raw")]
    )

    return LaunchDescription([
        # PERCEPTION
        lidar_front_filter,
        lidar_rear_filter
    ])