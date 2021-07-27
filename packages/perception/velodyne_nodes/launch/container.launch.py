from os import path

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
    lidar_front_node = Node(
        package='velodyne_nodes',
        namespace="lidar_front",
        executable='velodyne_cloud_node_exe',
        parameters=[("/opt/package/param/vlp16_voltron_front.param.yaml")],
        remappings=[("topic", "points_raw")],
        arguments=["--model", "vlp16"])

    return LaunchDescription([
        # PERCEPTION
        lidar_front_node,
    ])