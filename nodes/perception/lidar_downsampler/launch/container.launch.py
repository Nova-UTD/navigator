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

    downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        parameters=[("/opt/param/"+environ["param_name"])],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ],
    )

    return LaunchDescription([
        downsampler
    ])