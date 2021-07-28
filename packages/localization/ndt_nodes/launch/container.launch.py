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
    
    map_publisher = Node(
        package='vt_ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[("/opt/data/maps/"+environ["map_name"]+"/map_publisher.param.yaml")]
    )

    return LaunchDescription([
        map_publisher,
    ])