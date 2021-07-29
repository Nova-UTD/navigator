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
    
    route_planner = Node(
        package='vt_route_planner',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('ndt_pose', '/localization/ndt_pose')]
    )

    return LaunchDescription([
        route_planner
    ])