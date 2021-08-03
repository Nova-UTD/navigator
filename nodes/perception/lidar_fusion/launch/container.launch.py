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
    
    lidar_fusion = Node(
        package='vt_point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        namespace="lidars",
        parameters=[("/opt/param/"+environ["param_name"])],
        remappings=[
            ("output_topic", "points_fused"),
            ("input_topic1", "/lidar_front/points_filtered"), 
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )

    return LaunchDescription([
        lidar_fusion
    ])