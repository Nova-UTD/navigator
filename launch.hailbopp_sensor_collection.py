from os import name, path, environ
import sys

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

sys.path.append(path.abspath('/navigator/'))
from launch_node_definitions import *

NAVIGATOR_DIR = "/navigator/"

def generate_launch_description():

    return LaunchDescription([
        clock,
        gnss,
        left_lidar_driver,
        left_lidar_pointcloud,
        right_lidar_driver,
        right_lidar_pointcloud,
        lidar_processor,
        radar_processor,
        camera,
        urdf_publisher,
        rviz,
    ])