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

import subprocess

def generate_launch_description():

    # run a bag
    bag_process = subprocess.Popen("ros2 bag play /mnt/sda1/bags/april16/rosbag2_2022_04_16-22_02_11".split())

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"param")
    interface = "vcan0"

    # CONTROL
    gnss_pub = Node(
        package="bag_tools",
        executable="gnss_log_publisher"
    )
    # LIO-SAM only needs three inputs: IMU, Lidar, and GPS
    return LaunchDescription([
        gnss_pub
    ])