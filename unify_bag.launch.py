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

bag_path = "/home/main/voltron/assets/bags/april16/rosbag2_2022_04_16-22_02_11"
svo_path = "/home/main/voltron/assets/bags/april16/HD720_SN34750148_17-02-06_trimmed.svo"


def generate_launch_description():

    # run a bag in an infinite loop
    bag_process = subprocess.Popen(f"ros2 bag play -l {bag_path}".split())

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir, "param")
    interface = "vcan0"

    zed_wrapper_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        output='screen',
        parameters=[
            # YAML files
            path.join(param_dir, "perception", "zed.param.yaml"),
            {

                'general.camera_name': 'zed2',
                'general.camera_model': 'zed2',
                'general.svo_file': svo_path,
                #  'general.svo_loop': True, # Loop infinitely
                'pos_tracking.base_frame': 'base_link',
                'od_enabled': True
            }
        ]
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(launch_dir, "data", "hail_bopp.urdf")]
    )

    # CONTROL
    gnss_pub = Node(
        package="bag_tools",
        executable="gnss_log_publisher"
    )
    # LIO-SAM only needs three inputs: IMU, Lidar, and GPS
    return LaunchDescription([
        gnss_pub,
        urdf_publisher,
        zed_wrapper_node
    ])
