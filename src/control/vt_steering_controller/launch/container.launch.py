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


    # CONTROL
    # param
    vt_steering_controller = Node(
        package='vt_steering_controller',
        executable='controller_exe',
        name='controller_exe',
        namespace='control',
    )

    return LaunchDescription([
        # CONTROL
        vt_steering_controller
    ])