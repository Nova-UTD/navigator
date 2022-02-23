from os import name, path, environ

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

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"param")
    interface = "can1"

    # steering_pid
    can = Node(
        package='voltron_can',
        executable='interface',
        arguments=[interface]
    )

    linear = Node(
        package='linear_actuator',
        executable='controller',
        name='linear_actuator_controller',
        parameters=[(path.join(param_dir,"interface","throttle_controller.param.yaml"))],
        remappings=[
            ("outgoing_can_frames", "outgoing_can_frames_" + interface),
        ]
    )
    return LaunchDescription([
        can,
        linear,
    ])
