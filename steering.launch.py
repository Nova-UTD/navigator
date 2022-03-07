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
    interface = "can0"

    epas_reporter = Node(
        package='epas_translator',
        executable='reporter',
        parameters=[(path.join(param_dir,"interface","epas_reporter.param.yaml"))],
        remappings=[
            ("incoming_can_frames", "incoming_can_frames_"+interface),
            ("real_steering_angle", "real_steering_angle")
        ]
    )

    epas_controller = Node(
        package='epas_translator',
        executable='controller',
        parameters=[(path.join(param_dir,"interface","epas_controller.param.yaml"))],
        remappings=[
            ("steering_power", "steering_power"),
            ("outgoing_can_frames", "outgoing_can_frames_"+interface)
        ]
    )

    can = Node(
        package='can_interface',
        executable='interface',
        parameters=[(path.join(param_dir,"interface","can_interface.param.yaml"))],
        arguments=[interface]
    )


    pid = Node(
        package='pid_controller',
        executable='pid_controller',
        parameters=[(path.join(param_dir,"interface","steering_pid_controller.param.yaml"))],
        remappings=[
            ("output", "steering_power"),
            ("target", "target_steering_angle"),
            ("measurement", "real_steering_angle"),
        ]
    )

    return LaunchDescription([
        can,
        epas_controller,
        epas_reporter,
        pid,
    ])
