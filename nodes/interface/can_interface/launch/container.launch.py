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
    can_interface = Node(
        package='can_interface',
        executable='interface',
        parameters=[("/opt/param/"+environ["param_name"])],
        remappings=[
            ("incoming_can_frames", environ["incoming_topic_name"]),
            ("outgoing_can_frames", environ["outgoing_topic_name"])
        ]
    )

    return LaunchDescription([
        can_interface
    ])
