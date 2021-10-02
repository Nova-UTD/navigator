from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pid_controller = Node(
        package='pid_controller',
        executable='pid_controller',
        parameters=[("/opt/param/"+environ["param_name"])],
        remappings=[
            ("measurement", environ["measurement_topic_name"]),
            ("target", environ["target_topic_name"]),
            ("output", environ["output_topic_name"])
        ]
    )

    return LaunchDescription([
        pid_controller
    ])
