from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

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
