from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    reporter = Node(
        package='epas_translator',
        executable='reporter',
        parameters=[("/opt/param/"+environ["reporter_param_name"])],
        remappings=[
            ("incoming_can_frames", environ["incoming_can_topic_name"]),
            ("real_steering_angle", environ["real_steering_angle_topic_name"])
        ]
    )

    controller = Node(
        package='epas_translator',
        executable='controller',
        parameters=[("/opt/param/"+environ["controller_param_name"])],
        remappings=[
            ("steering_power", environ["steering_effort_topic_name"]),
            ("outgoing_can_frames", environ["outgoing_can_topic_name"])
        ]
    )

    return LaunchDescription([
        reporter,
        controller
    ])
