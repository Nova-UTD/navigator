# * Package:   bridge_manager
# * Filename:  container.launch.py
# * Author:    Raghav Pillai
# * Email:     raghavpillai101@gmail.com
# * Copyright: 2021, Nova UTD
# * License:   MIT License
#

from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    manager = Node(
        package='bridge_manager',
        executable='bridge_manager',
        remappings=[
            #("outgoing_bridge_messaging", environ["bridge_outgoing_topic_name"]),
            ("incoming_bridge_messages", environ["bridge_incoming_topic_name"])
        ]
    )

    return LaunchDescription([
        manager
    ])
