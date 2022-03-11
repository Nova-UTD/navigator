
from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pure_pursuit_controller = Node(
        package='nova_motion_control',
        executable='MotionControl',
        name='controls'
        ]
    )

    return LaunchDescription([
        pure_pursuit_controller
    ])