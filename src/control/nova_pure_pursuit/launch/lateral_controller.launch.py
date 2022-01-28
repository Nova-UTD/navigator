
from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pure_pursuit_controller = Node(
        package='nova_pure_pursuit',
        executable='LateralController',
        name='controller'
        ]
    )

    return LaunchDescription([
        pure_pursuit_controller
    ])