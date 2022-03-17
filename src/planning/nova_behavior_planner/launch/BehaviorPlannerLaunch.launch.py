from os import environ

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    behavior_planner = Node(
        package='nova_behavior_planner',
        executable='BehaviorPlannerLaunch',
        name='BehaviorPlanner'
        ]
    )

    return LaunchDescription([
        behavior_planner
    ])