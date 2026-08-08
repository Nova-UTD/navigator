from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim_tasks', executable='mode_toggle',   name='mode_toggle', output='screen'),
        Node(package='turtlesim_tasks', executable='follower_node', name='follower',    output='screen'),
    ])
