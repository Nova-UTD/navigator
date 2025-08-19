from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='turtlesim_tasks', executable='spawn_second', name='spawn_second', output='screen'),
        Node(package='turtlesim_tasks', executable='mouse_goal_publisher', name='mouse_goal', output='screen'),
        Node(package='turtlesim_tasks', executable='turtle1_controller',   name='turtle1_controller', output='screen'),
    ])
