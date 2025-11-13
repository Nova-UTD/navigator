"""Launch file for autonomous cruise controller."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for autonomous cruise controller."""
    # Get package directory
    pkg_dir = get_package_share_directory('autonomous_cruise')

    # Parameters file
    params_file = os.path.join(pkg_dir, 'params', 'autonomous_cruise_params.yaml')

    # Autonomous cruise controller node
    autonomous_cruise_node = Node(
        package='autonomous_cruise',
        executable='autonomous_cruise_node',
        name='autonomous_cruise_controller',
        output='screen',
        parameters=[params_file],
        emulate_tty=True
    )

    return LaunchDescription([
        autonomous_cruise_node
    ])
