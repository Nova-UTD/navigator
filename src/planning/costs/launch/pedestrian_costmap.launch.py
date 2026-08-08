"""
Shadow-mode launch for pedestrian_costmap_node.

Starts the node with its param file. The node publishes /grid/pedestrian but is
NOT registered in grid_summation_node, so it does not affect the planned path —
it is observable in RViz only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('costs'),
        'param', 'pedestrian_costmap_params.yaml')

    pedestrian_costmap_node = Node(
        package='costs',
        executable='pedestrian_costmap_node',
        name='pedestrian_costmap_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([pedestrian_costmap_node])
