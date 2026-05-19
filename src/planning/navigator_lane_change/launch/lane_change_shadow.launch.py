"""
Shadow-mode launch for navigator_lane_change.
Run standalone (not via launch.carla.py) for initial CARLA validation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

NAVIGATOR_DIR = "/navigator/"


def generate_launch_description():
    lane_change_node = Node(
        package="navigator_lane_change",
        executable="lane_change_node",
        name="lane_change_node",
        output="screen",
        parameters=[NAVIGATOR_DIR + "param/lane_change_params.yaml"],
    )

    return LaunchDescription([lane_change_node])
