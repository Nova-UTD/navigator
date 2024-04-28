"""This module contains the launch description for the perception stack."""

from os import path
import sys

from launch import LaunchDescription

sys.path.append(path.abspath("/navigator/"))
from launches.launch_node_definitions import *


def generate_launch_description():
    return LaunchDescription(
        [
            # image_segmentation,
            # semantic_projection,
            ground_seg,
            static_grid,
            # traffic_light_detector,
            # prednet_inference,
            # driveable_area,
        ]
    )
