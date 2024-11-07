"""This module contains the launch description for the perception stack."""

from os import path
import sys

from launch import LaunchDescription

sys.path.append(path.abspath("/navigator/"))
from launches.launch_node_definitions import *
from launches.utils import (
    err_fatal,
    try_get_launch_description_from_include,
    IncludeError,
)

def generate_launch_description():
    # Include 3D object detection launch file and extract launch entities for reuse.
    try:
        object3d_launch_description = try_get_launch_description_from_include(
            "launches/launch.object3d.py"
        )
    except IncludeError as e:
        err_fatal(e)

    object3d_launch_entities = object3d_launch_description.describe_sub_entities()
    
    road_signs_classifier = Node(
    	package='road_signs_classifier',
    	executable='road_signs_classifier',
    	name='road_signs_classifier',
    	parameters=[],
    )
    
    return LaunchDescription([
        # image_segmentation,
        # semantic_projection,
        # ground_seg,
        # static_grid,
        # *object3d_launch_entities,
        # traffic_light_detector,
        # prednet_inference,
        # driveable_area,
        road_signs_classifier,
    ])
