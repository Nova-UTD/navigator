"""This module provides templates for launch files."""

# Base launch file used for launch file creation.
# We currently assume launch_node_definitions.py will exist in the same directory as our launch file.
LAUNCH_FILE_TEMPLATE = """
import sys
from os import path

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

sys.path.append(path.abspath('/navigator/'))
from launches.launch_node_definitions import *

def generate_launch_description():
    return LaunchDescription([
        map_manager_carla,
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_manager_carla,
                on_start=[
                    LogInfo(msg='Map Manager Started... launching the rest of Navigator...'),
                ]
            )
        )
    ])"""
