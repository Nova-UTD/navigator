"""This module contains the launch description for the vehicle."""

from os import path
import sys

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart

sys.path.append(path.abspath('/navigator/'))
from launches.launch_node_definitions import *
from launches.utils import err_fatal, try_get_launch_description_from_include, IncludeError


def generate_launch_description():
    """Map manager launch file description."""

    return LaunchDescription([
        map_manager_carla,
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_manager_carla,
                on_start=[
                    LogInfo(msg='Map Manager Started... launching the rest of Navigator...'),
                    routing_monitor,
                    
                ]
            )
        )
    ])