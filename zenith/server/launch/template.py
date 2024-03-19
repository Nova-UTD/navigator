# Base launch file used for launch file creation.
# We currently assume launch_node_definitions.py will exist in the same directory as our launch file.
LAUNCH_FILE_TEMPLATE = """
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from launch_node_definitions import map_manager_carla

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
