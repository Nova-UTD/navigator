import sys
import json

from launch import Metadata, LaunchFileNode, LaunchFile, LaunchFileBuilder

LAUNCH_FILE_BASE = """
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

def _parse_create_args(raw_metadata: str, raw_nodes_list: str) -> tuple[Metadata, list[LaunchFileNode]]:
    try:
        metadata = Metadata.from_json(json.loads(raw_metadata))
    except:
        print(f"Invalid metadata format: {Metadata.help()}", file=sys.stderr)
        sys.exit(1)
    
    try:
        nodes = []
        for json_node in json.loads(raw_nodes_list):
            node = LaunchFileNode.from_json(json_node)
            nodes.append(node)
    except:
        print(f"Invalid node format: {LaunchFileNode.help()}", file=sys.stderr)
        sys.exit(1)
    
    return (metadata, nodes)


def create(path, raw_metadata: str, raw_nodes_list: str):
    metadata, nodes = _parse_create_args(raw_metadata, raw_nodes_list)
    builder = LaunchFileBuilder(LAUNCH_FILE_BASE).set_metadata(metadata).set_nodes(nodes)
    launch_file_contents = builder.generate_file()
    try:
        file = open(path, "wt+")
        file.write(launch_file_contents)
    except Exception as e:
        print(f"Failed to write to file: {e.with_traceback()}")
