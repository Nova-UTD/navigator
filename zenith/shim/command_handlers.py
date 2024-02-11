import sys
import os
import json

from launch import Metadata, LaunchFileNode, LaunchFileBuilder, LaunchFileFromExisting

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


def create(path: str, raw_metadata: str, raw_nodes_list: str):
    metadata, nodes = _parse_create_args(raw_metadata, raw_nodes_list)
    builder = LaunchFileBuilder(LAUNCH_FILE_BASE).set_metadata(metadata).set_nodes(nodes)
    launch_file_contents = builder.generate_file()
    try:
        file = open(path, "wt+")
        file.write(launch_file_contents)
    except Exception as e:
        print(f"Failed to write to file: {e.with_traceback()}")



def list(dir_path: str):
    """Lists all valid launch files found in dir_path.

    Response structure:
    {
        "/abs/path/to/launch/file1": {
            metadata: { name: "My Launch File 2" },
            nodes: [{package: "pkg", executable: "executable"}]
        },
        "/abs/path/to/launch/file2": {
            metadata: { name: "My Launch File 2" },
            nodes: [{package: "pkg", executable: "executable"}]
        }
    }
    """
    response = {}

    # Iterate over each file in the directory of launch files
    # and create a response entry for each valid launch file.
    # See launches/launch_file.py for specification of "valid."
    # Each entry is keyed by the files absolute path,
    # and its value is as follows:
    #   {
    #       metadata: { name: "My Launch File" },
    #       nodes: [ { package: "pkg", executable "executable" } ]
    #    }
    # Also see launches/launch_file.py for metadata and node structure.
    for dirpath, _dirnames, filenames in os.walk(dir_path):
        for filename in filenames:
            if filename.endswith('.py'):
                path = os.path.join(dirpath, filename)
                with open(path) as f:
                    launch_file = LaunchFileFromExisting(f.read())
                    if not launch_file.valid():
                        continue

                    abs_path = os.path.abspath(path)
                        
                    response[abs_path] = {
                        "metadata": launch_file.metadata.__dict__,
                        "nodes": [{
                            "package": node.package,
                            "executable": node.executable
                        } for node in launch_file.nodes]

                    }

    print(json.dumps(response, indent=4))


