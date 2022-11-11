import logging
import subprocess
import time
from launch.text_colors import text_colors as colors

TIME_FROM_LAST_CHECK = time.time() # Time from last sanity check
SANITY_FREQUENCY = 1 # Sanity check frequency in seconds

package_statuses = {} # Holder for processes

def initialize_process(package_name: str, process_name: str) -> None:
    """
    Initialized process
    @param package_name[str]    Package name of node
    @param process_name[str]    Name of process/executable of node
    """
    package_statuses[package_name] = "SUCCESS"
    print(f"{colors.HEADER}Successfully initialized Node {colors.CYAN}<{package_name}>{colors.ENDC} EXECUTED AS ({process_name})")


def change_process_status(package_name: str, status: str) -> None:
    """
    Sets a process to a certain status
    @param package_name[str]    Package name of node
    @param status[str]          Status of node to set
    """
    if package_name == "launch": 
        return
    if package_statuses[package_name] != status:
        package_statuses[package_name] = status
        #print(f"{colors.CYAN}<{package_name}>{colors.ENDC} node changed to state {colors.HEADER}{status}{colors.ENDC}")


def check_processes() -> None:
    """
    Loops through processes and checks all nodes
    """
    for package_name, status in package_statuses.items():
        if status != "SUCCESS":
            color = colors.WARNING
            if status == "FATAL":
                color = colors.FAIL
            print(f"{colors.CYAN}<{package_name}>{colors.ENDC} status is currently {color}{status} {colors.ENDC}")


def call_node_sanity_check() -> None:
    """
    Calls sanity check, checks if time from previous check was greater than SANITY_FREQUENCY
    """
    global TIME_FROM_LAST_CHECK
    if (time.time() - TIME_FROM_LAST_CHECK) > SANITY_FREQUENCY: # See if elapsed time from last sanity check was over 1 second
        TIME_FROM_LAST_CHECK = time.time()
        perform_node_sanity_check()


def perform_node_sanity_check() -> None:
    """
    Checks what nodes are active and compares to initial node list
    """

    alive_nodes: dict = {} # Dictionary to keep track of current nodes
    
    for exec_name, node_name in nodes.items(): # Traverse through node dictionary to create 
        if node_name != "launch": # Get all processes except for launch executable
            alive_nodes[node_name] = False

    process: subprocess = subprocess.Popen(
        "ros2 node list",
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        executable="/bin/bash",
        shell=True
    )

    for line in iter(process.stdout.readline, ''): # Loop through piped subprocess output
        line: str = line.decode('utf-8').rstrip() # Decode binary string to utf-8 and remove whitespace
        if line == "" and line is not None: # Break from loop if null or empty string encountered (if process completed)
            break
        
        node_name: str = line.rsplit('/', 1)[-1] # Split raw node name from namespace and get package name
        
        for node in alive_nodes: # Loop through initially launched nodes
            if node in node_name: # If we find our node alive, then set alive to True
                alive_nodes[node] = True

    for name, value in alive_nodes.items(): # Loop through initially launched nodes
        if value == True: # If node value is true, then it's still alive
            pass
            #print(f"<{name}> node is in state SUCCESS")
        else:
            if name != "launch" and name != "ros_client_library":
                change_process_status(name, "FATAL")

    check_processes() # Check processes


def get_node_from_process(process: str) -> str:
    """
    Takes an executable process name from console and matches with node list [ros2 node list] 
    @param process[str]     String process to parse
    """
    try: # Try catch to return process if exists, or show error if not
        return nodes[process]
    except:
        print(f"{colors.FAIL}{colors.BOLD}<{process}> NOT FOUND, PACKAGE NOT DEFINED{colors.ENDC}")
        return process


nodes: dict = { # Node dictionary for converting node executable to package name
    "rcl": "ros_client_library",
    "launch": "launch",
    "unified_controller_node": "unified_controller_node",
    "ukf_node": "localization_map_odom",
    "visualizer": "odr_visualizer_node",
    "static_transform_publisher": "static_transform_publisher",
    "robot_state_publisher": "robot_state_publisher",
    "vt_viz_exe": "vt_viz_node",
    "ZoneFusionLaunch": "zone_fusion",
    "ObstacleZonerLaunch": "obstacle_zoner",
    "BehaviorPlannerLaunch": "behavior_planner",
    "publisher": "path_publisher_node",
    "motion_planner": "motion_planner_node",
    "bridge": "carla_ros_bridge"
}