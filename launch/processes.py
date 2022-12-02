import logging
import subprocess
import time
import atexit
from launch.text_colors import text_colors as colors
from launch.backgroundscheduler import BackgroundScheduler
from launch.node_dict import node_dict

TIME_FROM_LAST_CHECK = time.time() # Time from last sanity check
SANITY_FREQUENCY = 1 # Sanity check frequency in seconds

package_statuses = {} # Holder for processes
sanity_scheduler = None # Sanity check scheduler (process)


def initialize_process(package_name: str, process_name: str) -> None:
    """
    Initialized process
    @param package_name[str]    Package name of node
    @param process_name[str]    Name of process/executable of node
    """
    package_statuses[package_name] = "SUCCESS"
    print(f"{colors.HEADER}Successfully initialized Node {colors.CYAN}<{package_name}>{colors.ENDC} EXECUTED AS ({process_name})")


def change_package_status(package_name: str, status: str) -> None:
    """
    Sets a process to a certain status
    @param package_name[str]    Package name of node
    @param status[str]          Status of node to set
    """
    if package_name == "launch": 
        return
    if package_statuses.get(package_name) is None:
        print(f"{colors.FAIL}Package <{package_name}> not properly initialized{colors.ENDC}: {status}")
        return
    if package_statuses[package_name] != status:
        package_statuses[package_name] = status
        #print(f"{colors.CYAN}<{package_name}>{colors.ENDC} node changed to state {colors.HEADER}{status}{colors.ENDC}")


def check_processes() -> None:
    """
    Loops through processes and checks all nodes
    """
    print(f"{colors.BOLD}{colors.HEADER}Running Process Check...{colors.ENDC}")
    for package_name, status in package_statuses.items():
        if status != "SUCCESS":
            color = colors.WARNING
            if status == "FATAL":
                color = colors.FAIL
            print(f"\t{colors.CYAN}<{package_name}>{colors.ENDC} status is currently {color}{status} {colors.ENDC}")


alive_procs: dict = {}
def get_alive_procs() -> dict:
    global alive_procs
    if alive_procs == {}:
        alive_procs = {}
        for exec_name, node_name in node_dict.items(): # Traverse through node dictionary to create 
            if exec_name not in ['launch', 'rcl']: # Get all processes except for launch executable
                if alive_procs.get(exec_name) is None:
                    alive_procs[exec_name] = False
    return alive_procs.copy() 


def perform_node_sanity_check() -> None:
    """
    Checks what nodes are active and compares to initial node list
    """
    proc_dict = get_alive_procs()

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
        found_process_name: str = line.rsplit('/', 1)[-1] # Split raw node name from namespace and get package name
        for proc_name in proc_dict: # Loop through initially launched nodes
            if proc_name in found_process_name or found_process_name in proc_name or node_dict[proc_name] in proc_name: # If we find our node alive, then set alive to True
                proc_dict[proc_name] = True
                continue
    
    for name, value in proc_dict.items(): # Loop through initially launched nodes
        if value != True:
            change_package_status(node_dict[name], "FATAL")

    check_processes() # Check processes


def get_node_from_process(process: str) -> str:
    """
    Takes an executable process name from console and matches with node list [ros2 node list] 
    @param process[str]     String process to parse
    """
    try: # Try catch to return process if exists, or show error if not
        return node_dict[process]
    except:
        print(f"{colors.FAIL}{colors.BOLD}<{process}> NOT FOUND, PACKAGE NOT DEFINED{colors.ENDC}")
        return process


def stop_check_loop() -> None:
    global sanity_scheduler¿
    if sanity_scheduler:
        sanity_scheduler.stop()


def start_sanity_check_loop() -> None:
    global sanity_scheduler¿
    atexit.register(stop_check_loop)
    sanity_scheduler = BackgroundScheduler(SANITY_FREQUENCY, perform_node_sanity_check)