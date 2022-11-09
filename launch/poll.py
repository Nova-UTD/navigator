import logging
import subprocess
import time
from launch.processes import nodes

TIME_FROM_LAST_CHECK = time.time() # Time from last sanity check
SANITY_FREQUENCY = 1 # Sanity check frequency in seconds


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
    found_nodes: dict = {} # Dictionary to keep track of nodes
    
    for exec_name, node_name in nodes.items(): # Traverse through node dictionary
        if node_name != "launch": # Get all processes except for launch executable
            found_nodes[node_name] = False

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
        
        for node in found_nodes: # Loop through initially launched nodes
            if node in node_name: # If we find our node alive, then set alive to True
                found_nodes[node] = True
    
    for name, value in found_nodes.items(): # Loop through initially launched nodes
        if value == True: # If node value is true, then it's still alive
            pass
            #print(f"<{name}> node is in state SUCCESS")
        else:
            print(f"<{name}> node is in state WARNING")