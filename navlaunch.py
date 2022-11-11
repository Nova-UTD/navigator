import subprocess
import signal
import re
import os

from threading import Thread
from typing import Pattern

from launch.text_colors import text_colors as colors
from launch.message import get_node_from_process, MessageLevels, MessageLevel, Message
from launch.patterns import parse_PLTNI, parse_PTI, parse_LPI, parse_PI
from launch.processes import initialize_process, call_node_sanity_check
from launch.logger import setup_logs, log

def parse_node_init_message(console_msg: str) -> None:
    """
    Takes console message of node executable initialization and adds it to process tracker
    @param console_msg[str]     Message from console to parse
    """
    exec_name: list = console_msg.split("] [")[1].split("-")[0] # Get node executable
    package_name = get_node_from_process(exec_name) # Convert executable to node package name
    initialize_process(package_name, exec_name)


def process_ros_launch_line(console_msg: str, msg_level: MessageLevel) -> None:
    """
    Takes a line from console output and parses it based on pattern matched
    @param console_msg[str]         Line to process
    @param msg_level[MessageLevel]  Minimum message level to output to console
    """

    # Patterns for different console message types
    #pattern_PLTNI: Pattern = re.compile(r"\[[^\]]*\] \[[A-Za-z0-9]+\] \[[0-9]*\.[0-9]+\] \[[^\]]*\]:", re.IGNORECASE)
    pattern_PLTNI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\] \[[0-9]*\.[0-9]+\] \[[^\]]*\]:", re.IGNORECASE)
    pattern_PTI: Pattern = re.compile(r"\[[^\]]*] [0-9]*\.[0-9]+ \[0\]", re.IGNORECASE)
    pattern_LPI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\]:", re.IGNORECASE)
    pattern_PI: Pattern = re.compile(r"\[[^\]]*\]\s([A-Za-z0-9]+( [A-Za-z0-9]+)+)", re.IGNORECASE)

    msg: Message = None # Set message to null type for now
    # Match console line to pattern
    if pattern_PLTNI.match(console_msg): # PROCESS LEVEL TIMESTAMP NODE INFO
        msg = parse_PLTNI(console_msg)
    elif pattern_PTI.match(console_msg): # PROCESS TIMESTAMP INFO
        msg = parse_PTI(console_msg)
    elif pattern_LPI.match(console_msg): # LEVEL PROCESS INFO
        msg = parse_LPI(console_msg)
        if "process started with pid" in console_msg: # If node initialization
            parse_node_init_message(console_msg)
    elif pattern_PI.match(console_msg): # PROCESS INFO
        msg = parse_PI(console_msg)
    else:
        print(f"{colors.FAIL}{colors.BOLD}UNRECOGNIZED MSG: {console_msg}{colors.ENDC}")
        log.error(str(console_msg))

    if msg and msg.process:
        msg.confirm_level() # Confirm level
        msg.print_items(msg.level >= msg_level) # Print items if level is above our standard level

def open_carla_bridge() -> None:
    process: subprocess = subprocess.Popen( # Open ros2 launch process, create pipe and use shell mode
        "ros2 launch carla_ros_bridge carla_ros_bridge.launch.py",
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        executable="/bin/bash",
        shell=True
    )

    for log_line in iter(process.stdout.readline, ''):  # With Python 3, you need iter(process.stdout.readline, b'') (i.e. the sentinel passed to iter needs to be a binary string, since b'' != '')
        proc_log_line: str = log_line.decode('utf-8').rstrip() # Decode string to utf-8 and remove end whitespace
        if proc_log_line == "" and proc_log_line is not None: # If line is null or empty then skip loop
            continue
        #print(log_line)
        #print(proc_log_line)
        Thread(target=process_ros_launch_line, args=(proc_log_line, msg_level, )).start() # Send line to process to thread [non-yield]

def start_main_launch(msg_level: MessageLevel) -> None:
    """
    Starts main launch process and sends each line to threads
    @param msg_level[MessageLevel]  Minimum message level to output to console
    """
    setup_logs() # Calls logs to setup
    subprocess.Popen(". install/setup.bash", executable="/bin/bash", shell=True) # Source packages
    process: subprocess = subprocess.Popen( # Open ros2 launch process, create pipe and use shell mode
        "ros2 launch ros_launch.py",
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        executable="/bin/bash",
        shell=True
    )

    for log_line in iter(process.stdout.readline, ''):  # With Python 3, you need iter(process.stdout.readline, b'') (i.e. the sentinel passed to iter needs to be a binary string, since b'' != '')
        proc_log_line: str = log_line.decode('utf-8').rstrip() # Decode string to utf-8 and remove end whitespace
        if proc_log_line == "" and proc_log_line is not None: # If line is null or empty then skip loop
            continue
        Thread(target=process_ros_launch_line, args=(proc_log_line, msg_level, )).start() # Send line to process to thread [non-yield]
        Thread(target=call_node_sanity_check).start() # Send sanity check to thread [non-yield]


def prompt_initial_input() -> MessageLevel:
    """
    Prints navigator splash screen, start prompt and input
    @return     MessageLevel of inputted message level
    """
    # Splash output
    print(f"\t{colors.ORANGE}🅽 🅰 🆅 🅸 🅶 🅰 🆃 🅾 🆁{colors.ENDC}") 
    print(f"{colors.BOLD}{colors.GREEN}Initializing Navigator...{colors.ENDC}")
    input_level: str = "" # Set to empty string for input
    while input_level not in ["INFO", "WARN", "FATAL"]: # While loop for input validation
        input_level = input(f"{colors.BOLD}{colors.GREEN}Enter message level{colors.ENDC}{colors.BOLD} [INFO, {colors.WARNING}WARN{colors.ENDC}{colors.BOLD}, {colors.FAIL}FATAL{colors.ENDC}{colors.BOLD}]:{colors.ENDC}\n")

    print(f"{colors.UNDERLINE}{colors.ORANGE}Successful input, running Navigator{colors.ENDC}\n\n{colors.BOLD}-------------------{colors.ENDC}")
    return MessageLevel( MessageLevels[input_level] ) # Convert string to int from dictionary and back to MessageLevel enum


def prompt_carla_bridge() -> None:
    """
    Prints carla prompt
    """
    user_input: str = "" # Set to empty string for input
    while user_input not in ["YES", "NO"]: # While loop for input validation
        user_input = input(f"{colors.BOLD}{colors.HEADER}Initialize CARLA?{colors.ENDC}{colors.BOLD} [{colors.GREEN}YES{colors.ENDC}, {colors.BOLD}{colors.FAIL}NO{colors.ENDC}{colors.BOLD}]:{colors.ENDC}\n")
    
    if user_input == "YES":
        Thread(target=open_carla_bridge).start() # Send sanity check to thread [non-yield]
   

if __name__ == "__main__":
    os.setpgrp() # create new process group, become its leader

    msg_level: MessageLevel = prompt_initial_input()
    prompt_carla_bridge()
    start_main_launch(msg_level)
    try: # Try finally to kill all associated processes when finished
        pass
    finally:
        os.killpg(0, signal.SIGKILL) # kill all processes in process group