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

def parse_node_init_message(message: str) -> None:
    """
    Takes console message of node executable initialization and adds it to process tracker
    @param message[str]     Message from console to parse
    """
    exec_name: list = message.split("] [")[1].split("-")[0] # Get node executable
    package_name = get_node_from_process(exec_name) # Convert executable to node package name
    initialize_process(package_name, exec_name)


def process_ros_launch_line(line: str, level: MessageLevel) -> None:
    """
    Takes a line from console output and parses it based on pattern matched
    @param line[str]            Line to process
    @param level[MessageLevel]  Minimum message level to output to console
    """

    # Patterns for different console message types
    pattern_PLTNI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\] \[[0-9]*\.[0-9]+\] \[[^\]]*\]:", re.IGNORECASE)
    pattern_PTI: Pattern = re.compile(r"\[[^\]]*] [0-9]*\.[0-9]+ \[0\]", re.IGNORECASE)
    pattern_LPI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\]:", re.IGNORECASE)
    pattern_PI: Pattern = re.compile(r"\[[^\]]*\]\s([A-Za-z0-9]+( [A-Za-z0-9]+)+)", re.IGNORECASE)

    msg: Message = None # Set message to null type for now
    # Match console line to pattern
    if pattern_PLTNI.match(line): # PROCESS LEVEL TIMESTAMP NODE INFO
        msg = parse_PLTNI(line)
    elif pattern_PTI.match(line): # PROCESS TIMESTAMP INFO
        msg = parse_PTI(line)
    elif pattern_LPI.match(line): # LEVEL PROCESS INFO
        msg = parse_LPI(line)
        if "process started with pid" in line: # If node initialization
            parse_node_init_message(line)
    elif pattern_PI.match(line): # PROCESS INFO
        msg = parse_PI(line)
    else:
        print(f"UNRECOGNIZED MSG: {colors.FAIL}{colors.BOLD}{str(line)}{colors.ENDC}")
        log.error(str(line))

    if msg and msg.process:
        msg.confirm_level() # Confirm level
        msg.print_items(msg.level >= level) # Print items if level is above our standard level


def start_main_launch(level: MessageLevel) -> None:
    """
    Starts main launch process and sends each line to threads
    @param level[MessageLevel]  Minimum message level to output to console
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

    for line in iter(process.stdout.readline, ''):  # With Python 3, you need iter(process.stdout.readline, b'') (i.e. the sentinel passed to iter needs to be a binary string, since b'' != '')
        line: str = line.decode('utf-8').rstrip() # Decode string to utf-8 and remove end whitespace
        if line == "" and line is not None: # If line is null or empty then skip loop
            continue
        Thread(target=process_ros_launch_line, args=(line, level, )).start() # Send line to process to thread [non-yield]
        Thread(target=call_node_sanity_check).start() # Send sanity check to thread [non-yield]


def prompt_initial_input() -> MessageLevel:
    """
    Prints start prompt and input
    @return MessageLevel of inputted message level
    """
    # Splash output
    print(f"\t{colors.ORANGE}🅽 🅰 🆅 🅸 🅶 🅰 🆃 🅾 🆁{colors.ENDC}") 
    print(f"{colors.BOLD}{colors.GREEN}Initializing Navigator...{colors.ENDC}")
    level: str = "" # Set to empty string for input
    while level != "INFO" and level != "WARN" and level != "FATAL": # While loop for input validation
        level = input(f"{colors.BOLD}{colors.GREEN}Enter message level{colors.ENDC}{colors.BOLD} [INFO, {colors.WARNING}WARN{colors.ENDC}{colors.BOLD}, {colors.FAIL}FATAL{colors.ENDC}{colors.BOLD}]:{colors.ENDC}\n")

    print(f"{colors.UNDERLINE}{colors.ORANGE}Successful input, running Navigator{colors.ENDC}\n\n{colors.BOLD}-------------------{colors.ENDC}")
    return MessageLevel( MessageLevels[level] ) # Convert string to int from dictionary and back to MessageLevel enum
    
    
if __name__ == "__main__":
    os.setpgrp() # create new process group, become its leader

    level: MessageLevel = prompt_initial_input()
    start_main_launch(level)
    try: # Try finally to kill all associated processes when finished
        pass
    finally:
        os.killpg(0, signal.SIGKILL) # kill all processes in process group