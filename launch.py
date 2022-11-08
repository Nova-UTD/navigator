import logging
import subprocess
import signal
import re
import os
import time
import datetime
from enum import IntEnum
from typing import Pattern

processes = {}
messages = []

log = logging.getLogger("navigator")
logging.basicConfig(
    filename='log.log',
    #encoding='utf-8',
    level=logging.INFO
)

nodes = {
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
    "motion_planner": "motion_planner_node"
}

class bcolors:
    HEADER = '\033[95m'
    ORANGE = '\033[1;33m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

MessageLevels = {
    'INFO': 0,
    'WARN': 1,
    'WARNING': 2
}

class MessageLevel(IntEnum): 
    INFO = 0
    WARN = 1
    WARNING = 2

class MessageType(IntEnum):
    PLTNI = 0
    PTI = 1
    LPI = 2
    PI = 3

def parse_process(process: str) -> str:
    try: 
        return nodes[process]
    except:
        print(f"{bcolors.FAIL}{bcolors.BOLD}{process} NOT FOUND, PACKAGE NOT DEFINED{bcolors.ENDC}")
        return process

class Message(object):
    def print_items(self, output: bool) -> None:
        color: str = ""
        if self.level == MessageLevel.INFO:
            processes[self.process] = "SUCCESS"
        elif self.level == MessageLevel.WARN:
            color = bcolors.WARNING
            processes[self.process] = "WARNING"
        elif self.level == MessageLevel.WARNING:
            color = bcolors.FAIL
            processes[self.process] = "WARNING"

        ts: datetime = datetime.datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S:%f')
        our_str: str =  f"{color} [{str(self.level)[13:]}] [{str(ts)}] [{self.process}]: {self.info} {bcolors.ENDC}"
        str_2: str =  f"[{str(self.level)[13:]}] [{str(ts)}] [{self.process}]: {self.info}"

        if output:
            print(
                f"{color} [{str(self.level)[13:]}] [{str(ts)}] [{self.process}]: {self.info} {bcolors.ENDC}"
            )

        if self.level == MessageLevel.INFO:
            log.info(str_2)
        elif self.level == MessageLevel.WARN:
            log.warning(str_2)
        elif self.level == MessageLevel.WARNING:
            log.warning(str_2)

    def confirm_level(self) -> None:
        int_level: int = MessageLevels[self.level]
        self.level = MessageLevel(int_level)
        
        self.process = parse_process(self.process.split("-")[0])

    def __init__(self, msg_type: MessageType) -> None:
        self.type: MessageType = msg_type
        
        self.process: str = None
        self.level: str = "INFO"
        self.timestamp = time.time()
        self.node: str = None
        self.info: str = None

def parse_PLTNI(message: str) -> Message: # [ObstacleZonerLaunch-9] [INFO] [1666749993.288106915] [planning.obstacle_zoner]: Start ob
    """
    Parses string of pattern [PROCESS] [LEVEL] [TIMESTAMP] [NODE]: INFO
    """
    split_messages: list = message.split(' ', 4)
    process: str = split_messages[0][1:].split("]")[0]
    level: str = split_messages[1][1:].split("]")[0]
    timestamp: str = split_messages[2][1:].split("]")[0]
    node: str = split_messages[3][1:].split("]")[0]
    info: str = message.split(':')[1][1:]

    msg = Message(MessageType.PLTNI)
    msg.process = process
    msg.level = level
    #msg.timestamp = timestamp
    msg.node = node
    msg.info = info
    
    return msg

def parse_PTI(message: str) -> Message: # [static_transform_publisher-5] 1666749993.283289 [0] static_tra
    """
    Parses string of pattern [PROCESS] [TIMESTAMP]: INFO
    """
    split_messages: list = message.split(' ', 3)
    process: str = split_messages[0][1:].split("]")[0]
    timestamp: str = split_messages[1].split("]")[0]
    info: str = message.split(':')[1][1:]

    msg = Message(MessageType.PTI)
    msg.process = process
    #msg.timestamp = timestamp
    msg.info = info
    
    return msg

def parse_LPI(message: str) -> Message: # [INFO] [ZoneFusionLaunch-8]: process starte
    """
    Parses string of pattern [LEVEL] [PROCESS]: INFO
    """
    split_messages: list = message.split(' ', 3)
    level: str = split_messages[0][1:].split("]")[0]
    process: str = split_messages[1][1:].split("]")[0]
    info: str = message.split(':')[1][1:]

    msg = Message(MessageType.LPI)
    msg.level = level
    msg.process = process
    msg.info = info
    return msg

def parse_PI(message: str) -> Message: # [robot_state_publisher-6] Link ardu
    split_messages = message.split(' ', 2)
    process = split_messages[0][1:].split("]")[0]
    info = message.split('] ')[1]

    msg = Message(MessageType.PI)
    msg.process = process
    msg.info = info
    return msg

def node_init(message: str) -> None:
    exec_process: list = message.split("] [")[1].split("-")[0]
    node_process = parse_process(exec_process)
    processes[node_process]: str = "SUCCESS"
    print(f"{bcolors.HEADER}Successfully initialized Node {bcolors.CYAN}<{node_process}>{bcolors.ENDC} EXECUTED AS ({exec_process})")

def start_main_launch(level) -> None:
    #subprocess.Popen(". /opt/ros/foxy/setup.bash", executable="/bin/bash", shell=True)
    #subprocess.Popen(". install/setup.bash", executable="/bin/bash", shell=True)
    process: subprocess = subprocess.Popen(
        "ros2 launch ros_launch.py",
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        executable="/bin/bash",
        shell=True
    )
    
    for line in iter(process.stdout.readline, ''):  # With Python 3, you need iter(process.stdout.readline, b'') (i.e. the sentinel passed to iter needs to be a binary string, since b'' != '')
        line: str = line.decode('utf-8').rstrip()

        if line == "" and line is not None:
            continue

        pattern_PLTNI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\] \[[0-9]*\.[0-9]+\] \[[^\]]*\]:", re.IGNORECASE)
        pattern_PTI: Pattern = re.compile(r"\[[^\]]*] [0-9]*\.[0-9]+ \[0\]", re.IGNORECASE)
        pattern_LPI: Pattern = re.compile(r"\[[^\]]*\] \[[^\]]*\]:", re.IGNORECASE)
        pattern_PI: Pattern = re.compile(r"\[[^\]]*\]\s([A-Za-z0-9]+( [A-Za-z0-9]+)+)", re.IGNORECASE)

        msg: Message = None
        if pattern_PLTNI.match(line): # PROCESS LEVEL TIMESTAMP NODE INFO
            msg = parse_PLTNI(line)
        elif pattern_PTI.match(line): # PROCESS TIMESTAMP INFO
            msg = parse_PTI(line)
        elif pattern_LPI.match(line): # LEVEL PROCESS INFO
            msg = parse_LPI(line)
            if "process started with pid" in line:
                node_init(line)
        elif pattern_PI.match(line): # PROCESS INFO
            msg = parse_PI(line)
        else:
            print("UNRECOGNIZED MSG: " + str(line))

        msg.confirm_level()

        msg.print_items(msg.level >= level)

def prompt_start() -> str:
    print(f"\t{bcolors.ORANGE}🅽 🅰 🆅 🅸 🅶 🅰 🆃 🅾 🆁{bcolors.ENDC}") 
    print(f"{bcolors.BOLD}{bcolors.GREEN}Initializing Navigator...{bcolors.ENDC}")
    level: str = ""
    while level != "INFO" and level != "WARN" and level != "WARNING":
        level = input(f"{bcolors.BOLD}{bcolors.GREEN}Enter message level{bcolors.ENDC}{bcolors.BOLD} [INFO, {bcolors.WARNING}WARN{bcolors.ENDC}{bcolors.BOLD}, {bcolors.FAIL}WARNING{bcolors.ENDC}{bcolors.BOLD}]:{bcolors.ENDC}\n")

    print(f"{bcolors.UNDERLINE}{bcolors.ORANGE}Successful input, running Navigator{bcolors.ENDC}\n\n{bcolors.BOLD}-------------------{bcolors.ENDC}")
    return MessageLevel( MessageLevels[level] )
    
if __name__ == "__main__":
    os.setpgrp() # create new process group, become its leader

    try:
        level: MessageLevel = prompt_start()
        start_main_launch(level)
    finally:
        os.killpg(0, signal.SIGKILL) # kill all processes in process group