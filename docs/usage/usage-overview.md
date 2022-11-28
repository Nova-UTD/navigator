---
layout: default
title: Usage
nav_order: 5
---

# Usage overview
{: .no_toc }

*Maintained by Raghav Pillai*

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

Navigator uses a layer built on top of roslaunch to provide cleaner usage and keep track of important node information. 


## Usage Overview
_This assumes you're using our development environment with Docker._
1. Ensure your workspace is sourced using `source install/setup.bash`
2. Run `navlaunch.py` using `python3 navlaunch.py`
3. When prompted, select the verbosity level you'd like to use, [INFO, WARN, FATAL]. 
    - INFO will report all node outputs, WARN will only report node outputs of warning or higher, and FATAL will only report outputs that are determined to be fatal. 
    - Note that setting your verbosity level to FATAL will not silence node warning statuses. 
4. Select if you'd like to use the CARLA sim node or not [YES or NO]. 


## Launch System Overview:

* navlaunch.py
    - Main launch file that handles launching of ROS processes and holds ownership of modules and objects. 
    - Starts main ros launch process on the main thread, and sends each line to a new thread. Also calls sanity check on another thread. 
    - Handles all prompts under `prompt_` functions. 

* launch/logger.py
    - Sets up logs and log file in `/nav_logs` folder. Treated as a module. 
    - If no `/nav_logs` folder is found, then logger will attempt to create one. 
    - Creates a log instance for the top level process. 
    - Each session creates a log under /nav_logs/navigator-log_<date_time>.log 

* launch/message.py
    - Holds message functions, message levels and types. Treated as objects. 
    - The MessageLevels dictionary is solely used for converting a string level to an integer level. 
    - MessageLevel is an IntEnum to keep track of integer warning states with a defined object. 
    - The MessageType IntEnum is designed to keep track of what type of message a Message object is. Each character represents a property inputted (Process, Level, Timestamp, Node, Information). 
    - The Message Object holds an individual ROS message (console output). Holds the following information in instance:
        - type[MessageType]: Message Type (PLTNI, PTI, etc)
        - Process[str]: Process name that the message finds
        - Level[MessageLevel]: MessageLevel of the info [INFO, WARN, FATAL]
        - Timestamp[float]: Timestamp of message, assigned during instantiation
        - Node[str]: Name of package name, derived from "nodes" dict in `processes.py`
        - Information[str]: Actual information from message

* launch/patterns.py
    - Functions to handle parsing of different regex patterns. Treated as a module.
    - Function names are defined using the following acronyms for output patterns:
        - P: Process
        - L: Level
        - T: Timestamp
        - N: Node
        - I: Information
    - Each of these functions extract data from the output message and form this into a Message object. 

* launch/processes.py
    - Handles instantiation, changing, checking and parsing of node and node statuses. Treated as a module. 
    - At the moment, node executable names and package names are converted _manually_, meaning you'll have to add your new executables to the "nodes" dictionary in format of "<executable/process-name>: <package/node-name>". Failure to do such can result in a KeyError.
    - Calls sanity check (call_node_sanity_check) in frequency of SANITY_FREQUENCY. At the moment this is only activated when a new message is discovered. 

* launch/text_colors.py
    - Quick colors to use for outputting to console. Used like HTML tags, end with ENDC always. Treated as a class


## Troubleshooting:

- If there is no output after launching, ensure you have `install/setup.bash` sourced. 