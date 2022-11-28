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
    *This assumes you're using our development environment with Docker*
    1. Ensure your workspace is sourced using `source install/setup.bash`
    1. Run `navlaunch.py` using `python3 navlaunch.py`
 

## Launch System Overview:

* navlaunch.py
    - Main launch file that handles launching of ROS processes and holds ownership of modules and objects. 

* launch/logger.py
    - Sets up logs and log file in /log/ folder. Treated as an object. 

* launch/message.py
    - Holds message functions, message levels and types. Treated as objects. 

* launch/patterns.py
    - Functions to handle parsing of different regex patterns. Treated as a module. 

* launch/processes.py
    - Handles instantiation, changing, checking and parsing of node and node statuses. Treated as a module. 

* launch/text_colors.py
    - Quick colors to use for outputting to console. Used like HTML tags, end with ENDC always. Treated as a class


## Troubleshooting:

- If there is no output after launching, ensure you have `install/setup.bash` sourced. 