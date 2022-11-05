---
layout: default
---

{: .fs-6 }
Navigator is a simple, extensible, and open-source autonomous driving framework.

[System overview](/navigator/system-overview){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/nova-utd/navigator){: .btn .fs-5 .mb-4 .mb-md-0 }

## Why Navigator?
Despite major advances in autonomous driving research, there has yet to exist a single framework that is both simple and extensible, all while being public and transparent.

Navigator is our answer to this delimma. It's built on standard technologies, is kept as simple as possible, and its modular design makes adding new features straightforward.

## System requirements
- System running Ubuntu 20.04 LTS or similar ([see here](http://docs.ros.org.ros.informatik.uni-freiburg.de/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html#system-requirements))
  - *[**Docker**](https://docs.docker.com/desktop/) engine already installed on Ubuntu*
- [**Ros2 Foxy**](https://docs.ros.org/en/foxy/Installation.html)
- [**CARLA**](https://carla.readthedocs.io/en/latest/start_quickstart/)
  - A dedicated GPU with at least 6 GB of memory, ideally 8 GB
  - x86/x64 CPU architecture
  - About 20 GB of space

## Installation

> Note: Lines with "$" are on host, while lines with "#" are within the container.

1. [Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) if you haven't already done so
2. Clone our repository
```
$ git clone --recursive https://github.com/Nova-UTD/navigator
$ cd navigator
```
3. Build and start our Docker container
```
$ docker build . -t navigator
$ ./start.sh
```
which gives:
```
wheitman@justingpu:~/navigator$ ./start.sh 
=====================================================================
▀█▄   ▀█▀                   ██                    ▄                   
 █▀█   █   ▄▄▄▄   ▄▄▄▄ ▄▄▄ ▄▄▄    ▄▄▄ ▄  ▄▄▄▄   ▄██▄    ▄▄▄   ▄▄▄ ▄▄  
 █ ▀█▄ █  ▀▀ ▄██   ▀█▄  █   ██   ██ ██  ▀▀ ▄██   ██   ▄█  ▀█▄  ██▀ ▀▀ 
 █   ███  ▄█▀ ██    ▀█▄█    ██    █▀▀   ▄█▀ ██   ██   ██   ██  ██     
▄█▄   ▀█  ▀█▄▄▀█▀    ▀█    ▄██▄  ▀████▄ ▀█▄▄▀█▀  ▀█▄▀  ▀█▄▄█▀ ▄██▄    
                                ▄█▄▄▄▄▀                               
=====================================================================
Developed by Nova, a student-run autonomous driving group at UT Dallas
Find out more at https://nova-utd.github.io/navigator
🦊 Sourcing ROS2 Foxy...
🔗 Configuring the ROS DDS...
🧭 Sourcing Navigator...
🔌 Setting up CARLA API...
root@justingpu:/navigator# 
```
4.  Now that you're in the container, build our ROS workspace:
```
$ colcon build --symlink-install
``` 

That's it!

## CARLA demo
1. Make sure you've installed Navigator using the steps above.
2. Source the workspace by running `$ source install/setup.bash` from our repo root.
3. Open Rviz2 and CARLA. We use the latest version of CARLA in our testing, which is [0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13) at time of writing.
4. Launch our demo with `$ ros2 launch carla carla.launch.py`

At this point, Navigator will connect to CARLA over our custom bridge.