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
- [**Docker**](https://docs.docker.com/desktop/)
- [**CARLA 0.9.13**](https://carla.readthedocs.io/en/latest/start_quickstart/) for simulated testing (optional)
- For best results, a dedicated GPU with at least 6 GB of memory, ideally 8 GB
- x86_64 CPU
- About 20 GB of disk space

## Installation

> Note: Lines with "$" are on host, while lines with "#" are within the container.

1. [Install Docker](https://www.docker.com/get-started/) if you haven't already done so
2. If not already set, choose a number between 0-100 and use this for your user-wide ROS_DOMAIN_ID. Add this to your `.bashrc` with `echo 'export ROS_DOMAIN_ID=[your number]' >> ~/.bashrc`
2. Clone our repository
```
$ git clone --recursive https://github.com/Nova-UTD/navigator
$ cd navigator
```
3. Build and start our Docker container
```
$ docker build . -t navigator
$ ./docker.sh
```
which gives:
```
[username]@justingpu:~/navigator$ ./docker.sh 
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
2. On the host, start CARLA: `./CarlaUE4.sh` See the [CARLA docs](https://carla.readthedocs.io/en/latest/start_quickstart/#running-carla) for more info.

3. Run the following commands to start our system and connect it to CARLA:
```
$ ./docker.sh
...
root@yourhost:/navigator# ./navigator
root@yourhost:/navigator# ros2 launch carla_interface carla.launch.py
```
This should start a series of ROS nodes, spawning an ego vehicle in the simulator along with a number of sensors:
```
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-11-05-01-09-18-915839-justingpu-55
[INFO] [launch]: Default logging verbosity is set to INFO
[WARNING] [launch_ros.actions.node]: Parameter file path is not a file: /navigator/param/planning/motion_planner.param.yaml
[INFO] [unified_controller_node-1]: process started with pid [57]
[INFO] [sim_bridge_node-2]: process started with pid [59]
[INFO] [ukf_node-3]: process started with pid [61]
...
```
4. Visualize the output by starting Rviz2. You can run Rviz2 directly from a second container instance:
```
$ xhost + # Authorize Docker to launch GUI programs
$ ./docker.sh
...
root@yourhost:/navigator# rviz2
```

## Real-world use
To run in the real-world, you'll need to provide your own LiDAR stream, map data, and throttle/brake/steering interface. We provide our own interface, for use with our specific hardware, as an example.

To use, run `./docker.sh` to enter the container, then source Navigator with `. install/setup.bash`, and finally launch the real-world script with `ros2 launch main.launch.py`. If you haven't built the workspace yet, run `colcon build --symlink-install` after sourcing.
