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
- A dedicated GPU with at least 6 GB of memory, ideally 8 GB

## Installation
1. [Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) if you haven't already done so
2. Clone our repository
```
$ git clone --recursive https://github.com/Nova-UTD/navigator
$ cd navigator
```
3.  Build with colcon
```
$ colcon build
```

That's it!

## CARLA demo
1. Make sure you've installed Navigator using the steps above.
2. Source the workspace by running `$ source install/setup.bash` from our repo root.
3. Open Rviz2 and CARLA. We use the latest version of CARLA in our testing, which is [0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13) at time of writing.
4. Launch our demo with `$ ros2 launch carla.launch.py`

At this point, Navigator will connect to CARLA over our custom bridge.