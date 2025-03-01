---
layout: default
title: Third-Party Subtrees
parent: Tools
---

# System overview
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

We use the `git` `subtree` functionality to embed third-party libraries (repositories) as subdirectories within our repositories. This allows us to import them in our ROS packages.

## In the `navigator` Repository
1. We bring in our own navigator_msgs repository. This allow us to modify the ROS messages in that repository in one place and have them be used in several of our repositories:
```
# To make the subtree for navigator_msgs:
git subtree add --prefix=src/msg/navigator_msgs --squash https://github.com/Nova-UTD/navigator_msgs.git main
# To pull new changes:
git subtree pull --prefix=src/msg/navigator_msgs --squash https://github.com/Nova-UTD/navigator_msgs.git main
```
2. Similarly, we also bring in ROS' CARLA messages:
```
# To make the subtree for ros-carla-msgs: 
git subtree add --prefix=src/msg/ros-carla-msgs --squash https://github.com/carla-simulator/ros-carla-msgs.git master
# To pull new changes:
git subtree pull --prefix=src/msg/ros-carla-msgs --squash https://github.com/carla-simulator/ros-carla-msgs.git master
```
3. The `ros2_numpy` repository facilitates translating between ROS messages and numpy:
```
# To make the subtree for ros2_numpy:
git subtree add --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git humble
# To pull new changes:
git subtree pull --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git humble
```

## In the `carla_interface` Repository
1. As before, we bring in Nova's `navigator_msgs` repository:
```
git subtree add --prefix=src/msg/navigator_msgs --squash https://github.com/Nova-UTD/navigator_msgs.git main
git subtree pull --prefix=src/msg/navigator_msgs --squash https://github.com/Nova-UTD/navigator_msgs.git main
```
2. As before, we bring in the `ros2_numpy` repository (note that the carla_interface uses ROS2 `foxy`):
```
git subtree add --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git foxy-devel
git subtree pull --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git foxy-devel
```

## In the `vehicle_interface` Repository
1. This repository also uses the `ros2_numpy` repository:
```
git subtree add --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git humble
git subtree pull --prefix=src/tools/ros2_numpy --squash https://github.com/Box-Robotics/ros2_numpy.git humble
```
