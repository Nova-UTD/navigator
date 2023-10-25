---
layout: default
title: Node Template
nav_order: 2
parent: Perception
---

# static_occupancy_node
{: .no_toc }

*Maintained by Nova*

## Overview
The `StaticOccupancyNode` is a ROS 2 node responsible for generating static occupancy and mass grids based on ground-segmented pointclouds provided by the 
[`ground_segmentation_node`](https://github.com/jpahm/navigator/blob/documentation/docs/Perception/ground_segmentation_node.md). 
These grids are then consumed by the 
[`junction_manager`](https://github.com/jpahm/navigator/blob/documentation/docs/Planning/junction_manager.md) 
and the 
[`grid_summation_node`](https://github.com/jpahm/navigator/blob/documentation/docs/Planning/grid_summation_node.md) 
for planning purposes.

The **occupancy grid** is generated using 
[Dempster-Shafer Theory (DST)](https://en.wikipedia.org/wiki/Dempster%E2%80%93Shafer_theory) 
with a process which roughly consists of three steps:
- Ray-tracing free space toward occupied space
- Filling the remaining grid with free space, also using ray-tracing
- Adding an occupied space representing the vehicle

The **mass grid** is generated via the following steps:
- Updating the previous mass grid via a decay factor
- Updating probabilities of this decayed grid using a [Bayes filter](https://en.wikipedia.org/wiki/Recursive_Bayesian_estimation)

These grids are then published separately, with the occupancy grid data being populated by the averaged
occupied and free probability values obtained via generation of the mass grid.

---

### In:
- **clock_sub** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **pcd_sub** [*PointCloud2*](https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud2.html)

### Out:
- **occupancy_grid_pub** [*OccupancyGrid*](https://docs.ros2.org/galactic/api/nav_msgs/msg/OccupancyGrid.html)
- **masses_pub** [*Masses*](https://github.com/Nova-UTD/navigator/blob/documentation/src/msg/navigator_msgs/msg/Masses.msg)

---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla
