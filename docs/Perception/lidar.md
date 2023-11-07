---
layout: default
title: Lidar Processor
nav_order: 2
parent: Perception
---

# Lidar Processing Node
{: .no_toc }

*Maintained by Nova*

## Overview

This node is responsible for processing our 128-ring Ouster LiDAR data. It listens to points published by a ROS LiDAR driver, crops vehicle points, transforms them into the vehicle reference frame, and publishes to the ```/lidar``` topic.

---

### In:
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
  - This message provides the current time. The node subscribes to this topic and updates its internal clock based on this message.
- **/ouster/points** [*PointCloud2*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)
  - This message holds PointCloud2 data from the ROS2 Driver that translates packets from the LiDAR.

### Out:
- **/lidar** [*PointCloud2*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)
  - Transformed, cropped, and unfiltered LiDAR points.

---

### transformToBaseLink(self, pcd: np.array, source_frame: str)
Transforms the input point cloud into the base_link frame of the vehicle. It performs both translation and rotation operations to align the point cloud data with the coordinate system of Hail Bopp.

### remove_ground_points(self, pcd: np.array, height: float)
Removes points below a certain height from the input point cloud.

### remove_nearby_points(self, pcd: np.array, x: tuple, y: tuple)
Removes points in a rectangle around the sensor from the input point cloud.

### remove_points_above(self, pcd: np.array, height: float)
Removes points above a certain height from the input point cloud.