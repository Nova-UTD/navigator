---
layout: default
title: Ground Segmentation
nav_order: 1
parent: Perception
---

# Ground Segmentation Node
{: .no_toc }

*Maintained by Nova*

## Overview
The code establishes a ROS node which processes LiDAR input, removes ground points using a grid-based MRF method, and publishes the filtered LiDAR data. This segmentation operates within a 2D grid framework and expands from the center to determine whether each grid cell ground points.

---

### In:

- **/clock** [*Clock*](https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_clock.html)
  - Receives synchronization data from CARLA's simulation clock.

- **/lidar** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
  - Receives raw LiDAR point cloud data.


### Out:

- **/lidar/filtered** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
  - Publishes LiDAR data with ground points removed.

---

### pointCloudCb(msg: PointCloud2::SharedPtr)
Processes raw LiDAR data by removing ground points and then publishes the filtered version.

### removeGround(raw_cloud: pcl::PointCloud<pcl::PointXYZI>)
Removes ground points from raw LiDAR data using the Markov Random Field method..
