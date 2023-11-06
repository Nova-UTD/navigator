---
layout: default
title: Ground Segementation Node
nav_order: 1
parent: Perception
---

# \<Node Title\>
{: .no_toc }

*Maintained by Nova*

## Overview
The code establishes a ROS node which processes LiDAR input, removes ground points using a grid-based MRF method, and publishes the filtered LiDAR data. This segmentation operates within a 2D grid framework and expands from the center to determine whether each grid cell ground points.

---

### In:

- **clock_sub** [*Clock*] (https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_clock.html)

 **raw_lidar_sub** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)

### Out:

- **filtered_lidar_pub** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
