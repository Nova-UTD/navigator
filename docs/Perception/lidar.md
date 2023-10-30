---
Layout: default
Title: Junction Manager
Parent: Perception
---
# \<Dual lidar processing node\> 


Maintained by Nova(Vindhya Kaushal)

## Overview

This node responsible for processing LiDAR data. It is a ROS2 node for fusing, filtering, and processing LiDAR data from two sensors. The node subscribes to two PointCloud2 messages, one from each LiDAR sensor. It then transforms the point clouds into the base_link frame of the vehicle, removes ground points, and publishes the fused and filtered point cloud to the /lidar topic.

### Subscriber:

**left_lidar_sub** [*PointCloud2*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)

Purpose: Subscribes to PointCloud2 messages from the left LiDAR sensor. It subscribes to PointCloud2 messages from the topic /velo_left/velodyne_points. The data from this subscriber is processed in the leftLidarCb callback method. 


**right_lidar_sub** [*PointCloud2*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)

Purpose: It subscribes to PointCloud2 messages from the topic /velo_right/velodyne_points. The data from this subscriber is processed in the rightLidarCb callback method.


**clock_sub** [*Clock*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Clock.html)

Purpose: This subscriber is used Subscribes to Clock messages from the ROS2 system clock.


### Publisher:

**clean_lidar_pub** [*PointCloud2*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)

Purpose: Publishes a PointCloud2 message containing the fused and filtered point cloud data from the two LiDAR sensors.



### Individual Function 1

blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2

blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla blabla bla bla blabla blablabla blabla bla bla blabla blablabla
