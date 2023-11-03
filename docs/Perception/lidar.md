---
layout: default
title: Lidar Processor
nav_order: 2
parent: Perception
---

# Lidar processing node


Maintained by Nova

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

The [*remove_ground_points()*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)function removes points below a certain height from the input point cloud. This is done to reduce the amount of data that needs to be processed and to improve the accuracy of the fused point cloud.
It filters out the z coordinate values less than the specified height.
It inputs an array of the pointclouds , and outputs the points belwo the speicfied height.


### Individual Function 2

The [*transformToBaseLink()*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html) function transforms the input point cloud into the base_link frame of the vehicle. It does this by using the tf2_ros.TransformListener class to lookup the transform between the two frames.It performs both translation and rotation operations to align the point cloud data with the coordinate system of the vehicle.


### Individual Function 3

The  [*remove_nearby_points()*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html) function removes points from a point cloud that are within a certain distance of the origin. This is useful for removing points that are close to the LiDAR sensor.

The function works by first converting the point cloud to a numpy array. It then uses the np.logical_and() function to combine two conditions. The first condition checks if the point's x-coordinate and y-coordinate are within a certain distance of the origin. The second condition checks if the point's z-coordinate is greater than a certain height.


### Individual Function 4

The [*remove_points_above()*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html) function removes points from a point cloud that are above a specific height.It operates on a numpy array representing the incoming point cloud  and returns a modified array with points above the specified height removed. 
