---
layout: default
title: 3D Object Detector
nav_order: 6
parent: Perception
---

# Lidar 3D Object Detector Node
{: .no_toc }

*Maintained by Ragib Arnab*

## Overview
A ROS node which performs 3D object detection on LiDAR data using [MMDetection3D](https://github.com/open-mmlab/mmdetection3d) API. 

Any LiDAR object detection models supported by that package can be used by this node. To use a model, specify the path to the config and checkpoint in the parameters file `/navigator/param/perception/lidar_objdet3d_params` and use that when launching this node. In this project, configs and checkpoints are stored under `/navigator/data/perception` in their respective directories.

---

### In:

- **/lidar** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
  - Receives raw LiDAR point cloud data.


### Out:

- **/objdet3d_raw** [*Object3DArray*](../messages.md#object3darray)
  - Publishes an array of 3D objects.

---

### lidar_callback(lidar_msg: PointCloud2)
Takes in LiDAR input, performs 3D object detection using [MMDetection3D](https://github.com/open-mmlab/mmdetection3d) API, and publishes the results as an `Object3DArray` message.
