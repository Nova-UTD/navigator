---
layout: default
title: Offline Lidar SLAM
nav_order: 3
parent: Mapping
---

# Offline Lidar SLAM
{: .no_toc }

*Maintained by Ragib Arnab*

## Overview
The Offline Lidar SLAM Python package is responsible for mapping out any environment given consecutive lidar scans in an offline fashion. It uses [KISS-ICP](https://github.com/PRBonn/kiss-icp) for odometry, [MapClosures](https://github.com/PRBonn/MapClosures) for loop closure detection, and [g2o](https://github.com/RainerKuemmerle/g2o) for pose graph optimization. The package is meant for "offline" use, meaning use on recorded ROS bags without any real-time restrictions. The output is a folder containing all local maps along with their poses in a text file, which can be used fairly easily to stitch together the local maps to form a global map. The output of this SLAM pipeline can be also be used for localization.

---

### offline_lidar_slam.py
A Python script that performs offline lidar SLAM on recorded ROS bags.
#### Command Line Arguments:
- **bag_path**:
    - path to the ROS bag you want to apply lidar SLAM
- **topic_name**:
    - name of the topic containing the lidar messages of type [PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
- **save_dir**:
    - directory to which SLAM results will be saved to; automatically created if it does not exist

#### Example Usage
```
$ python3 /navigator/src/mapping/offline_lidar_slam/offline_lidar_slam.py /bags/2024/bag_2024-02-09_Demo/ /lidar /navigator/data/maps/slam_results/
```
