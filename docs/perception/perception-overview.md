---
layout: default
title: Perception
nav_order: 3
---

# Perception overview
{: .no_toc }

*Maintained by Ragib Arnab*

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

The perception component of the system takes the data from various sensors and extracts meaningful information for downstream components such as planning.

Some of the tasks that are core parts of the perception include, but not limited to:
- Localization and state estimation
- Obstacle and scene classifiation
- Obstacle tracking and prediction

In this page we will go each individual packages that is part of the perception component. A lot of the packages within perception are "in-progress" just as with most other packages within our system. As this project progresses, we will update the existing packages as well as add new ones to meet the growing demands of our autonomous system.


## darknet_inference
The darknet_inference package contains Python tools to build and run Darknet-based object detection models in ROS 2. The standard model that is used is YOLOv4 which can achieve real-time inference on a modern GPU with good overall accuracy. There is also an option to use the YOLOv4-tiny model to increase the inference rate but with a sacrifice to accuracy. The node for this package subscribes to a RGB image message topic and outputs 2D bounding box predictions for each class of object in its own message formats. In this current version of navigator, all the detections for the vehicle is performed in this node, which includes detecting cars and pedestrians as well as finding landmarks such as stop signs and fire hydrants. All these detections are also processed within the node itself by using parameters such as object confidence threshold and non-maximum suppression (NMS) threshold to filter out the unwanted detections.

## obstacle_detection_3d
This package takes as input the 2D detections along with 3D sensing data from lidars and depth camera to output 3D bounding boxes. 3D detection are required for behavior and planning components of our system. Currently the code simply backprojects the 2D bounding boxes into 3D using information and extends the boxes into a cuboid based on the object's class. This is a naive approach given that the orientation information of the cuboids will be the same as the vehicle and that the lengths of the objects are fixed. The algorithm is a placeholder and will be replaced by an actual 3-D object detection algorithm in the future.

## obstacle_classes
Contains the enumeration definition for the different classes of obstacles.

## obstacle_drawer
A simple visualization package that takes the 3D bounding box outputs and produces visualization messages that can be depicted in RViz.

## lidar_fusion
This package fuses the 2 different lidar sources within our system into a single point cloud that will be registered into the same frame (base-link) within the system transform tree. The package also performs basic point cloud filtering.

## lidar_obstacle_detector
This package is tasked with detecting low-level obstacles around the vehicle for the purpose of collision prevention. The node takes as input a point cloud and output zones around the vehicle for low-level obstacles.

