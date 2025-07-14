---
layout: default
title: LiDAR Detection Node
nav_order: 6
parent: Perception
---

# Lidar 3D Object Detector Node
{: .no_toc }

*Maintained by Gueren Sanford and Ragib Arnab*

## Overview
The LiDAR Detection Node performs 3D object detection on LiDAR pointclouds. The detection must be done using raw pointcloud data to optimize the detection rate. The node has two different methods for detection:

### Complex YOLOv4
The `complex_yolov4_model` turns pointcloud data into a birdseye view map of the points. The model uses these 2D images to form 3D detections. The implementation was adpated from [this](https://github.com/maudzung/Complex-YOLOv4-Pytorch) GitHub repo. Look at `complex_yolov4_model` for an example of how to integrate + organize a custom neural network into navigator. More information:
- Average Hz:
  - 20.5
- Optimal confidence threshold:
  - 0.9
- Pros: 
  - Higher accuracy
  - Many detections
- Cons:
  - More hallucinations

### MMDetection3D
The `mmdetection3d_model` uses pointcloud data to form 3D detections. This implementation uses the [MMDetection3D](https://mmdetection3d.readthedocs.io/en/latest/) API for its model intialization and inferencing. The API supports a variety of LiDAR vehicle detection models, which can be switched by changing the `config_path` and `checkpoint_path`. More information:
- Average Hz:
  - 23.5
- Optimal confidence threshold:
  - 0.4
- Pros: 
  - More reliable
  - Interchangable model files
- Cons:
  - Low confidence scores

---

### Parameters:

- **device** *str*
  - The device the model will run on. Choices:
    - "cuda:0" (DEFAULT)
    - "cuda:{NUMBER}"
- **model** *str*
  - The type of model making the detections. Choices:
    - "mmdetection3d" (DEFAULT) 
    - "complex_yolo"
- **conf_thresh** *float*
  - The mininum confidence value accepted for bounding boxes. Choices: 
    - 0.7 (DEFAULT) 
    - 0.0 to 1.0
- **nms_thresh** *float*
  - The maximum accepted intersection accepted for bounding boxes. Choices: 
    - 0.2 (DEFAULT) 
    - 0.0 to 1.0 

---

### In:

- **/lidar** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)
  - Receives raw LiDAR point cloud data.


### Out:

- **/detected/objects3d** [*Object3DArray*](../messages.md#object3darray)
  - Publishes an array of 3D objects.

---

### lidar_callback(lidar_msg: PointCloud2)
Uses the lidar msg and a 3D object deteciton model to form 3D bouding boxes around objects. Then, publishes the boxes to a 'detected' topic.
