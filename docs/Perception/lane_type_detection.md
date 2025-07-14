---
layout: default
title: Lane Type Detection
nav_order: 7
parent: Perception
---

# Lane Type Detection
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A ROS Node that uses a YOLO11 model to classify the different lanes on a road into their type.

---

### In:

- **/cameras/camera0** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.


### Out:

- **/lane_types/detections** [*AllLaneDetections*](../messages.md#alllanedetections)
  - Publishes an array of lane type detections.

---

### image_callback(self, msg: Image)
Takes in a camera image as input, performs a classification by calling the make_lane_detections(self) function, then publishes the results as a `AllLaneDetections` message.