---
layout: default
title: Road User Detection
nav_order: 7
parent: Perception
---

# Road User Detection
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A ROS Node that uses a YOLO11 model to classify the different road users (e.g. pedestrians, cars, buses, bicycles, etc.).

---

### In:

- **/cameras/camera0** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.


### Out:

- **/road_users/detections** [*RoadUserDetections*](../messages.md#roaduserdetections)
  - Publishes an array of Road User messages.

---

### image_callback(self, msg: Image)
Takes in a camera image as input, performs a classification by calling the make_detections(self) function, then publishes the results as a `RoadUserDetections` message.