---
layout: default
title: Object Detector
nav_order: 3
parent: Perception
---

# Object Detector Node
*Maintained by Nova*

## Overview
This node implements object detection on the front camera feed. It outputs the video as well as "String" containing all the information about the detected objects. This node uses the tensorflow models repository.

---

### In:
- **/cameras/camera0** ([Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)): Shows the front camera view


### Out:
- **/cameras/objectDetection** ([Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)): Displays the front camera view with object detection enabled using Tensorflow
- **/cameras/objectDetection/outputDict** ([String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)): Sends out the Tensorflow output Dictionary which contains information about the objects detected in the camera feed. The output is a String which can be parsed using json and converts into a python dictionary.

---

