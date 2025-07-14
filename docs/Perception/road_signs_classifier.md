---
layout: default
title: Road Signs Classifier
nav_order: 7
parent: Perception
---

# Road Signs Classifier
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A ROS Node that uses the inference-sdk to make API calls and classify the road signs within an image.

---

### In:

- **/cameras/camera0** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.


### Out:

- **/road_signs/detections** [*RoadSignsDetection*](../messages.md#roadsignsdetection)
  - Publishes an array of road sign detections.

---

### image_callback(self, msg: Image)
Takes in a camera image as input, performs a classification by calling the classify_sign(self) function, then publishes the results as a `RoadSignsDetection` message.