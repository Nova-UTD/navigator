---
layout: default
title: Pedestrian Intent To Enter Road
nav_order: 7
parent: Perception
---

# Pedestrian Intent To Enter Road
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A ROS Node that identifies pedestrians in the scene, determines the direction they are facing and their distance to the road, and uses this information to classify whether a pedestrian will be entering the road.

---

### In:

- **/cameras/camera0** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.

- **/segmentation_mask** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives a binary mask image that corresponds to the image taken by the front camera.


### Out:

- **/pedestrians** [*PedestrianInfoDetections*](../messages.md#pedestrianinfodetections)
  - Publishes an array of pedestrian detections.

---

### image_callback(self, msg: Image)
Takes in a camera image as input and stores it for use by other parts of the node.

### binary_mask_callback(self, msg: Image)
Takes in a binary mask image as input and stores it for use by other parts of the node.

### detect_pedestrians(self)
Identifies pedestrians in the scene, determines the direction they are facing, uses the calculate_horizontal_distance() method, and then publishes the results of each pedestrian detection.

### calculate_horizontal_distance(self, bounding_box_coordinates, direction_facing)
Calculates the distance between the pedestrian and the road to determine intent of entering the road.