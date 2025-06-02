---
layout: default
title: Drivable Surface Segmentation
nav_order: 7
parent: Perception
---

# Drivable Surface Segmentation
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A set of three ROS nodes (Image segmentation, Depth processing, and Occupancy grid) that determine what surfaces can be driven on (road) and which cannot be driven on (everything else).

---

### In:

- **/ouster/range_image** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.

- **/ouster/signal_image** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image taken by the front camera.


### Out:

- **/segmentation_mask** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Publishes a binary mask where white represents drivable areas and black represents non-drivable areas.

- **/processed_depth** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Publishes a depth map image representing the distance from the camera to the point in the image.

- **/occupancy_grid** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
  - Publishes an occupancy grid where unoccupied cells represent drivable areas and occupied cells represent non-drivable areas.

---
## Occupancy Grid Node:

### generate_occupancy_grid(self)
Takes in the binary mask and depth map images as input, projects these into 3D space, then bins the coordinates onto a 2D occupancy grid representing drivable and non-drivable cells

### process_segmentation(self, msg)
Stores the segmented binary mask in CV2 format for usage when generating the occupancy grid

### process_depth(self, msg)
Stores the segmented depth map image in CV2 format for usage when generating the occupancy grid


## Depth Processing Node: 

### process_depth(self, msg)
Use the image from topic /ouster/range_image to generate a depth map image


## Image Segmentation Node:

### image_callback(self, msg)
Store the image from topic /ouster/signal_image in CV2 format, call the run_sam_segmentation(self, image) function, then publish the returned binary mask image

### run_sam_segmentation(self, image)
Use a custom trained SAM2 model to generate multiple binary masks representing drivable and non-drivable surfaces and return the best one