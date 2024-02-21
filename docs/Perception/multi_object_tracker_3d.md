---
layout: default
title: 3D Multi-Object Tracker
nav_order: 7
parent: Perception
---

# 3D Multi-Object Tracker Node
{: .no_toc }

*Maintained by Ragib Arnab*

## Overview
A ROS node which performs tracking-by-detection to assign unique IDs to objects from 3D object detection. Estimates the states of objects using a Kalman filter with a constant acceleration model and then applies greedy matching to match bounding boxes. More details available in the [source repository](https://github.com/hailanyi/3D-Multi-Object-Tracker).

Parameters to tune the tracker can be found under `/navigator/param/perception/lidar_objdet3d_params`.

---

### In:

- **/objdet3d_raw** [*Object3DArray*](../messages.md#object3darray)
  - Receives an array of untracked 3D objects.


### Out:

- **/objdet3d_tracked** [*Object3DArray*](../messages.md#object3darray)
  - Publishes an array of tracked 3D objects, where each object now has a unique ID.

---

### detection_callback(detection_msg: Object3DArray)
Takes in untracked 3D objects as input, performs tracking-by-detection and assigns unique IDs, then publishes the results as an `Object3DArray` message.
