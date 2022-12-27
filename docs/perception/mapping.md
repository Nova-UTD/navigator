---
layout: default
title: Localization and Mapping
nav_order: 3
---

# Localization and Mapping

{: .no_toc }

_Maintained by Will Heitman_

## Table of contents

{: .no_toc .text-delta }

1. TOC
   {:toc}

---

## Octomap Module

Using the open-source [Octomap](https://octomap.github.io/) library in conjunction with a particle filter, this module aims to provide a **long-term, simultaneous localization and mapping** solution.

### Target

- Translational accuracy of $\pm 1.0$ meters at worst, $\pm 0.25$ meters nominal in urban environment.
- Heading accuracy of $\pm 30 \degree$ at worst, $\pm 10\degree$ nominal in urban environment.
- Above worst-case accuracy only occurs 1% of the time.
- Able to operate in case of loss from GNSS and/or IMU.

### Behavior

1. On start, the module should attempt to load an existing map using the map name from `/carla/world_info`.
   1. If no file exists, create an empty Octree and save this.
2. When new odometry is received (such as from an IMU), store this as an accumulated offset $(\Delta x, \Delta y, \Delta \theta)$. Update the stored `Odometry` message and publish it with the new offset.
3. When a new LiDAR point cloud is received, feed this into the particle filter.
   1. Update all particles using the accumulated offset and Gaussian noise in a **motion update**.
   2. Evaluate the probability of each particle in an **observation update**.
   3. **Resample** all particles using their probabilities.
   4. Compute a new robot pose using the mean and covariance of the particles.
   5. Publish the pose as an `Odometry` message.
4. After the pose is updated from the LiDAR cloud, this cloud should be added to the map.
5. Steps 2-4 should be repeated in a loop.
6. A timer should prompt a `visualization_msgs/Marker` to be published periodically that visualizes the voxel map.
7. Upon termination, the octree map should be saved to a file using the name from (1).

#### Progressive resolution of visualization

![Progressive map visualization](assets/res/progressive_map_resolution.drawio.png)

### Assumptions

- The robot will only move along a 2D plane. That is, only 3-DOF motion will be assumed, and the localization will be calculated accordingly.
