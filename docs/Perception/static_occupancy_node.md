---
layout: default
title: Static Occupancy
nav_order: 2
parent: Perception
---

# Static Occupancy Node
{: .no_toc }

*Maintained by Nova*

## Overview
The `StaticOccupancyNode` is responsible for generating static occupancy and mass grids based on ground-segmented pointclouds provided by the 
[`ground_segmentation_node`](https://github.com/jpahm/navigator/blob/documentation/docs/Perception/ground_segmentation_node.md). 
These grids are then consumed by the 
[`junction_manager`](https://github.com/jpahm/navigator/blob/documentation/docs/Planning/junction_manager.md) 
and the 
[`grid_summation_node`](https://github.com/jpahm/navigator/blob/documentation/docs/Planning/grid_summation_node.md) 
for planning purposes.

The **occupancy grid** is generated using 
[Dempster-Shafer Theory (DST)](https://en.wikipedia.org/wiki/Dempster%E2%80%93Shafer_theory) and [Bresenhaum's Line Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
with a process which roughly consists of three steps:
- Ray-tracing free space toward occupied space
- Filling the remaining grid with free space, also using ray-tracing
- Adding an occupied space representing the vehicle

The **mass grid** is generated via the following steps:
- Updating the previous mass grid via a decay factor
- Updating probabilities of this decayed grid using a [Bayes filter](https://en.wikipedia.org/wiki/Recursive_Bayesian_estimation)

These grids are then published separately, with the occupancy grid data being populated by the averaged
occupied and free probability values obtained via generation of the mass grid.

---

### In:
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **/lidar/filtered** [*PointCloud2*](https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud2.html)

### Out:
- **/grid/occupancy/current** [*OccupancyGrid*](https://docs.ros2.org/galactic/api/nav_msgs/msg/OccupancyGrid.html)
- **/grid/masses** [*Masses*](../messages.md#masses)

---

### `createOccupancyGrid(pcl::PointCloud<pcl::PointXYZI> &cloud)`
Returns a grid using [Dempster-Shafer Theory (DST)](https://en.wikipedia.org/wiki/Dempster%E2%80%93Shafer_theory) by performing the following steps:
1. Ray-traces free space towards recorded points (occupied space)
2. Fills the rest of the grid with free space using same ray-tracing algorithms (can combine steps 1 and 2?)
3. Adds occupied space representing the vehicle

### `add_points_to_the_DST(pcl::PointCloud<pcl::PointXYZI> &cloud)`
Fills and adds points to the DST grid by projecting the pcl points onto the 2D occupancy grid.

### `add_free_spaces_to_the_DST()`
Adds unoccupied spaces to the DST using ray tracing.

### `addEgoMask()`
Adds the vehicle shape to the occupied/unoccupied zones.

### `publishOccupancyGrid()`
Publishes the occupancy and mass grids based on the current probabilities generated via DST.

### `update_previous()`
Sets the "previous" occupied and free zones to the values of the current occupied and free zones.

### `mass_update()`
Updates the current grids with the previous grid values, applied with a decay factor.

### `update_of()`
Updates the current probabilities using a [Bayes filter](https://en.wikipedia.org/wiki/Recursive_Bayesian_estimation),
taking into account both measured and predicted probability values.

### `std::vector<std::vector<float>> getGridCellProbabilities()`
Computes the average of the updated occupancy and free values and returns the resulting cell probabilities.

### `ray_tracing_approximation_y_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive)`
Performs approximate raytracing from the origin to the given x2 and y2 coordinates, incrementing the origin y coordinate according to a slope error given by the distance between the starting and ending y coordinates. Optionally includes the destination coordinate.

### `ray_tracing_approximation_x_increment(int x2, int y2, int flip_x, int flip_y, bool inclusive)`
Performs approximate raytracing from the origin to the given x2 and y2 coordinates, incrementing the origin x coordinate according to a slope error given by the distance between the starting and ending x coordinates. Optionally includes the destination coordinate.

### `ray_tracing_vertical(int x2)`
Performs vertical raytracing along the positive vertical direction (+x), tracing from the origin to the given point.

### `ray_tracing_vertical_n(int x1)`
Performs vertical raytracing along the negative vertical direction (-x), tracing from the given point to the origin.

### `ray_tracing_horizontal(int y2)`
Performs horizontal raytracing along the positive horizontal direction (+y), tracing from the origin to the given point.

### `ray_tracing_horizontal_n(int y1)`
Performs horizontal raytracing along the negative horizontal direction (-y), tracing from the given point to the origin.

### `clear()`
Clears the occupied and free zones.
