---
layout: default
title: Map Management
nav_order: 2
parent: Mapping
---

# Map Management Node
{: .no_toc }

*Maintained by Nova*

## Overview
Routinely publishes a `drivable_grid` and `junction_grid` that is centered around the car. Uses the map information from either the `world_info` topic or a parameter to form the grids. Also, publishes a `goal_pose` (rotation) and `route_path` (path) for the car to follow. 

---

### In:
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **/clicked_point** [*PointStamped*](https://docs.ros2.org/latest/api/geometry_msgs/msg/PointStamped.html)
- **/carla/world_info** *CarlaWorldInfo*

### Out:
- **/grid/drivable** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **/grid/junction** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **/planning/smoothed_route** [*Path*](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)
- **/planning/goal_pose** [*PoseStamped*](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)

---

### MapManagementNode::publishSmoothRoute
Once a route is requested and the `route_linestring_` variable is defined, this method will regularly publish the global smoothed route.

### MapManagementNode::publishGrids
**General steps**
- Query the map-wide R-tree to find all lanes within range
- Creates a second, local R-tree and insert all nearby lanes, which were found from the query
- For each row j and column i, query the local R-tree to see if that cell (i,j) is within a lane's bounding box
  - If yes, find out if it is within the actual lane, not just its bounding box. R-trees only calculate for bounding boxes.
    - If yes again, the cell is truly occupied. Append '100' ("occupied") to OccupancyGrid. Otherwise '0'.
- Set OccupancyGrid metadata and return.

## *CURRENTLY BEING ABSTRACTED TO SMALLER NODES*