---
layout: default
title: Grid Summation
nav_order: 1
parent: Planning
---

# Grid Summation Node
{: .no_toc }

*Maintained by Nova*

## Overview
This node subscribes to cost maps, calcualtes the weighted sum, and publishes the final cost map. The different grids include futureOccupancy, currentOccupancy, and others, which encompass the driveable surface and the route distance. The grids have to be fresh, so a method in this node checks the grids to make sure they are less than .4 seconds old. The occupancy grid is then weighted and resized, and then turned into a cost map which is published.  

---

### In:
- **current_occupancy_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **future_occupancy_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **drivable_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **junction_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **route_dist_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **speed_sub** [*CarlaSpeedometer*]
- **clock_sub** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **speed_cost_map_sub** [*Path*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)


### Out:
- **junction_occupancy_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **steering_cost_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **speed_cost_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **combined_egma_pub** [*Egma*]
- **status_pub** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)

---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla


