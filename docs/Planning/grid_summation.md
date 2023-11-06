---
layout: default
title: Grid Summation
nav_order: 2
parent: Planning
---

# Grid Summation Node
{: .no_toc }

*Maintained by Nova*

## Overview
This node subscribes to cost maps, calculates the weighted sum, and publishes the final cost map. The different grids include futureOccupancy, currentOccupancy, and others, which encompass the driveable surface and the route distance. The grids have to be fresh, so a method in this node checks the grids to make sure they are less than .4 seconds old. The occupancy grid is then weighted and resized, and then turned into a cost map which is published.  

---

### In:
- **current_occupancy_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **future_occupancy_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **drivable_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **junction_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **route_dist_grid_sub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **speed_sub** [*CarlaSpeedometer*](../messages.md#carlaspeedometer)
- **clock_sub** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **speed_cost_map_sub** [*Path*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)


### Out:
- **junction_occupancy_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **steering_cost_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **speed_cost_pub** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **combined_egma_pub** [*Egma*](../messages.md#egma-evidential-grid-map-array)
- **status_pub** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)

---

### checkForStaleness(self, grid: OccupancyGrid, status: DiagnosticStatus)
Checks the grid to see if it's old. It gets the timestamp on the grid and converts it into seconds, then measures it against the staleness tolerance. If it is, it checks the next layer to determine if more than one layer is stale. 


### fastForward(self, grid: OccupancyGrid)
This method manipulates the occupancy grid by resizing it, transforming it, changing the metadata of the grid, and then returning it. The grid is converted into a numpy array and then resized depending on the original grid's size. The method then tries to find transformations between the frames, then extracts rotation angles, and calculates new coordinates. From this information, a new occupancy grid is made and returned. 

### buildRouteCostmap(self)
This method creates a costmap for the given route. It then transforms the route points to a specific frame and updates the grid with these points. The modified costmap is then returned. 

### getWeightedArray(self, msg: OccupancyGrid, scale: float)
This method converts the occupancy grid into a numpy array and then multiplies it by scale. 

### resizeOccupancyGrid(self, original: np.ndarray)
This method resizes the occupancy grid by removing every sixth row and every sixth column. It then takes three columns off and returns the final grid. 

### createCostMap(self)
createCostMap computes steering and speed cost maps for a navigation system. It starts by initializing two grids for steering and speed costs. It then processes several types of grids and combines their weighted contributions into the steering and speed cost maps. Additionally, it incorporates a route cost map into the steering cost. These cost maps are then formatted into OccupancyGrid messages and published.


