---
layout: default
title: Junction Manager
nav_order: 3
parent: Planning
---

# Junction Manager Node
*Maintained by Ayush Sharma*

## Overview
This node controls the behavior of the car in a junction (Intersection). Given a junction grid (where each cell is either within an intersection or not), plus the current ego speed and occupancy grid, this node publishes a map that either includes the junction cost or not. This basically switches a junction's cost on or off, hence allowing or blocking entrance into junctions based on perception data.

---

### In:
- **/grid/occupancy/current** ([OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)): Shows the current grid that is being occupied by the car
- **/grid/junction** ([OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)): Shows the grid of the junction
- **/carla/hero/speedometer** [CarlaSpeedometer](../messages.md#carlaspeedometer): Keeps track of current speed of the car

### Out:
- **/grid/stateful_junction** ([OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)): Defines the current junction cost
- **/node_statuses** ([DiagnosticStatus](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)): Sends out the diagnostic status of the car

---
### resizeOccupancyGrid()
This method resizes the received occupancy grid to match the expected size and format. It downsamples and trims the grid to ensure it has the correct dimensions.

### junctionIsOccupied()
This method checks if a junction is occupied based on the occupancy and junction grids. It erodes both grids to denoise them and then checks for overlap to determine if the junction is occupied.

### egoCanEnter()
This method determines if the ego vehicle can enter a junction. It considers various conditions, including if the ego vehicle has recently stopped, if the junction is occupied, and if the joystick button is pressed.

### getWeightedArray()
This method converts an OccupancyGrid message into a numpy array and scales it by a given factor, returning the resulting array.
