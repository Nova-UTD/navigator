---
layout: default
title: RTP
nav_order: 4
parent: Planning
---

# Recursive Tree Planner (RTP) Node
{: .no_toc }

*Maintained by Nova*

## Overview
This node (`rtp_node.py`) implements a Recursive Tree Planner (RTP) to generate paths based on cost values. It uses recursion to generate branches (or path segments), assessing the cost for each segment, and eventually selects and publishes the path with the least cost. This node also holds temporary controller logic.

---

### In:
- **/grid/steering_cost** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **/grid/speed_cost** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- **/gnss/odometry** [*Odometry*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)
- **/guardian/mode** [*Mode*](../messages.md#mode)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)

### Out:
- **/planning/path** [*Path*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)
- **/planning/barrier_marker** [*Marker*](https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)
- **/planning/target_speed** [*VehicleSpeed*](../messages.md#vehiclespeed)
- **/vehicle/control** [*VehicleControl*](../messages.md#vehiclecontrol)
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)

---

### Constants and Parameters:
* `N_BRANCHES`: Number of branches generated in each iteration.
* `STEP_LEN`: Length of each path segment.
* `DEPTH`: Depth of recursion.
* `WHEEL_BASE`: Vehicle's wheelbase.
* `MAX_TURN_ANGLE`: Maximum steering angle of the vehicle.
* `COST_CUTOFF`: Cost value above which a segment is considered nonviable.

---

### getBarrierIndex(self, path: CostedPath, map: np.ndarray, width_meters=1.8)
   * Checks a path for collision points.
   * Returns the index of the first pose that would collide with an obstacle.

### getSegment(self, inital_pose: np.ndarray, steering_angle, segment_length: float, res: float, costmap)
   * Determines the cost of a path segment based on the provided steering angle.

### generatePaths(self, depth: int, path: CostedPath, steering_angle, segment_length, res, num_branches, results: list, result_costs, costmap)
   * Recursively generates path branches based on steering angles.
   * Computes segments based on cost.

### startGeneration(self, costmap: np.ndarray, depth=7, segment_length=9.0, branches=7)
   * Commences the path generation process using a recursive approach.

### costMapCb(self, msg: OccupancyGrid)
   * Generates paths based on the incoming cost map data.
   * Determines the best path based on cost.
   * Publishes the least-cost path.