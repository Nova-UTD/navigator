---
layout: default
title: Topics
nav_order: 2
---

# Navigator topics
{: .no_toc }

*Maintained by Daniel Vayman*

### Last Updated: November 2nd, 2023
*Does not include Carla topics*

- `/lidar`: Unfiltered [pointclouds](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html) formatted from our Lidars.
- `/lidar/filtered`: Ground segmented [pointclouds](https://docs.ros2.org/foxy/api/nav_msgs/msg/Pointcloud2.html)
- `/gnss/odometry`: [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) obtained from our GNSS RTK.
- `/speed`: Speed obtained from our GNSS RTK formatted into our [CarlaSpeedometer](messages.md#carlaspeedometer) message.
- `/cameras/cameraX`: [Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html) messages obtained from our physical/simulator cameras.

---

- `/grid/occupancy/current`: A static, current [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) message.
- `/grid/masses`: Similar formatting as the above, only a custom [Masses](messages.md#masses) grid that uses DST & decay for inference models.
- `/grid/junction`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents an intersection.
- `/grid/stateful_junction`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents the same intersection, only high or low cost depending on status.
- `/grid/drivable`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents driveable area (lanes).
- `/grid/route_distance`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents distance to center line of path (cost gradient).
- `/grid/steering_cost`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents steering angle after compounding previous cost maps.
- `/grid/speed_cost`: [OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html) that represents possible speed after compounding previous cost maps.

---

- `/planning/rough_route`: A rough [Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) published by the Carla simulator.
- `/planning/smoothed_route`: A rough [Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) compiled from a pre-defined physical route (linestring).
- `/planning/route`: A final route [Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) translated either from /planning/rough_route (Carla) or /planning/smoothed_route (Navigator).
- `/planning/is_waiting`: A [Bool](https://docs.ros2.org/galactic/api/std_msgs/msg/Bool.html) message that represents a waiting status to enter a junction.
- `/planning/target_speed`: A [CarlaSpeedometer](messages.md#carlaspeedometer) message that holds a target speed used in intersection speed management.

---

- `/joy`: A [Joy](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) message published by ROS2's joystick drivers, used for teleoperation.
- `/requested_mode`: A custom [Mode](messages.md#mode) message that governs our control output (disabled, manual, auto).
- `/vehicle/control`: Custom [VehicleControl](messages.md#vehiclecontrol) that holds Throttle/Brake/Steering(TBS) values used by our controller and sent to our actuation nodes.