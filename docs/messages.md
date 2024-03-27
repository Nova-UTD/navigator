---
layout: default
title: Navigator Messages
nav_order: 2
---

# Navigator's Custom ROS Messages
{: .no_toc }

Maintained by Daniel Vayman, last updated November 2nd, 2023

## [Here's a link](https://github.com/Nova-UTD/navigator/tree/dev/src/msg/navigator_msgs/msg) to our custom messages directory within Navigator if you need all the details.

### BoundingBox3D
- Corners ([Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html)[8])
- Coordinates (float64[7])

### CarlaGnssRoute
- Road Options
  - VOID (-1), LEFT (1), RIGHT (2), STRAIGHT (3), LANEFOLLOW (4), CHANGELANELEFT (5), CHANGELANERIGHT (6)
- Coordinates ([NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html))

### CarlaRoute
- Road Options
  - VOID (-1), LEFT (1), RIGHT (2), STRAIGHT (3), LANEFOLLOW (4), CHANGELANELEFT (5), CHANGELANERIGHT (6)
- Coordinates ([Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html))

### CarlaSpeedometer
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Speed (float)

### CostedPath
- Points ([Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html))
- Safety Cost (float)
- Routing Cost (float)

### CostedPaths
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Paths ([CostedPath](#costedpath)[])

### Egma (Evidential Grid Map Array)
- Egma ([OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html))
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Goal Point ([Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html))

### FinalPath
- Points ([Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html)[])
- Speeds (float[])

### GPSDiagnostic
- Nano (int)
- GPS Fix (uint)
- Valid Date (bool)
- Valid Time (bool)
- Fully Resolved (bool)
- GPS Fix Ok (bool)
- Diff Soln (bool)
- Wknset (bool)
- Towset (bool)
- Head Veh Valid (bool)

### Masses
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Occupied cells (float[])
- Free cells (float[])
- Width (int)
- Height (int)

### Mode
- Mode
  - Disabled (0), Manual (1), Auto (2)

### Object3D
- Label (uint8)
  - PEDESTRIAN (0), CYCLIST (1), CAR (2), OTHER (99)
- ID (uint32)
- Confidence Score (float32)
  - [0, 1]
- Bounding Box ([BoundingBox3D](#boundingbox3d))

### Object3DArray
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Objects ([Object3D](#object3d)[])

### PedalPosition
- Data (float)
  - [0.0, 1.0] *1.0 is fully depressed*

### PolygonArray
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Polygons ([Polygon](https://docs.ros2.org/latest/api/geometry_msgs/msg/Polygon.html)[])
### Prediction
- Prediction ([OccupancyGrid](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)[])

### RadarSpotlight
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Track ID (int)
- Amplitude (float)

### RouteCost
- Lane ID (int)
- Cost (float)

### RouteCosts
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Costs ([RouteCost](#routecost)[])

### SteeringPosition
- Data (float)
  - [.78, -.78] *radians*

### TrajectoryPoint
- X (float)
- Y (float)
- vx (float)
- vy (float)

### Trajectory
- ID (uint)
- Selected (uint)
- Points ([TrajectoryPoint](#trajectorypoint)[])

### Trajectories
- Trajectories ([Trajectory](#trajectory)[])

### VehicleControl
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Throttle (float)
  - [0., 1.]
- Steer (float)
  - [-1., 1.]
- Brake (float)
  - [0., 1.]
- ~~Hand Brake (bool)~~
- Reverse (bool)
- ~~Gear (int)~~
- ~~Manual Gear Shift (bool)~~

### VehicleSpeed
- [Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)
- Speed (float)