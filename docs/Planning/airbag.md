---
layout: default
title: Airbag
nav_order: 1
parent: Planning
---

# Airbag Node
{: .no_toc }

*Maintained by Nova*

## Overview
Code to establish safety zones around the car where the speed is limited.

---

### In:
- **/lidar/filtered** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) 
- **/control/unlimited** [*CarlaEgoVehicleControl*](../messages.md#vehiclecontrol) 
- **/carla/hero/speedometer** [*CarlaSpeedometer*](../messages.md#carlaspeedometer)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html) 

### Out:
- **/carla/hero/vehicle_control_cmd** [*CarlaEgoVehicleControl*](../messages.md#vehiclecontrol)
- **/status/airbags** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
- **/visuals/airbags** [*Marker*](https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)
- **/planning/current_airbag** [*String*](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)

---

### speedCb(self, msg: CarlaSpeedometer)
Updates current_speed with new speed.

### distanceToSpeedLimit(self, dist: float)
Maps distance to max speed, limiting speed as little as possible.

### lidarCb(self, msg: PointCloud2)
Considers points in front of the car and not too far off to the side. It assume car is 2 meters wide.

### commandCb(self, msg: CarlaEgoVehicleControl)
Checks if current speed is greater than speed limit, prints warning and applies brake proportionally to speed over limit.

