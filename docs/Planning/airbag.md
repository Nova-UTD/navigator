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
"Airbags" are what we call safety zones that limit our vehicle's speed. Airbags form a low-level safety system that is one step above a similar automatic emergency braking (AEB) system.

The logic is simple. We establish three safety zones (red, amber, and yellow), where our speed is limited to varying degrees. If any region within these zones is occupied by a LiDAR point, the vehicle's speed will be limited to the zone's value. If no LiDAR point falls within the zone, the speed is not limited.

In the case of a speed limitation, if the car is currently traveling faster than the zone allows, the airbag system will send a 100% brake command.

Airbags extend in front of the vehicle, and slightly to either side.

![Diagram of airbag dimensions](/navigator/assets/res/airbags.png)

<small>Above: Diagram of airbag dimensions</small>

---

### In:
- **/lidar/filtered** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) 
- **/control/unlimited** [*VehicleControl*](../messages.md#vehiclecontrol) 
- **/carla/hero/speedometer** [*CarlaSpeedometer*](../messages.md#carlaspeedometer)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html) 

### Out:
- **/vehicle/control** [*VehicleControl*](../messages.md#vehiclecontrol)
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

### commandCb(self, msg: VehicleControl)
Checks if current speed is greater than speed limit, prints warning and applies brake proportionally to speed over limit.

