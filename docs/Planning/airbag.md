---
layout: default
title: Airbag
nav_order: 1
parent: Planning
---

# airbag_node.py
{: .no_toc }

*Maintained by Nova*

## Overview
Code to establish safety zones around the car where the speed is limited.

---

### In:
- **/lidar/filtered** [*PointCloud2*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) 
- **/control/unlimited** [*CarlaEgoVehicleControl*](https://github.com/Nova-UTD/navigator/blob/dev/src/msg/ros-carla-msgs/msg/CarlaEgoVehicleControl.msg) 
- **/carla/hero/speedometer** [*CarlaSpeedometer*](https://github.com/Nova-UTD/navigator/blob/dev/src/msg/navigator_msgs/msg/CarlaSpeedometer.msg)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html) 

### Out:
- **/carla/hero/vehicle_control_cmd** [*CarlaEgoVehicleControl*](https://github.com/Nova-UTD/navigator/blob/dev/src/msg/ros-carla-msgs/msg/CarlaEgoVehicleControl.msg)
- **/status/airbags** [*DiagnosticStatus*](https://github.com/Nova-UTD/navigator/blob/dev/src/msg/ros-carla-msgs/msg/CarlaEgoVehicleControl.msg)
- **/visuals/airbags** [*Marker*](https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)
- **/planning/current_airbag** [*String*](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)

---

### speedCb
Updates current_speed with new speep.

### distanceToSpeedLimit
Maps distance to max speed, limiting speed as little as possible.

### lidarCb
Considers points in front of the car and not too far off to the side. It assume car is 2 meters wide.

### commandCb
Checks if current speed is greater than speed limit, prints warning and applyes brake proportionally to speed over limit.

