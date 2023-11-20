---
layout: default
title: Joystick Translation
nav_order: 1
parent: Controls
---

# Joystick Translation
{: .no_toc }

*Maintained by Nova*

## Overview
Converts [*joy*](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) messages coming from the ROS2 joystick driver and translates those values into our custom
[`Vehicle/Control`](../messages.md#vehiclecontrol) message type to be used for TBS (throttle/brake/steering).

---

### In:
- **/joy** [*joy*](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
- **/speed** [*CarlaSpeedometer*](../messages.md#carlaspeedometer)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **/guardian/mode** [*Mode*](../messages.md#mode)

### Out:
- **/vehicle/control** [*VehicleControl*](../messages.md#vehiclecontrol)
- **/requested_mode** [*Mode*](../messages.md#mode)
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)


---

### getSpeedAdjustedSteering(self, joystick_pos: float)
Adjusts steering response based off vehicle speed. The higher the speed, the more sensitive the steering. We calculate a lower maximum steer value on either direction using linear scaling on our inputted speed.

### joyCb(self, msg: Joy)
Processes joystick input and publishes vehicle control commands as well as the requested mode