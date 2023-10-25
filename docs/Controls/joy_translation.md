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
Vehicle/Control message type to be used for TBS (throttle/brake/steering).

---

### In:
- **joy_sub** [*joy*](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)
- **speed_sub** *CarlaSpeedometer*
- **clock_sub** [*clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **current_mode_sub** *Mode*

### Out:
- **command_pub** *VehicleControl*
- **requested_mode_pub** *Mode*
- **status_pub** *DiagnosticStatus*


---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla