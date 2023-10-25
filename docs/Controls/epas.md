---
layout: default
title: EPAS
nav_order: 2
parent: Controls
---

# Electric Power Assisted Steering
{: .no_toc }

*Maintained by Nova*

## Overview
The Electric Power Assisted Steering (EPAS) node provides an interface between ROS and our EPAS system using CAN bus. Inputting and parsing vehicle control command, it sends control commands over CAN, as wel as receives incoming CAN messages.

---

### In:
- **command_sub** *VehicleControl*
- **clock_sub** [*clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **current_mode_sub** *Mode*

### Out:
- **status_pub** *DiagnosticStatus*
- CAN Bus: *Torque Command*

---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla