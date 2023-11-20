---
layout: default
title: Linear Actuator
nav_order: 4
parent: Controls
---

# Linear Actuator Node
{: .no_toc }

*Maintained by Nova*

## Overview
This node's responsible for our brake system. It communicates via CAN bus with a linear actuator that pulls our brake pedal. 

---

### In:
- **/vehicle/control** [*VehicleControl*](../messages.md#vehiclecontrol)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **/guardian/mode** [*Mode*](../messages.md#mode)

### Out:
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
- CAN Bus: *Position/Clutch Command*

---

### connectToBus(self)
Attempts to establish a connection to the CAN bus that controls our linear actuator.

### sendBrakeControl(self, msg: VehicleControl)
Calculates a position command from an inputted brake value and calls [`sendToPosition()`](#sendtopositionself-pos-float-bus-canbus).

### enableClutch(self, bus: can.Bus)
Enables clutch.

### disableClutch(self)
Disables clutch.

### sendToPosition(self, pos: float, bus: can.Bus)
Encodes a desired position value for the linear actuator into serial and attempts to send it.