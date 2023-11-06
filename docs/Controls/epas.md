---
layout: default
title: EPAS (Steering)
nav_order: 2
parent: Controls
---

# Electric Power Assisted Steering (EPAS) Node
{: .no_toc }

*Maintained by Nova*

## Overview
The Electric Power Assisted Steering (EPAS) node provides an interface between ROS and our EPAS system using CAN bus. Inputting and parsing vehicle control command, it sends control commands over CAN, as well as receives incoming CAN messages.

---

### In:
- **/carla/hero/vehicle_control_cmd** [*VehicleControl*](../messages.md#vehiclecontrol)
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
- **/guardian/mode** [*Mode*](../messages.md#mode)

### Out:
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
- CAN Bus: *Torque Command*

---

### connectToBus(self)
Attempts to establish a connection with the EPAS CAN bus.

### parseIncomingMessages(self, msg1_data: bytearray, msg2_data: bytearray)
Parses incoming CAN messages, extracting data such as torque, duty cycle, current, supply voltage, temperature, and other diagnostics.

### sendCommand(self, target, bus)
Calculates and sends a CAN message to our EPAS to adjust the steering angle based on the current vehicle angle and the target angle. It uses a proportional–integral (PI) controller for error correction.