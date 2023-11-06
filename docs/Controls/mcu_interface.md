---
layout: default
title: MCU Interface
nav_order: 3
parent: Controls
---

# MCU Interface Node
{: .no_toc }

*Maintained by Nova*

## Overview
Serves as our interface between Navigator (ROS) and our Adafruit Grand Central M4 microcontroller (MCU). [`vehicle/control`](../messages.md#vehiclecontrol) messages are parsed and converted into throttle messages to be sent via serial.

---

### In:
- **/carla/hero/vehicle_control_cmd** [*VehicleControl*](../messages.md#vehiclecontrol)
- **/guardian/mode** [*Mode*](../messages.md#mode)

### Out:
- Serial: Throttle \[0, 0.8\]


---

### main(args=None)
Other than initializing the ROS node, it attempts to establish a serial connection to the MCU

### publishCommand(self)
Using an inputted throttle command via the */carla/hero/vehicle_control_cmd* topic, it parses, encodes, and writes the throttle value into serial. The format we use is `"s{throttle}e\r"`