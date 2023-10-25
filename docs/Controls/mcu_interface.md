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
Serves as our interface between Navigator (ROS) and our Adafruit Grand Central M4 microcontroller (MCU). `vehicle/control` messages are parsed and converted into throttle messages to be sent via serial.

---

### In:
- **vehicle_command_sub** *VehicleControl*
- **current_mode_sub** *Mode*

### Out:
- Serial: Throttle \[0, 0.8\]


---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla