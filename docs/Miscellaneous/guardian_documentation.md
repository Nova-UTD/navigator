---
layout: default
title: Guardian
nav_order: 1
parent: Miscellaneous
---

# Guardian Node
{: .no_toc }

*Maintained by Nova*


## Overview
The guardian node is a diagnostics node created to monitor the individual systems running within Navigator. It consolidates data from our various nodes and outputs overall system state, operating mode, and status.

---

### In:
- **/requested_mode** [*Mode*](../messages.md#mode)
  - Receives the requested operationg mode - disabled, manual control, or autonomous control.
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
  - Receives diagnostics information of nodes currently operating in the stack.
- **/planning/path** [*Path*]()
  - Receives a path from our [planning](../Planning/index.md) subsystem.
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)

### Out:
- **/status** [*DiagnosticArray*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html)
  - An array of diagnostic information on each running node on the watchlist, with a global status that reflects the overall state at the end of the array.
- **/guardian/mode** [*Mode*](../messages.md#mode)
  - What mode navigator needs to be in based on whether a safety event violation has triggered a disable of the auto or manual mode.

---

### function(1)
TODO

### function(2)
TODO