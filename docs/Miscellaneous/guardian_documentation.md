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

## Notes
1. The guardian node currently tracks all nodes in the navigator stack; it does not track vehicle interface nodes. 
2. The guardian node can be extended to handle more nodes by adding the node to one of the two watch lists and adding diagnostic publishing information to the node being added.
3. Nodes publishing diagnostic information for the guardian node to handle should you the following format: "node_name, status, timestamp"

---

### In:
- **/requested_mode** [*Mode*](../messages.md#mode)
  - Receives the requested operationg mode - disabled, manual control, or autonomous control.
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
  - Receives diagnostics information of nodes currently operating in the stack.
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)

### Out:
- **/status** [*DiagnosticArray*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html)
  - An array of diagnostic information on each running node on the watchlist, with a global status that reflects the overall state at the end of the array.
- **/guardian/mode** [*Mode*](../messages.md#mode)
  - What mode navigator needs to be in based on whether a safety event violation has triggered a disable of the auto or manual mode.