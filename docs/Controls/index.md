---
layout: default
title: Controls
nav_order: 7
has_children: true
---

# Controls
{: .no_toc }

*Maintained by Nova*

---

## Our controls subsystem is the final subsystem of Navigator. The controller inputs a trajectory from our [planning subsystem](../Planning/index.md) and outputs to our vehicle actuation nodes which interface with our hardware.

Our controller takes into consideration the physical constraints of our vehicle (speed, acceleration, turn radius, braking strength) and outputs appropriate actuation commands to correct for any gaps between planned trajectory and vehicle driving capability while still staying on target.
