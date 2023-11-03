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

## Our controls subsystem is the final step of Navigator. The controller inputs a trajectory from our [planning subsystem]() and outputs to 

 subsystem takes real-time *perception* and *mapping* data and determines what our vehicle should do. 

Our planning architecture relies on a unique layering of [cost maps](../system-overview.md#cost-maps) to traverse a representation of our environment and produce a "cost-effective" trajectory which ultimately translates to real-world actuation.
