---
layout: default
title: Airbags
---

# Airbags

_Maintained by Will Heitman_

"Airbags" are what we call safety zones that limit our vehicle's speed. Airbags form a low-level safety system that is one step above a similar automatic emergency braking (AEB) system.

The logic is simple. We establish three safety zones (red, amber, and yellow), where our speed is limited to varying degrees. If any region within these zones is occupied by a LiDAR point, the vehicle's speed will be limited to the zone's value. If no LiDAR point falls within the zone, the speed is not limited.

In the case of a speed limitation, if the car is currently traveling faster than the zone allows, the airbag system will send a 100% brake command.

Airbags extend in front of the vehicle, and slightly to either side.

![Diagram of airbag dimensions](/navigator/assets/res/airbags.png)

<small>Above: Diagram of airbag dimensions</small>
