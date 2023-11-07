---
layout: default
title: Sensors
nav_order: 3
---

# Sensors
{: .no_toc }

*Maintained by Nova*

---

The Sensing system is responsible for processing raw sensor data into a usable form for the Perception system.

For example, raw LiDAR data from our front and rear sensors is merged into a single reference frame, downsampled, and cropped to remove points along the vehicle itself (points of our vehicle’s doors, for example).

Recently, we've added a [vehicle_interface](https://github.com/Nova-UTD/vehicle_interface) repository, strictly responsible for housing our nodes that interface with our sensor and actuation hardware. You'll still find all the documentation for those nodes here in the navigator docs (Perception and Controls).

More info to come! But our filters aren’t reinvinting the wheel.