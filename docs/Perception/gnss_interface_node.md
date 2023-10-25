---
layout: default
title: Node Template
nav_order: 1
parent: Perception
---

# gnss_interface_node
{: .no_toc }

*Maintained by Nova*

## Overview
The `GnssInterfaceNode` is a ROS 2 node responsible for interfacing with a GNSS (Global Navigation Satellite System) receiver in a robotic or autonomous system. It reads GNSS data, processes it, and publishes critical information in the form of ROS messages. These messages include vehicle speed, diagnostic status, odometry (position and velocity), and GNSS position data (latitude, longitude, and altitude). The node ensures that the GNSS data is accurately integrated into the ROS 2 ecosystem, enabling other components and nodes to utilize this information for navigation and situational awareness.

---

### In:
- **clock_sub** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)

### Out:
- **speed_pub** [*CarlaSpeedometer*](https://github.com/Nova-UTD/navigator/blob/dev/src/msg/navigator_msgs/msg/CarlaSpeedometer.msg)
- **status_pub** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
- **odom_pub** [*Odometry*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)
- **navsat_pub** [*NavSatFix*](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)

---

### Individual Function 1
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2
blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla
blabla bla bla blabla blablabla blabla bla bla blabla blablabla
