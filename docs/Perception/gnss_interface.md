---
layout: default
title: Gnss Interface
nav_order: 1
parent: Perception
---

# GNSS Interface Node
{: .no_toc }

*Maintained by Nova*

## Overview
The `GnssInterfaceNode` is responsible for interfacing with our GNSS (Global Navigation Satellite System) RTK receiver on Hail Bopp. It reads GNSS data, processes it, and publishes critical information. These messages include vehicle speed, diagnostic status, odometry (position and velocity), and GNSS position data (latitude, longitude, and altitude).

---

### In:
- **/clock** [*Clock*](https://docs.ros2.org/galactic/api/rosgraph_msgs/msg/Clock.html)
  - This message provides the current time. The node subscribes to this topic and updates its internal clock based on this message.

### Out:
- **/speed** [*VehicleSpeed*](../messages.md#vehiclespeed)
  - This message represents the vehicle's speed. The node publishes this message to the /speed topic, giving the current speed of the vehicle based on GNSS data for other nodes to use.
- **/node_statuses** [*DiagnosticStatus*](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
  - This message provides diagnostic status information about the node. It includes details about the node's health and status.
- **/gnss/odometry** [*Odometry*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)
  - This message contains information about the vehicle's position, orientation, and velocity based on GNSS information.
- **/gnss/fix** [*NavSatFix*](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)
  - This message represents the latitude, longitude, and altitude from the GNSS receiver. 

---

### toRadians(degrees: float)
Converts degrees to radians.

### latToScale(lat: float)
Calculates the UTM projection scale based on latitude.

### publishTf(self)
Publishes a transformation (TF) message from a map coordinate frame to a base_link coordinate frame. Uses the TransformBroadcaster to send the transformation.

### getData(self)
Reads data from the GNSS receiver, processes it, and publishes relevant ROS messages. Handles GNSS sentences of types GGA and VTG.

### latlonToMercator(self, lat: float, lon: float, scale: float)
Converts geodetic coordinates (latitude and longitude) to UTM (Universal Transverse Mercator) coordinates in meters.

### getOdomMsg(self, lat: float, lon: float)
Constructs and returns an Odometry message based on latitude and longitude data.

### connectToPort(self)
Attempts to connect to the GNSS receiver through a serial port. It initializes a serial connection and reports the status.

### initStatusMsg(self)
Constructs and returns a DiagnosticStatus message with basic information.

### clockCb(self, msg: Clock)
Updates the node's internal clock based on the received Clock message.

