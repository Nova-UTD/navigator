---
layout: default
title: Intersection Manager
nav_order: 7
parent: Planning
---

# Intersection Manager
{: .no_toc }

*Maintained by Pranav Boyapati*

## Overview
A ROS Node that uses various preprocessed data from the car's sensors to determine actions to take when driving through an intersection.

---

### In:

- **/traffic_light/detections** [*TrafficLightDetection*]()
  - Receives information about the upcoming traffic light(s).
- **/road_signs/detections** [*RoadSignsDetection*](../messages.md#roadsignsdetection)
  - Receives information about upcoming road signs
- **/planning/path** [*Path*](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)
  - Receives information about the path the car will be following
- **/speed** [*CarlaSpeedometer*](../messages.md#carlaspeedometer)
  - Receives the speed at which the car is currently traveling
- **/grid/occupancy/current** [*OccupancyGrid*](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
  - Receives information about the current occupancy of the surroundings
- **/lane_types/detections** [*AllLaneDetections*]()
  - Receives information about the types of lanes on the road


### Out:

- **/intersection** [*IntersectionBehavior*](../messages.md#intersectionbehavior)
  - Publishes a string of text about what action to take for proceeding through the intersection.

---

### laneTypeCallback(self, msg: AllLaneDetections)
Takes in a message of type `AllLaneDetections` and extracts lane type information

### occupancyCallback(self, msg: OccupancyGrid)
Takes in a message of type `OccupancyGrid` and extracts information about the occupancy of the surroundings

### trafficLightCallback(self, msg: TrafficLightDetection)
Takes in a message of type `TrafficLightDetection` and extracts traffic light information

### roadSignsCallback(self, msg: RoadSignsDetection)
Takes in a message of type `RoadSignsDetection` and extracts road sign information

### routeCallback(self, msg: Path)
Takes in a message of type `Path` and extracts information about the route the car will follow

### speedCallback(self, msg: CarlaSpeedometer)
Takes in a message of type `CarlaSpeedometer` and extracts information about the vehicle's speed

### classify_intersection_type(self)
Uses traffic light, road sign, and occupancy grid information to determine whether an intersection is traffic light controlled, stop sign controlled, or is a roundabout

### navigate_traffic_light_intersection(self)
Uses information about the traffic light and current speed to determine behavior to follow at this type of intersection

### navigate_stop_sign_intersection(self)
Uses information about the road signs, speed, and occupancy to determine behavior to follow at this type of intersection

### navigate_roundabout(self)
Uses information about the road signs, speed, and occupancy to determine behavior to follow at this type of intersection

### navigate_intersection(self)
This function serves as a control structure that calls other functions as necessary based on various conditions
