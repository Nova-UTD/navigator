---
layout: default
title: routing_monitor
nav_order: 1
parent: Mapping
---

# Routing Monitor
{: .no_toc }

*Maintained by Nova*

## Overview
The Routing Monitor Node is responsible for monitoring and processing route information for navigation purposes. It subscribes to the `/planning/rough_route` topic to receive a rough route, processes it, and republishes the smoothed route on the `/planning/smooth_route` topic. Additionally, it provides a service for setting the route and handles clock synchronization.


---

### In:
- **rough_route_sub** [*Path*](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)

- **smooth_route_sub** [*Path*](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)

- **clock_sub** [*Clock*](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html)


### Out:

- **self.smooth_route_pub** [*Path*](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)

---

### smooth_route_pub_tick(self):
Continuously publishes the smoothed route with synchronized timestamps. It ensures that the route message is updated with the latest clock information.

### request_refined_route(self):
Requests a refined route from the MapManager service. It takes the initial and destination points from the rough route, constructs a request, and sends it to the service. Once the request is sent, the function tracks its state to prevent multiple requests and ensures that the route is refined only once.