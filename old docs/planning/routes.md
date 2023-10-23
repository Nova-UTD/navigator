---
layout: default
title: Routes
---

# Routes

_Maintained by Will Heitman_

A route is a sequence of poses that our AV is encouraged to follow. At its simplest, a route constitutes the start and goal poses. However, routes can contain many poses in between.

The AV's goal is to _loosely_ follow the poses contained in the route. How loosely? That depends on the priorities given to the downstream motion planner. In general, though, we consider that the AV has reached a pose along the route when it is within about 10 meters. That means that if we drive through the lane adjacent to the pose, or if we park the vehicle near the goal pose, for example, the route is still satisfied.

This loose approach is in contrast to the specific path that the downstream motion planner calculates. That path, which factors in the route as well as occupancy and other perception data, serves as a precise target for the car. In other words, the car does not have to drive over the route, but it should drive over the motion planner's path.

As an example, consider making a turn at an intersection. The route planner will set the route to be the precise centerline of the turn lane, but human drivers will follow a curve that bends slightly away from the center, creating a smoother ride. We want to give Navigator the flexibility to make these kinds of adjustments.

## Route format

Let's define some operational constraints on the route.

First, the route will be formatted as a standard ROS [Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html) message, where each pose is stamped with a timestamp reflecting the general goal arrival time. For example, the final pose in the Path message (that is, the goal pose of the route) may have a timestamp set to fifteen minutes past the current time for a neighborhood route four miles long.

Second, these goal timestamps should be taken lightly. Unlike the timestamp from the motion planner, these should serve as loose targets, just like the poses themselves.

Third, the route should be updated routinely to ensure that any poses already reached are removed from the route. In other words, all poses in the latest route should be ahead of the car in terms of route progression.

Fourth, the route should be divided into a **refined** and a **rough** section. In the rough section, the route poses have _no maximum spacing_. The rough section should have the goal pose as its final point, and it should always succeed the refined section.

The refined section extends from the car's current position to the area immediately ahead in the route, with the section's length determined by a "lookahead distance" set as a node parameter. In the refined section, the route poses should have a spacing of between 0.5-10 meters (see the table below).

As the car progresses along the route, the refined section will move with it, refining more and more of the route. When the goal pose is sufficiently close, the refined section will cover the entirety of the remaining route, and the rough section will be empty.

![Route sections](assets/res/routing-sections.png)

### Message

```
# Path.msg containing the route
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
```

### Parameters

| Parameter                                     | Worst case                      | Best case                           |
| --------------------------------------------- | ------------------------------- | ----------------------------------- |
| Spacing between refined section as crow flies | 10m (seen along straight roads) | 0.5 meters (seen along tight turns) |
|                                               |                                 |                                     |
|                                               |                                 |                                     |

## Route generation

1. Receive rough route from CARLA
2. Get closest route point to car
3. Trim all points not within interval (closestPoint, goalPoint), where the goal point is the final point in the list
4. While distanceFromCar < some param, insert refined points. using HD map data for lane centerlines
