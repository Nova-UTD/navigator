---
layout: default
title: Planning
nav_order: 4
---

# Planning overview

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

"Planning" refers to the part of the system that synthesizes perception and 
scenario information into an actionable decision that can be handed to controls.
Our current control stack is designed for Demo 2 tasks, at its applicability to 
other tasks is limited. 

## The Strategy
The current strategy generates a determined path (i.e. spatial trajectory), 
which it assigns velocities to based on the situation. There are currently three
modifying factors on assigned velocity: the speed limit, the curvature of the 
path (to avoid taking sharp turns at high speeds), and most importantly zones. 

A "zone" is a closed polygon that limits the speed that the vehicle can have 
when passing through the enclosed space. They are generated from a variety of 
sources, and are used to control the vehicles behavior: if any node wishes to
stop the vehicle from entering an intersection, for example, they would enclose
the intersection with a zone of speed 0.

## Zone Features
  * Zones are defined by the points describing an enclosed polygon and a 
    non-exceed speed
  * Any single zone is homogenous, meaning the non-exceed speed is constant 
    within the zone
  * Zones may overlap. In this case, the lowest non-exceed speed must be obeyed
  * Zones may have a speed of 0
  * Zones don't evolve through time: instead, a new zone must replace the old
    zone

## Calculating the Trajectory: Assigning speed to the path
  The path is turned into a trajectory by assigning speed. The trajectory should
  be safe, comfortable, and possible, and should obey the zones. The process for
  assigning velocities works as follows:
   1. For each point, set its speed to the speed limit of the lane
   2. For each point, calculate the local curvature of the path. Using the path
      curvature, limit the speed for that point so that the lateral acceleration
      of the vehicle is never more than a configured maximum
   3. For each zone, determine if it intersects with the trajectory. If it does:
      1. The trajectory is a discrete path, so the first point affected may be 
        further from the edge of the zone than we would like. To observe the 
        speed limitation starting from the very edge of the zone, insert a new 
        point to the trajectory where it intersects the zone (both entering and
        exiting)
      2. Set the speed of this new point to the lowest speed among its adjacent
        points the zone
      3. For all points within the zone, set the speed to no greater than the 
        zone speed
   4. Do a backwards pass of the trajectory, and lower the speed of each point
      so that the speed of the next point (next in time, previous point in the 
      pass) can be achieved with comfortable deceleration. The only time this 
      may not be physically obtainable is immediately in front of the vehicle 
      when it is moving too fast: in this case, lower the speed anyway and let 
      the controller sort it out
   5. Do a forwards pass of the trajectory, and lower the speed of each point so
      it can be reached from the previous point using a comfortable acceleration

  Although the path itself may extend from the origin to the destination, only 
  the points within a certain horizon of the vehicle need to be considered for
  the trajectory.

  Notice that aside from step 1, the speed is never increased. The vehicle should
  be cautious, meaning that if there is a reason to go slow and a reason to go
  regular speed, the reason to go slow wins as a rule of thumb. Since this is 
  true, as long as acceleration/deceleration smoothing is done last, the other
  steps can be done in any order.

```cpp
void MotionPlannerNode::send_message() {
    if (ideal_path == nullptr || odometry == nullptr) {
        // RCLCPP_WARN(this->get_logger(), "motion planner has no input path, skipping...");
        return;
    }
    Trajectory tmp = build_trajectory(ideal_path, horizon);
    if(zones != nullptr){
      limit_to_zones(tmp, *zones);
    }
    limit_to_curvature(tmp, max_lat_accel);
    smooth(tmp, max_accel, max_decel);
    trajectory_publisher->publish(tmp);
    return;
}
```
## Zone sources
  Zones primarily come from two sources: the obstacle detection system and 
  the `BehaviorPlanner` node. 

  The obstacle detection system will create a zone around each obstacle. It will
  actually typically create two zones: one zone with a speed of 0 signifying not
  to move through the space at all, and one larger zone with a lower speed 
  indicating where the vehicle may pass but should be cautious.

  One limitation of the zone architecture is that obstacle zones need to be 
  larger than the actual obstacles. This is because speeds are assigned as 
  intersections with the trajectory, which has zero width. The car does not have
  zero width, and so can physically go through zones that the path did not 
  intersect.

  At this time, the primary function of the BehaviorPlanner node is traffic
  control. Using a state machine, it will create zones around intersections, 
  and then remove them when it is safe to proceed. 

  ## Strengths and Weaknesses
  
  Strengths:
   * Zones are an easily understood description of what the car is doing
   * Zones can accomplish any sort of stop-go behavior we need, including 
     following another vehicle
  
  Weaknesses:
   * Zones cannot describe dynamic scenarios
   * Zones cannot describe uncertain or branching scenarios
   * Only as strong as the zone-creating entities
   * Can be awkward for the controller, like when it is halfway out of a zone with 0 speed