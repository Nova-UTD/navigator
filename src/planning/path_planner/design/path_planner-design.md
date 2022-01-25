Path planner {#path-planner}
===========

This is the design document for the `path_planner` package.

# Purpose / Use cases
Path Planner is used to determine trajectory
It will use centerline of lanes obtained from HADMap, convert it to cubic spline, and convert it into a set of potential trajectories. It selects the trajectory that minimizes a cost function.