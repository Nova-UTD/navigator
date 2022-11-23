Cost Map Maker {#cost-map-maker}
===========

This is the design document for the `cost_map_maker` package.

# Purpose / Use cases
The cost map maker is used to create a cost map in a useable format for a path planner to decide which paths are least 'costly' based on factors such as safety, following traffic laws, using proper/good traffic protocols, etc.

The cost map maker takes in a dynamic occupancy grid in order to determine which paths are safe or follow traffic laws and outputs a cost map that tells the vehicle what areas should be avoided, which is passed to the motion planner.

The cost map maker takes in a velocity of the vehicle and passes it to the motion planner so that it knows what are achievable velocities of a car at each time step 't'.