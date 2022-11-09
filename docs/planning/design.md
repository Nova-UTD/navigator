---
layout: default
title: Behavior Planning and Controls Design Document (draft)
---

# Behavior Planning and Controls Design Document (draft)
{: .no_toc }

*Maintained by Hansika*
## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Important Definitions Used in Doc:

Configuration Space: set of all configurations of a vehicle

## System overview

### Current system:
#### Planning:
We use zones to determine speed at given way points, acting like an 'on-rails' vehicle. Zones are an enclosed region of space (represented as a polygon) with a maximum speed, which may be 0 to indicate a no-entry zone. Zones may overlap (in which case the lower speed wins). Zones may come from a variety of sources but currently originate from the Behavior Planner (Traffic Planner) and the Obstacle Zoner. Zones are currently not tagged with a type or origin: all zones are anonymous and equal.

#### Controls:
Uses pure pursuit for steering control. The velocity controller is best not mentioned and should be replaced. 

### What Planning intends to accomplish:
 Develop a planning system that can take in a prediction about where cars, pedestrians, and other dynamic agents will be several seconds in the future and how we can find the most efficient trajectory for our autonomous vehicle (hereinafter referred to as the AV) on a short-term distance to get from point A to B (< a hundred feet) on a long-term path determined by widely spaced waypoints (> several miles).

### What Planning needs:
#### Input:
We choose to represent our prediction of the ground-truth state environment surrounding the AV several seconds in advanced using a Dynamic Occupancy Grid. This Dynamic Occupancy Grid will ideally hold information (in the form of probabilities) about whether a given grid cell (xn, yn) in a 2-Dimensional representation of real space (x, y) is occupied by an obstacle at some time tick t. This Dynamic Occupancy Grid will contain an individual grid frame (hereinafter referred to as a frame) for every discrete decisecond time tick for up to 3 seconds  in advance of the current time state t = 0.  Each of the frames in the Dynamic Occupancy Grid will be based on the local coordinate grid system around the AV. 

#### Justification: 
Because safety is the highest priority when designing a AV planning system, our primary goal is to ensure that crashes never occur when our vehicle has the ability to avoid them. For simplicity, we treat all crashes with equal importance, meaning crashing with an animal, pedestrian, or other vehicles are all given equal "badness". Defining a crash as the collision between the bounding boxes of two objects within real space, the only information we need to know is when another object's bounding box will collide with our own. Knowing this, we can track all possible collisions with other dynamic and static objects through creating a Dynamic Occupancy Grid that tells us the spaces that contain other objects at some time t, for which we should not also occupy at that time t. The usefulness of the Dynamic Occupancy Grid is that all agents within our environment can be easily represented under the Grid because they all can be easily and efficiently sampled for any (x, y, t), whereas Zones struggled with efficiency and was unable to represent time altogether. 

#### Proposed Planning System:
Our proposed behavior planning and controls subsystem takes in a dynamic Occupancy Grid as the input for our planning system. Our planning system will output a path of waypoints finely spaced by equal time steps to help navigate the car to the correct location.

In order to make this possible, we have outlined some approaches/options for planning and controls. This document outlines our proposed methods and how they fit together to form a cohesive subsystem. It describes what our system is not responsible for as well. Finally, it compares our proposed methods to Navigator's current approach and to other popular methods in the literature.

## Proposed structure

![BPC_Flow](/navigator/assets/res/BPC_flow.png)


### Current-State Information Required:

Position (x,y,z)
Linear velocities (vx, vy, vz) 

Orientation (Theta_x, Theya_y, Theta_z)
angular velocities (wx, wy, wz) 
(Egan: take a brief look into use of quaternions for modeling orientation and its derivatives, as that's what currently is used)

Short-term local coordinate transform history


### Representation of Surrounding Environment:

Source: [1]

![BPC_options](/navigator/assets/res/bpc_surrounding_environment_rep_options.png)


Voroni Diagrams: unsuitable for non-holonomic vehicles (cars)

State lattices: repeating primitive paths which connect possible states for the vehicle [1]

### Dynamic Occupancy Grid:

#### Structure of Grid:
Each frame of the grid will be created around the local coordinate grid of the AV. It will span 40 meters to the left, 40 meters to the right, 40 meters behind, and 80 meters in front of the current position of the AV at time t = 0. Each grid cell will be made of squares spanning 0.2 meters long and wide. Each frame will represent a discrete time t = a/10 where a = (0, 30) such that our dynamic grid includes predictions from times t = (0, 3).

The grid must be timestamped with when it was created so coordinate systems can be properly matched. The map is accessed via M[t, x,y], where higher values of x and y are to the front and right of the vehicle respectively. (The rear leftmost point is index (0,0)). 

#### Information within cell:
Each cell is associated with a probability of cell being occupied by an obstacle (float value). This probability will be in the range [0, 1].

### Conversion to Cost Map:

#### Determination of Cost:
The primary factor associated with cost will be crashes. Trajectories that lead to crashes will be given extremely high costs to deter the planner from choosing them. The Dynamic Occupancy Grid can be converted into a Cost Map. Other factors to associate with cost are: Travel distance, number of merges/turns, good traffic navigation protocols, etc. We would also use the measure of traffic in different lanes to help us determine cost of a given path.

#### Distance From End:
Each path should be assigned a cost value proportional to the maximum distance from its final position of the AV and the next waypoint.

(Egan: I'm confused about this one. The point is that we maximize travel distance in the direction we want to go, and costs are relative to other paths, so consant cost increase don't matter.) (Response – Chitsein: I think I was thinking incorrectly about travel distance's relevance to the cost map – I meant for it to originally represent the travel distance between two points a and b. I realize that the RRT won't be creating paths to get from a point a to b, but will rather find all paths that could be taken by the AV, so I think making Travel Distance represent a cost proportional to the maximum distance like you suggested would be the better design. )

#### Number of merges/turns:
Increasing the cost of a path every time the AV merges or makes a turn to deter from paths that make unneccessary merges or turns. For example, we probably don't want the vehicle to be merging in and out of lanes constantly on the highway to move a little faster at the expense of slowing down others cars it merges in front of and possibly increasing risk of collision.

##### Good traffic navigation protocols:
Increase the cost of a path that do not follow good traffic navigation protocols. For example, imagine there is a line up of cars to turn right into the highway. Based on the cost map before good traffic navigation protocols are applied, the car could choose to turn into the left lane and then try to merge at the front of the line because it would decrease the time taken to get onto the highway. However, this would go against courteous driving practices, so we should teach the AV to wait it's turn in the line.


#### Calculation of Path Costs:
Each path's cost is calculated based on the summation of the costs of each grid cell at time t (C[t, x, y]) plus the other factors associated with cost, including distance from end, number of merges/turns, and good traffic navigation protocols.

Cost for travel from a given to another: Source [4]

We can represent the points into a graph with vertices being possible options given current position and environment. Edges are the transition from a given point (where AV currently is) to next point. We weight  transition to be the following equation: Source [4]

![Cost_math_method](/navigator/assets/res/bpc_cost_map_method.png)


Proposal (Egan, I think this makes sense, I also think that we may need to tweak this conversion when we try to decide algorithms that work best for RRTs - Hansika): We may additionally want a cost function more general than a summation over occupied cells. The specific case I'm thinking of is we want to encode that, where possible, the vehicle should end in drivable area at the prediction horizon- we don't want to attempt to overtake where since it looks good now, but when the horizon rolls forward the vehicle realizes it was impossible halfway through. This cannot be encoded by a cost map alone.  So a more general approach is:
C(P)=∑C_i (P), where Pis the path under consideration, C(P) is the associated cost, and C_i (P) is a component cost function. So grid cost C_g (P)=∑8_t▒∑_{x \〖in X〗_t }▒∑_{y∈Y_t }▒〖C[t,x,y]〗

Tweaks based on the 10/13 meeting: We have two places to inject cost, which is the RRT cost function evaluated on a per-node basis and a function of the leaf nodes. 
	• RRT cost: cost that can be evaluated at each node without knowing where it will end up
		○ Cost map: occupancy grid, road semantics (drivable area)
		○ Safety/Dynamic considerations: physical obtainability, difference between trajectory speed and road speed limit. A function of the state more than a lookup in the cost map.
	• Leaf nodes:
		○ Whether the trajectory ends up in drivable area
		○ Whether the trajectory ends up closer to the goal? This may be possible to push into RRT cost but makes more sense based on endpoints.

RRTs:

Approach 1: 
Algorithm for RRTs with random sampling: [1] 
Input requirements: Configuration Space 
Ouputs: best state to go to next or Xnear


Benefits: 
	1. probabilistically complete
		a. If solution for path problem exists, RRT will find a soln with probability of 1 as running time goes to infinity 
	2. Guarantee kinematic feasibility 
	3. Quick to explore free space

Detractions:
	1. Jerky paths created
	2. Strong dependence on Neares Neighbor metric
	3. Need to do collision checking for every expanded node

![bpc_rrt_algo](/navigator/assets/res/bpc_rrt_algo.png)


Approach 2: Source [3]

RRT* - works towards shortest path
1. Records distance each vertex has traveled relative to parent
2. Closest node to parent vertex can be replaced by node with lower cost in given radius
3. Neighbhors can be changed higher up in the tree if a cheaper path is found. 

Disadvantages:
1. More computationally expensive

Psuedo Code: Source [3]:

![bpc_rrt_algo](/navigator/assets/res/bpc_rrt*_algo.png)


Approach 3:
Instead of using random point, all feasible connections are evaluated and only minimum cost paths are added to the tree. 
 
Given a cost map, we could build off it in the following manner:

1. Cost functions:  c_safety and c_time calculations can be found in source [4]


To expand tree: souce [5]
1. Sample a pposition uniformly at random and then sample two dimenssionaly gaussian distribution centered at the around intial path. 

![bpc_rrt*_algo](/navigator/assets/res/bpc_rrt_cost.png)


Approach 4: CL-RRT

Tree expansion: grows a tree of feasible trajectories originating from the current vehicle state that attempts to reach a specified goal set [2]

![bpc_cl_rrt_algo](/navigator/assets/res/bpc_cl_rrt.png)


Further RRT approaches are described here: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
Once we decide on a cost map approach, we can determine an algorithm to use using the cost map to create edge weights. Using heuristic algorithms such as A-star, greedy, BFS, etc. We can see if building off the avalaible methods (cost-functions, and weight functions) mentioned above into one of these algorithms, will help with making the best decision.

Other choices than RRTs:

Lattice Planners: We don't wnat to use this because we are getting a dynamic grid:

![bpc_rrt_vs_lattice_planners](/navigator/assets/res/bpc_rrt_vs.png)


Source 1: https://www.sciencedirect.com/science/article/pii/S0968090X15003447

Source 2: https://dspace.mit.edu/bitstream/handle/1721.1/65396/Frazzoli-2009-Real-Time%20Motion%20Planning%20With%20Applications%20to%20Autonomous%20Urban%20Driving.pdf?sequence=1&isAllowed=y

Source 3: https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378

Source 4: https://www.scitepress.org/Papers/2012/40334/pdf/index.html

Source 5: https://www.researchgate.net/profile/Michael-Brunner-9/publication/236847575_Hierarchical_Rough_Terrain_Motion_Planning_using_an_Optimal_Sampling-Based_Method/links/0c9605196187ad600f000000/Hierarchical-Rough-Terrain-Motion-Planning-using-an-Optimal-Sampling-Based-Method.pdf

Creating occupancy map: https://www.scitepress.org/Papers/2012/40334/pdf/index.html -> to share with perception team if they want

Control:

Current System (Severely lacking, high potential):
	• PurePursuit
		○ Tracking algorithm enabling the vehicle's steering wheel to smoothly adjust steering angle in relation to the curve of the given trajectory.
		○ ***Possibly can breakdown this algorithm and apply similar smoothing to other aspects of our vehicle's movement, such as velocity, acceleration, breaking.

Input:
Final, Unidisputed Trajectory: Set of discrete points, x(t), in a local coordinate system taken at time t0
Current Vehicle State
Coordinate transform history between t0 and now

Output:
Commands to hardware on vehicle

Planned System:
	• PurePursuit likely kept
		○ Possibly expanded/using similar approaches to smoothing velocity along trajectory
	• Input:
		○ Final, Unidisputed Trajectory: Set of discrete points (waypoints), x(t) or v(t) [Depends on RRT and how we determine cost-effective routes)
	• Output:
		○ Precise commands to vehicle hardware
			▪ Steering
			▪ Pedal
			▪ Break
		○ Communicate with HFE team
PurePursuit
More PurePursuit

