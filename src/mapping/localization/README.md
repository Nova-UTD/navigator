# Localization

Author: Ragib Arnab


The file localization.py contains a prototype of a lidar localization scheme that re-purposes a loop closing algorithm (MapClosures) as a place recognition module for global localization. You can run the code by just running the script through python. 

The code periodically creates a local map (query) and tries to match it to a database of local maps (indices) collected from the SLAM tool. Once a good match has been found it will use the index map's pose and relative transform between the query map and index map to estimate the vehicle's current pose in the map. The code publishes a global map as well as the odometry of the vehicle on the map that can be visualized in RViz.

The code currently localizes everytime a local map is created. Any poses between these localizations are estimated using lidar odometry (KISS-ICP). This may be accurate enough, but for better performance I suggest the following:

- Create shifting window of local maps so it can query the database more often for localization
- Or track the pose on the global map by performing point cloud registration with odometry as initial transform guess (see ICP as example)
