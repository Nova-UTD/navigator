
# \<Node lidar processing node\> 

{: .no_toc }

Maintained by Nova

### Overview

This node responsible for processing LiDAR data. It is assigned to perform various tasks related to LiDAR data, including preprocessing and filtering.

### In:

<lidar_sub><PointCloud2>
Purpose: This subscriber is responsible for receiving raw LiDAR data from the /carla/hero/lidar topic.
Example: lidar_sub subscribes to /carla/hero/lidar and receives PointCloud2 messages.

<semantic_lidar_sub ><PointCloud2>
Purpose: This subscriber handles incoming semantic LiDAR data from the /carla/hero/semantic_lidar topic.
Example: semantic_lidar_sub subscribes to /carla/hero/semantic_lidar and receives PointCloud2 messages.

<clock_sub><Clock>
Purpose: This subscriber is used to keep track of time by receiving Clock messages from the /clock topic,to get the current simulation time.
Example: clock_sub subscribes to /clock and receives Clock messages.


### Out:

<clean_lidar_pub> <PointCloud2>
Purpose: This publisher is responsible for disseminating processed LiDAR data after transformations and filtering.
Example: clean_lidar_pub publishes on the /lidar/fused topic and sends PointCloud2 messages.

<clean_semantic_lidar_pub><PointCloud2>
Purpose: This publisher is used to transmit processed LiDAR data, typically after transformations and filtering.
Example: clean_semantic_lidar_pub publishes on the /lidar_semantic_filtered topic and sends PointCloud2 messages.

<occupancy_grid_pub><OccupancyGrid>
Purpose: This publisher is responsible for broadcasting an OccupancyGrid message on the /cost/occupancy topic.
Example: occupancy_grid_pub publishes an OccupancyGrid message.


### Individual Function 1

blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla blabla bla bla blabla blablabla blabla bla bla blabla blablabla

### Individual Function 2

blabla bla bla blabla blablabla blabla bla bla blabla blablablablabla bla bla blabla blablabla blabla bla bla blabla blablabla blabla bla bla blabla blablabla
