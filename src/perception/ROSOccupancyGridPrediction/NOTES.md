# General
Supports both TensorFlow and PyTorch. Since we already have PyTorch installed, let's use that.

The prediction result is a 20 x 2 x 128 x 128 array:
- from t = [0,19]
- Both occupied and unoccupied?
- 128 x 128 cells (0.33m resolution, ~43 meters)

# Messages
## DDLPointType
Typedef to `lidar_msgs::PointXYZIDBS`, where each point has xyz, intensity, device ID, beam ID, and score (?).

## Masses
Appears to be a custom Occupancy Grid message, with separate arrays for occupied and free probabilities. "It contains masses for DST [Dempster-Shafer
Theory] occupancy grid calculation and prediction."

## Prediction
Array of `nav_msgs/OccupancyGrid`

# Components
## AggregatePoints
The Ford AV dataset includes four LiDAR sensors named Red, Yellow, Green, and Blue. This node simply adds them into a single point cloud and publishes it to `/agg_points`

## FrameBroadcaster
Appears to send a transform from `body -> global_pcl_frame` that rotates 90 degrees about the z axis.

## InferenceTorch
### `onInit()`
Publishes to both `/prediction` (single OccupancyGrid) and `/prediction_all` (Prediction msg, just an array of OccupancyGrids).

Subscribes to `/masses`, which is a DST-formatted occupancy grid.

1. Loads model from `models/`
2. Initializes subs, pubs, and timer

### `timerCallback`
Every 1 ms (at best):
1. `createTensorFromFrames` ("Processing masses")
2. `Infer` ("Making predictions")
3. `Publish` ("Publishing the prediction")

### `masses_callback`
Collects `masses` data (DT-style occupancy grids) into a vector of length 5.

## MrfGroundSeg
Markov Random Field ground segmentation. Subscribes to `/agg_points` and publishes the filtered cloud to `/mrf_filtered_points`

Published result preserves all "raw" fields for each point, including device ID, beam ID, and score.

## OccupancyGridGeneration
Publishes a standard OccupancyGrid message with 128x128 cells. Resolution is ~0.33 meters, so grid is ~42x42 meters, centered on the car.

Used to rely on upstream vehicle localization to determine the car's change in position at each update. *Now assumes that the change is zero-- e.g. that the car is stationary.*

Call stack:
1. `cloud_cb` (on each message received from ground seg)
   1. `create_DST_grid`
      1. `add_points_to_the_DST`
      2. `add_free_spaces_to_the_DST`
      3. `add_ego_vehicle_to_the_DST`

Functions operate on global variables, so they don't take arguments.

## Visualize
Subscribes to `/prediction_all` (array of OccupancyGrids).

Loops through each prediction layer from t = [4, 20), publishes it to `/visualization`, and moves to the next layer after 0.1 seconds.