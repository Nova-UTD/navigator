# Localization

Author: Ragib Arnab


The file localization.py contains a prototype of a lidar localization scheme that re-purposes a loop closing algorithm (MapClosures) as a place recognition module for global localization. You can run the code by just running the script through python. 

The code periodically creates a local map (query) and tries to match it to a database of local maps (indices) collected from the SLAM tool. Once a good match has been found it will use the index map's pose and relative transform between the query map and index map to estimate the vehicle's current pose in the map. The code publishes a global map as well as the odometry of the vehicle on the map that can be visualized in RViz.

The code currently localizes everytime a local map is created. Any poses between these localizations are estimated using lidar odometry (KISS-ICP). This may be accurate enough, but for better performance I suggest the following:

- Create shifting window of local maps so it can query the database more often for localization
- Or track the pose on the global map by performing point cloud registration with odometry as initial transform guess (see ICP as example)

## Running

Dependencies (`kiss-icp==1.0.0`, `map_closures==1.0.1`, `open3d`, `g2o-python`, `rosbags`) are installed by the Dockerfile.

1. Build a map database from a bag that contains lidar scans:

   ```bash
   cd /navigator/src/mapping/offline_lidar_slam
   python3 offline_lidar_slam.py <bag_dir> /lidar /navigator/data/slam_results
   python3 view_map.py /navigator/data/slam_results               # 3D global map
   python3 view_map.py /navigator/data/slam_results --trajectory  # 2D trajectory of local map poses
   ```

   This writes `local_maps/*.ply`, `poses.txt` (before pose graph optimization) and `optimized_poses.txt`.

2. Run the localization node (subscribes to `/lidar`):

   ```bash
   python3 /navigator/src/mapping/localization/localization.py --ros-args -p map_db_path:=/navigator/data/slam_results
   ```

   | Parameter | Default | Meaning |
   | --- | --- | --- |
   | `map_db_path` | `/navigator/data/slam_results/` | Output directory of `offline_lidar_slam.py` |
   | `local_map_range` | `60.0` | Metres driven before a query local map is matched. Keep equal to `map_range` in `offline_lidar_slam.py`. |
   | `map_frame` | `map` | Frame of the published pose and global map |
   | `vehicle_frame` | `vehicle` | Child frame of the published pose / TF |
   | `icp_max_correspondence_distance` | `0.02` | Max correspondence distance (m) of the ICP that refines a MapClosures match. Values from 0.25 to 2.0 made no measurable difference in CARLA. |

   Publishes `/vehicle_odom` (`nav_msgs/Odometry`), TF `map_frame -> vehicle_frame`, and the downsampled global map on `/local_map` at 1 Hz.

## Notes

- The pose is in the frame of the SLAM map, whose origin is the lidar pose at the first scan of the mapping bag. It is not yet aligned with the stack's GNSS-based `map` frame, and it is the lidar's pose, not `base_link`'s.
- No pose is published until the first successful match, i.e. after driving at least `local_map_range` metres.
- Matches only succeed when a query map roughly lines up with a database map, and the last partial local map is not saved by `offline_lidar_slam.py`, so the end of a mapped route is not covered. The shifting-window suggestion above would address both.

## CARLA test results

Live test (Town02, CARLA 0.9.13, `carla_interface` lidar on `/lidar`, synchronous mode at 0.2 s, no traffic). The ego vehicle was driven with CARLA's `BasicAgent` around a fixed 1.84 km loop:

1. Mapping drive from one corner of the loop, recorded and run through `offline_lidar_slam.py` (26 local maps, loop closures found).
2. A separate localization drive of the same loop, starting from the opposite corner, with this node running live against that map.

Errors are against CARLA's true vehicle pose, with the SLAM map frame placed at the true pose of the first mapping scan (no fitting).

| | Localization drive (live) | Mapping drive replayed against its own map |
| --- | --- | --- |
| First fix after | 71.5 m (22 s), from an unknown start | 62.3 m |
| Scans with a published pose after the first fix | 2160 / 2161 | 2186 / 2186 |
| Position error RMSE / median / p95 / max | 1.18 / 0.97 / 2.08 / 2.68 m | 1.23 / 0.93 / 2.17 / 4.27 m |
| Cross-track / along-track RMSE | 0.96 / 0.71 m | |

A separate drive localizes as accurately as the data the map was built from, so the remaining ~1 m error comes from distortion in the SLAM map over the 1.8 km loop rather than from the localization. Improving the map (e.g. GNSS priors in the pose graph, scan deskewing) is the next lever. Corrections at each relocalization jump the pose by up to ~3 m. The CARLA lidar has no noise, so expect worse numbers on a real vehicle.

An earlier offline replay of the CARLA bag `/storage/bags/2025/rosbag2_2025_05_14-16_45_40` (349 m drive, map from the same bag, 5 local maps) gave 0.26-0.39 m RMSE after a rigid alignment, at ~63 ms per scan; the shorter map has less room to drift.
