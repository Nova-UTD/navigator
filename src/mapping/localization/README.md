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

   Publishes `/vehicle_odom` (`nav_msgs/Odometry`), TF `map_frame -> vehicle_frame`, and the downsampled global map on `/local_map` at 1 Hz.

## Notes

- The pose is in the frame of the SLAM map, whose origin is the lidar pose at the first scan of the mapping bag. It is not yet aligned with the stack's GNSS-based `map` frame, and it is the lidar's pose, not `base_link`'s.
- No pose is published until the first successful match, i.e. after driving at least `local_map_range` metres.
- Offline replay on the CARLA bag `/storage/bags/2025/rosbag2_2025_05_14-16_45_40` (349 m drive, map built from the same bag, 5 local maps): started at 0 m, 63 m and 125 m into the drive, the node localized within 1-2 query maps and then tracked `/carla/hero/gnss_gt` with 0.26-0.39 m RMSE (max 1.2 m) after a rigid alignment, at ~63 ms per scan. Started 190 m or more into the drive it did not localize: matches only succeed when a query map roughly lines up with a database map, and the last partial local map is not saved by `offline_lidar_slam.py`, so the end of the route is not covered. The shifting-window suggestion above would address this.
