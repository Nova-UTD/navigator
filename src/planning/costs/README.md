# costs

Processes, combines, and publishes cost maps (grids) for the planner.

## Nodes

### `pedestrian_costmap_node`

Converts pedestrian-intent detections into a metric cost grid. **Shadow mode:**
publishes `/grid/pedestrian` for observation in RViz; it is **not** registered
in `grid_summation_node`, so it does not affect the planned path.

**Subscribes**
- `/pedestrians` (`navigator_msgs/PedestrianInfoDetections`) — intent detections,
  including metric `pos_x`/`pos_y` in `base_link`.
- `/clock` (`rosgraph_msgs/Clock`) — for grid stamping.

**Publishes**
- `/grid/pedestrian` (`nav_msgs/OccupancyGrid`) — 151x151 @ 0.4 m/cell,
  `base_link`, origin (-20.0, -30.0). In-road pedestrians paint cost ~100;
  edge-of-road pedestrians paint a graded (lower) cost. Empty / no detections
  yet -> all-zero grid.

**Parameters** (`param/pedestrian_costmap_params.yaml`)
- `publish_rate_hz` (default `15.0`) — publish rate.
- `d_max_m` (default `10.0`) — distance at which a pedestrian stops contributing.
- `inflation_radius_m` (default `0.8`) — painted disk radius (footprint + pad).

The monocular projection that fills `pos_x`/`pos_y` runs in the producer
(`pedestrian_intent_to_enter_road`); its `cam_offset_x`/`cam_offset_y` params
hold the camera->base_link offset (default 0.0).

**Run standalone**

    ros2 launch costs pedestrian_costmap.launch.py
