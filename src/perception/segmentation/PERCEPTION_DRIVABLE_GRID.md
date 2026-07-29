# Perception-Based Drivable Area Grid

Real-time drivable area estimation using on-board camera semantic segmentation
and LiDAR ground returns. Operates without HD maps, runs at 10 Hz, and covers
the full 360° around the vehicle.

## Overview

The Navigator stack previously relied on the map manager to determine which road
cells are drivable. That approach requires a pre-built HD map and updates slowly
(< 1 Hz) because it re-projects the full map graph each cycle. This system
replaces that as the primary drivable area source, falling back to the HD map
only when perception confidence is low.

Two new nodes work together:

| Node | Output topic | Role |
|------|-------------|------|
| `PerceptionDrivableGridNode` | `/grid/drivable/segmented` | Raw perception evidence grid (10 Hz) |
| `HybridDrivableGridNode` (updated) | `/grid/drivable` | Merged output consumed by planning (10 Hz) |

The final `/grid/drivable` topic — which feeds `grid_summation_node` →
`/grid/steering_cost` — is a three-layer fusion:

```
Perception (primary, 10 Hz)
    ↓  uncertain cells fall back to
HD Map (fallback, ~0.7 Hz)
    ↓  obstacle detections veto both
Occupancy Grid (/grid/occupancy/current, 5 Hz)
    ↓
/grid/drivable  →  grid_summation_node  →  /grid/steering_cost
```

No changes are required to any planning, cost, or control node.

---

## PerceptionDrivableGridNode

**File:** `src/perception/segmentation/segmentation/perception_drivable_grid_node.py`  
**Topic:** `/grid/drivable/segmented`  
**Rate:** 10 Hz  
**Frame:** `base_link`  
**Grid:** 300 × 300 cells, 0.2 m/cell, origin (−20 m, −30 m)

### Pipeline

Each 100 ms publish cycle runs 11 steps:

```
Step  1  Camera BEV projection
         PSPNet semantic images → ground-plane ray-cast LUT → per-cell road scores

Step  2  LiDAR ground/obstacle evidence
         Count-based: ground hits (z < 0.35 m) vs obstacle hits (z > 1.20 m) per cell.
         LiDAR-only cells with ≥ 2 ground hits and 0 obstacle hits → 0.85 evidence.

Step  3  Temporal accumulation + pose compensation
         Alpha-blend (α = 0.65) new frame onto rolling evidence grid.
         Odometry-based warpAffine shifts grid as vehicle moves.
         Dead zone: skip warp when |Δx| < 2 cm, |Δy| < 2 cm, |Δyaw| < 0.008 rad.

Step  4  Vehicle footprint prior
         4 m × 6 m rectangle centred at base_link always ≥ 0.90 evidence.
         The vehicle is always sitting on drivable surface.

Step  5  Path history prior
         Last 50 odometry positions (≈ 12 s / 60 m) stored as (x, y) world coords.
         Each point stamps a 1.6 m radius patch at ≥ 0.82 evidence.
         Memory cost: 800 bytes (deque of float pairs, no full-grid storage).

Step  6  Gaussian boundary smoothing
         σ = 1.2 cells; removes hard jagged edges from projection aliasing.

Step  7  Road dilation into blind spots
         Morphological dilation of confirmed road (ev > 0.65) into uncertain cells
         (0.36–0.64). Standard kernel: 25 cells (5 m radius).
         At intersections (road cells > 1500): 51-cell kernel (10 m radius) to
         bridge across the gap to perpendicular cross-roads.

Step  8  LiDAR ground fill
         Uncertain cells (ev 0.38–0.62) that have ≥ 2 LiDAR ground returns
         boosted to 0.72. Fills lateral camera blind spots and intersection
         branches confirmed flat by LiDAR.

Step  9  Connected road fill (flood fill)
         scipy.ndimage.label identifies all non-obstacle cells (ev > 0.22)
         reachable from the vehicle footprint. Any uncertain cell in that
         connected component is boosted to 0.70 evidence.
         Fills intersection branches topologically connected to the vehicle's
         road regardless of geometric distance or direction.

Step 10  Road corridor fill
         For each forward grid column (x direction), finds the span of confirmed
         road cells (first row to last row) and fills uncertain cells within that
         span to 0.75 evidence. Cross-streets run laterally (column-wise span of
         1–2 cells) and are naturally skipped by a min-4-cells-per-column guard.

Step 11  Morphological cleanup
         3 × 3 open (remove isolated noise) + 7 × 7 close (fill small gaps).
```

Evidence is published as an `OccupancyGrid` with the standard ROS2 convention:
`occupancy = round((1 − evidence) × 100)` so 0 = fully drivable, 100 = obstacle.

### Camera Configuration

Extrinsics are hardcoded from `carla_objects.json`. Camera TF frames do not
exist in the TF tree at runtime — any TF-based lookup produces all-gray output.

| Camera | Semantic topic | Position (x, y, z) m | Yaw |
|--------|---------------|----------------------|-----|
| Front  | `/semantic/front` | (0.70, −0.15, 1.88) | 0° |
| Right  | `/semantic/right` | (0.70, −0.15, 1.88) | −70° |
| Left   | `/semantic/left`  | (0.70,  0.15, 1.88) | +70° |
| Back   | `/semantic/back`  | (−1.50, 0.00, 1.88) | 180° |

Intrinsics: fx = fy = 571.12, cx = 400, cy = 300 (FOV 70°, 800 × 600).

Back camera requires `rgb_back` to be present in `carla_objects.json` and
CARLA to be restarted to spawn the sensor. The node runs with 3 cameras if
`/semantic/back` is not published.

### LiDAR Configuration

Subscribes to `/lidar/filtered`. Ground hits are points with
−0.30 m ≤ z < 0.35 m; obstacle hits are z ≥ 1.20 m. Points between
these bands (0.35–1.20 m) are ignored — this band covers curbs, low
vegetation, and occasional LiDAR noise that would otherwise generate
false obstacles within the road.

---

## HybridDrivableGridNode (updated)

**File:** `src/perception/segmentation/segmentation/hybrid_drivable_grid_node.py`  
**Topic:** `/grid/drivable`  
**Rate:** 10 Hz (previously 5 Hz)

### Fusion Logic

```
For each grid cell:

  if perception occupancy < 30  (evidence > 0.70 — high confidence road):
      output = 0  (road)
      # Overrides HD map including legal lane boundaries.
      # Perception has seen this surface clearly.

  elif perception occupancy >= 30  (uncertain):
      if HD map == 0  (mapped road):
          output = 0  (road — HD map fallback)
      elif HD map is intermediate:
          output = hdmap value
      else  (HD map boundary or unknown):
          output = 100  (obstacle — safe default)

  if no perception data (startup / topic gap):
      output = HD map  (full fallback, identical to previous behaviour)

  # Final obstacle veto (applied after above):
  if output == 0 and occupancy_grid >= 80:
      output = 100  (real-time obstacle blocks even confirmed road)
  elif output == 0 and occupancy_grid > 0:
      output = occupancy value  (hazard nuance)
```

Key properties of this design:

- **Perception drives decisions at 10 Hz** on all cells where it has ≥ 70%
  confidence. The HD map does not throttle these cells to its own update rate.
- **HD map is a safety net, not the authority.** It handles cells where
  perception is uncertain — for example, road cells beyond camera range or
  where all cameras happen to see sky.
- **Speckle-immune.** Uncertain perception cells (evidence 0.30–0.70) do not
  open or close road cells on their own; they fall back to HD map. Only
  high-confidence cells affect the output.
- **Graceful degradation.** If `/grid/drivable/segmented` is not yet published
  (startup lag, node crash), the node falls back silently to the previous
  HD-map-only behaviour and logs a throttled warning.
- **No downstream changes.** `/grid/drivable` topic name, message type
  (`nav_msgs/OccupancyGrid`), grid dimensions (300 × 300), resolution
  (0.2 m/cell), and frame (`base_link`) are identical to before.

---

## Integration with the Occupancy Grid PR

This PR includes both the drivable area changes and the updated
`HybridPerceptionGridNode` (`/grid/occupancy/current`). The two are independent
at the topic level:

- `HybridDrivableGridNode` subscribes to `/grid/occupancy/current` for the
  obstacle veto step. If that topic is not yet available (occupancy grid PR
  not merged), the veto step is skipped and the node functions correctly using
  only perception + HD map. No crash, no hang.
- `/grid/drivable/segmented` and `/grid/occupancy/current` are published by
  separate nodes on separate timers and never block each other.

The system degrades gracefully through any combination of missing inputs:

| Available inputs | Behaviour |
|-----------------|-----------|
| Perception + HD map + occupancy | Full three-layer fusion |
| Perception + HD map | Road opens from perception; no real-time obstacle veto |
| Perception only | Road opens from perception; uncertain cells default to obstacle |
| HD map only (perception not started) | Identical to previous behaviour |

---

## RViz Visualisation

Three `Map` displays are available in `data/navigator_default.rviz`:

| Display name | Topic | What it shows |
|-------------|-------|---------------|
| `percpetionentry` | `/grid/drivable/segmented` | Raw perception evidence only |
| (existing) | `/grid/drivable` | Final merged output (what planning uses) |
| (existing) | `/grid/drivable/hdmap` | HD map only (MapManager output) |

---

## Starting the Node Manually

```bash
docker exec -d navigator_carla_snandyala bash -c \
  "export ROS_DOMAIN_ID=57 && \
   source /opt/ros/humble/setup.bash && \
   source /navigator/install/setup.bash && \
   ros2 run segmentation perception_drivable_grid_node >> /tmp/perc_drivable.log 2>&1"
```

Expected startup output:

```
[front] LUT ready — 216096/480000 pixels (45.0%)
[right] LUT ready — 211987/480000 pixels (44.2%)
[left]  LUT ready — 211832/480000 pixels (44.1%)
[back]  LUT ready — 193600/480000 pixels (40.3%)
PerceptionDrivableGridNode ready — /grid/drivable/segmented
```

LUTs are built at `__init__` time (< 200 ms). The node publishes immediately
from the first frame without any warm-up period.

---

## Version History

All tags are on `feat/hybrid-perception-occupancy-grid`:

| Tag | What changed |
|-----|-------------|
| `perception-drivable-v1` | Initial node — hardcoded extrinsics, ground-plane LUT |
| `perception-drivable-v2` | Back camera, vehicle footprint prior, path history, dilation |
| `perception-drivable-v3` | Adaptive dilation, removed intersection corridor suppression, jitter dead zone |
| `perception-drivable-v4` | Count-based LiDAR ground evidence; fills lateral blind spots |
| `perception-drivable-v5` | Connected road flood fill, column-wise corridor fill, wider morpho cleanup |
