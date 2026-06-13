# Pedestrian-Intent Cost Layer (shadow mode) — Design

- **Date:** 2026-06-13
- **Author:** Shrey Joshi
- **Status:** Approved design, pre-implementation
- **Branch:** `feature_pedestrian_cost_layer`

## 1. Motivation

The perception stack already runs a full pedestrian-intent pipeline
(`pedestrian_skeleton` → `pedestrian_intent_to_enter_road`) that detects
pedestrians who intend to enter the road and publishes them on `/pedestrians`
(`navigator_msgs/PedestrianInfoDetections`). **No node downstream consumes this
topic** — the data is computed and discarded.

This feature activates that pipeline by converting intent detections into a
metric cost grid that fits Navigator's existing layered-costmap architecture.
The planner already routes around / slows for cost grids; we only need to
produce one.

## 2. Scope

**In scope (this PR):**
1. Extend `navigator_msgs/PedestrianInfo.msg` with metric position fields.
2. Update `pedestrian_intent_to_enter_road` to populate those fields.
3. New `pedestrian_costmap_node` (in the `costs` package) that subscribes to
   `/pedestrians`, paints a graded cost grid, and publishes `/grid/pedestrian`.
4. Unit tests for the pure logic (cost mapping, projection, painting).

**Shadow mode:** `/grid/pedestrian` is published and observable in RViz, but is
**NOT** registered in `grid_summation_node`. The planned path is therefore
unaffected by this PR.

**Out of scope (documented for the follow-up):**
- Registering `/grid/pedestrian` in `grid_summation_node` (makes it live).
- Real camera extrinsics (a static offset param is provided as the hook).

## 3. Data flow

```
/cameras/camera0 ──┐
/segmentation_mask ─┴→ pedestrian_intent_to_enter_road
                         │ publishes /pedestrians (PedestrianInfoDetections)
                         │   now incl. metric pos_x, pos_y
                         ▼
              [NEW] pedestrian_costmap_node
                         │ publishes /grid/pedestrian (OccupancyGrid, base_link)
                         ▼
              grid_summation_node   ← NOT wired this PR (shadow)
```

## 4. Message extension

Append to `src/msg/navigator_msgs/msg/PedestrianInfo.msg` (back-compatible —
new fields added at the end, existing fields untouched):

```
float32 pos_x   # metric forward distance in base_link (m), +x = ahead
float32 pos_y   # metric lateral offset in base_link (m), +y = left
```

Existing fields retained: `x, y, width, height` (camera pixels) and `distance`
(meters to road edge).

## 5. Producer changes — projection math

In `pedestrian_intent_to_enter_road.py`, when building each `PedestrianInfo`,
compute metric position from the monocular pinhole model the node already uses
internally:

```python
depth   = 470 * 1.75 / bbox_pixel_height            # forward range (m)
cx      = image_width / 2.0
# 1.0913 = tan(HFOV/2); matches the constant already used in this node
lateral = -((center_x - cx) / image_width) * (2.0 * depth * 1.0913)

pos_x = depth   + cam_offset_x
pos_y = lateral + cam_offset_y
```

Sign convention: pedestrian right of image center (`center_x > cx`) → negative
`pos_y` (vehicle's right). `cam_offset_x/y` are ROS params (default 0.0) that
let a real camera→base_link extrinsic be plugged in later without code change.

**Assumption / known limitation:** v1 treats the camera as mounted at the
`base_link` origin pointing straight forward. This is the primary accuracy
caveat; acceptable because the layer is shadow-only and the offset params make
the upgrade path mechanical.

## 6. New node — `pedestrian_costmap_node`

Lives in the `costs` package, mirrors `lane_controlled_costmap_node`.

**Grid contract (must match the other layers / `grid_summation_node`):**
- `frame_id = base_link`
- size `151 × 151` cells, resolution `0.4 m/cell`
- origin `(-20.0, -30.0)` in base_link
- values clipped to `0–100` (`int8`); `≥ 90` is the planner obstacle threshold
- stamped from `/clock`
- timer-driven publish (param `publish_rate_hz`, default `15.0`)
- keeps only the newest `/pedestrians` message

**Per detection:**
1. Map `(pos_x, pos_y)` → grid `(row, col)` via origin/resolution; skip if
   off-grid.
2. Graded cost by proximity to the road:
   `cost = clip(100 * (1 - distance / D_MAX), 0, 100)`  (`D_MAX` param, default
   `5.0` m). `distance = 0` (in road) → `100`; `distance ≥ D_MAX` → `0`.
3. Paint a filled inflated disk (radius = pedestrian footprint + safety pad,
   param `inflation_radius_m`, default `0.8` m) centered on the cell, combining
   with the running grid via `np.maximum`.

**Parameters** (in `param/pedestrian_costmap_params.yaml`):
`publish_rate_hz`, `d_max_m`, `inflation_radius_m`, `cam_offset_x`,
`cam_offset_y`.

## 7. Behavior when wired live (follow-up, specified now)

The graded "avoid + slow" behavior requires `grid_summation_node` to route
`/grid/pedestrian` into **both** outputs (one-line additions to its routing
branch in `createCostMap`):
- into `steering_cost` (via `np.maximum`) → in-road pedestrians (cost ≈ 100 ≥
  threshold 90) become padded obstacles the Dijkstra planner routes around.
- into `speed_cost` (via `np.maximum`) → edge-of-road pedestrians (moderate
  cost) reduce target speed without altering the geometric path.

Plus a subscription, newest-message callback, `SCALE` constant, and a tuple
entry in the `grids` list — mirroring the existing `lane_control` wiring. This
PR does **not** make these changes.

## 8. Error handling

- No `/pedestrians` received yet, or an empty `pedestrians` array → publish an
  all-zero grid (consistent with other layers' "nothing to add" state).
- A detection that projects off-grid or has invalid geometry → skip that
  pedestrian, log at debug, continue publishing.
- Grids stamped from `/clock` so existing staleness handling behaves when the
  layer is later wired into `grid_summation_node`.

## 9. Testing

Pure-function unit tests (no ROS spin), in the `navigator_lane_change` test
style:
- `distance → cost` mapping at boundaries: `0`, mid-range, `D_MAX`, beyond.
- `(pos_x, pos_y) → (row, col)` projection: center, off-grid clamping, sign
  conventions on all four quadrants.
- inflated-disk painting and `np.maximum` overlap of two nearby pedestrians.
- producer projection: `center_x` left vs right of image center → correct
  `pos_y` sign and magnitude.

## 10. Risks & limitations

- **Monocular depth** (`depth = f·H / h_px`) is sensitive to bbox height noise;
  range is approximate. Acceptable for a shadow layer and inflation absorbs some
  error.
- **Camera extrinsics** approximated as identity in v1 (see §5).
- The producer only emits pedestrians *facing* the road (existing filter), so
  the layer reflects intent detections, not all pedestrians — by design.
