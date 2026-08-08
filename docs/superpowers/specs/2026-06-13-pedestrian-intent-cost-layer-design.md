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

**Split into two PRs** (downstream check confirmed `PedestrianInfo` has no
consumers — the bugfix changes no live behavior, so it stands alone):

- **PR 1 — bugfix (prerequisite):** the §5.1 variable-shadowing fix plus its §10
  regression test. Self-contained; mergeable independently.
- **PR 2 — feature:** items 1, 3–5 below; rebases on PR 1.

**In scope (PR 2 — feature):**
1. Extend `navigator_msgs/PedestrianInfo.msg` with metric position fields.
2. Update `pedestrian_intent_to_enter_road` to populate those fields. (The
   variable-shadowing bug it depended on is fixed separately in PR 1 — see §5.1.)
3. New `pedestrian_costmap_node` (in the `costs` package), split into a pure-logic
   module (`pedestrian_costmap.py`) and a thin ROS node, that subscribes to
   `/pedestrians`, paints a graded cost grid, and publishes `/grid/pedestrian`.
4. Package wiring so the node actually runs: `setup.py` entry point, a launch
   file, `package.xml` dependencies, and a parameter file (see §8).
5. Unit tests for the pure logic (cost mapping, projection, painting).

**Shadow mode:** `/grid/pedestrian` is published and observable in RViz, but is
**NOT** registered in `grid_summation_node`. The planned path is therefore
unaffected by this PR.

**Out of scope (deferred to Phase 3, PR 3 — fully specified in §11):**
- Registering `/grid/pedestrian` in `grid_summation_node` and adding a dedicated
  routing branch that feeds **both** `steering_cost` and `speed_cost` (makes it
  live — behavior in §7, wiring in §11.1; more than a one-line change).
- Real camera extrinsics / constant re-validation (a static offset param is the
  hook — §11.2).
- Parameter tuning once the layer affects motion (§11.3).
- Documentation updates to `docs/` (tracked in §12).

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

## 5. Producer changes

### 5.1 Pre-existing bug fix (variable shadowing) — **PR 1, shipped separately**

`detect_pedestrians()` in `pedestrian_intent_to_enter_road.py` has a latent bug
that must be fixed before the projection math can work. It ships as its own PR
(PR 1) since it has no downstream consumers and changes no live behavior:

- L75 binds the bounding box: `center_x, center_y, width, height = xywh`
- L98 then **clobbers** them: `height, width, channels = self.image.shape`
- L110-111 publish `pedestrian_object.width/height = float(width/height)` — so
  every pedestrian's `width`/`height` fields currently carry the **full image
  dimensions**, not the bbox. This is wrong today, independent of this feature.

**Fix:** rename the bbox values immediately after L75 to `bbox_w`, `bbox_h`
(and use a distinct name such as `img_h, img_w` for the image shape at L98).
Assign the real bbox dimensions to the message fields. The bbox pixel height for
the depth estimate comes from `xyxy` (`bbox_pixel_height = BRy - TLy`), which is
already in scope. This is a small, well-contained bugfix; it ships as PR 1 ahead
of the feature so the corrected `width`/`height` fields land independently.

### 5.2 Projection math

When building each `PedestrianInfo`, compute metric position from the monocular
pinhole model the node already uses internally:

```python
bbox_pixel_height = BRy - TLy                        # from xyxy (in scope)
depth   = 470 * 1.75 / bbox_pixel_height             # forward range (m)
img_h, img_w = self.image.shape[:2]
cx      = img_w / 2.0
# 1.0913 = tan(HFOV/2); matches the constant already used in this node
lateral = -((center_x - cx) / img_w) * (2.0 * depth * 1.0913)

pos_x = depth   + cam_offset_x
pos_y = lateral + cam_offset_y
```

Sign convention: pedestrian right of image center (`center_x > cx`) → negative
`pos_y` (vehicle's right). `cam_offset_x/y` are ROS params (default 0.0) that
let a real camera→base_link extrinsic be plugged in later without code change.
Use `self.image.shape` for image width consistently (the implied capture
resolution is ~672×376 / ZED 720p).

**Header frame note:** the producer stamps `PedestrianInfoDetections.header` with
`frame_id = "camera_frame"`. We leave that as-is; `pedestrian_costmap_node`
consumes only `pos_x`/`pos_y`, which are defined in `base_link`, and ignores the
header frame. The new fields' frame is documented in the `.msg` comment.

**Assumption / known limitation:** v1 treats the camera as mounted at the
`base_link` origin pointing straight forward. This is the primary accuracy
caveat; acceptable because the layer is shadow-only and the offset params make
the upgrade path mechanical.

## 6. New node — `pedestrian_costmap_node`

Lives in the `costs` package, mirrors `lane_controlled_costmap_node` (the
post-fix, modern template — **not** `route_costmap_node`, whose callback naming
is inconsistent). Split into two files so the logic is testable without ROS:

### 6.1 Module structure

**`costs/pedestrian_costmap.py`** — pure functions, no ROS imports, deterministic:

```python
def distance_to_cost(distance_m: float, d_max_m: float) -> int
def pose_to_grid_coords(pos_x_m, pos_y_m, origin_x_m, origin_y_m,
                        resolution_m, grid_size) -> Optional[Tuple[int, int]]
def paint_disk(grid: np.ndarray, row: int, col: int,
               radius_m: float, resolution_m: float, cost: int) -> None
```

**`costs/pedestrian_costmap_node.py`** — thin `Node` subclass: parameters,
subscription, timer, message construction. Delegates all math to the pure module
(mirrors how `lane_change_node` instantiates `SafetyChecker` etc.).

### 6.2 Grid contract (must match the other layers / `grid_summation_node`)
- `frame_id = base_link`
- size `151 × 151` cells (square), resolution `0.4 m/cell`
- origin: `info.origin.position.x = -20.0` (longitudinal), `.y = -30.0`
  (lateral) — **in this exact order** (the #494 origin-swap regression)
- `info.width = cols`, `info.height = rows` — **never transposed** (the #493
  x/y-flip regression); safe here because the grid is square, but stated so the
  code cannot drift
- `data = arr.flatten().tolist()` in numpy row-major order
- values clipped to `0–100`; `≥ 90` is the planner obstacle threshold
- stamped from `/clock`
- timer-driven publish (param `publish_rate_hz`, default `15.0`)
- keeps only the newest `/pedestrians` message

### 6.3 Painting (per detection)
1. `cost = distance_to_cost(ped.distance, d_max_m)` —
   `clip(100 * (1 - distance / D_MAX), 0, 100)`. `distance = 0` (in road) →
   `100`; `distance ≥ D_MAX` → `0`. (`distance` sets cost *magnitude*; it is the
   gap-to-road-edge field and is deliberately distinct from `pos_y`, which only
   sets *where* on the grid the cost is painted.)
2. `pose_to_grid_coords(ped.pos_x, ped.pos_y, ...)` → `(row, col)`; skip if
   off-grid.
3. `paint_disk(...)` — filled inflated disk (radius = pedestrian footprint +
   safety pad, param `inflation_radius_m`, default `0.8` m), combined into the
   running grid via `np.maximum`.

Empty / no `/pedestrians` yet → publish an all-zero grid.

### 6.4 Implementation conventions (from exemplar code + Nova docs)
- numpy dtype progression: **work in `int16`, publish as `int8`**, serialize via
  `.flatten().tolist()`.
- callbacks named `_cb_<topic>`, timer `_on_timer`.
- clock cached as `self._sim_clock = Clock()` with the comment noting it avoids
  shadowing `rclpy.Node._clock`.
- module docstring in `lane_controlled` style (file header + grid contract);
  Doxygen docblocks and type hints on all functions; a package `README` entry.

**Parameters** (in `costs/param/pedestrian_costmap_params.yaml`, each documented
with units/defaults): `publish_rate_hz`, `d_max_m`, `inflation_radius_m`,
`cam_offset_x`, `cam_offset_y`.

## 7. Behavior when wired live (follow-up, specified now)

The graded "avoid + slow" behavior requires `grid_summation_node` to route
`/grid/pedestrian` into **both** `steering_cost` and `speed_cost`:
- into `steering_cost` (via `np.maximum`) → in-road pedestrians (cost ≈ 100 ≥
  threshold 90) become padded obstacles the Dijkstra planner routes around.
- into `speed_cost` (via `np.maximum`) → edge-of-road pedestrians (moderate
  cost) reduce target speed without altering the geometric path.

**Note (corrected during stress test):** this is *not* a one-line change.
`createCostMap` currently routes each grid to a single bucket by name (drivable
→ steering, junction → speed, everything else → steering via the `else` branch).
Feeding one grid to *both* buckets needs a new dedicated branch for
`'pedestrian'` that applies `np.maximum` to `steering_cost` **and**
`speed_cost`, in addition to the usual subscription, newest-message callback,
`SCALE` constant, and `grids`-list entry. This PR does **not** make these
changes; the shadow node simply publishes one graded grid that the follow-up
routes to both outputs.

## 8. Package wiring (so the shadow node actually runs)

The chosen template, `lane_controlled_costmap_node`, is itself **not registered
in `costs/setup.py` and not in any launch file** — it is effectively dead code.
We must avoid that and fully wire the new node:

1. **`costs/setup.py`** — add entry point:
   `'pedestrian_costmap_node = costs.pedestrian_costmap_node:main'`. Do not
   propagate the existing `maintainer='main'` placeholder to new content.
2. **Launch** — add a `costs/launch/pedestrian_costmap.launch.py` that starts the
   node with the param file, and (optionally) reference it from
   `launches/launch_node_definitions.py` so it comes up with the stack. Shadow
   mode means it runs and publishes; it is simply absent from
   `grid_summation_node`.
3. **`costs/package.xml`** — add missing runtime deps used by the node:
   `rclpy`, `navigator_msgs`, `rosgraph_msgs` (for `Clock`); `nav_msgs` is
   already present. Add test deps to match `navigator_lane_change`:
   `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.
   (Note: `route_costmap_node` already imports `navigator_msgs` without
   declaring it — adding the dep here also closes that latent gap.)
4. **`costs/param/pedestrian_costmap_params.yaml`** — created with all five
   parameters documented (units + defaults), under a
   `pedestrian_costmap_node: ros__parameters:` block.
5. **`README`** — add a section documenting the node (purpose, I/O topics,
   parameters), per Nova's per-package documentation requirement.

## 9. Error handling

- No `/pedestrians` received yet, or an empty `pedestrians` array → publish an
  all-zero grid (consistent with other layers' "nothing to add" state).
- A detection that projects off-grid or has invalid geometry → skip that
  pedestrian, log at debug, continue publishing.
- Grids stamped from `/clock` so existing staleness handling behaves when the
  layer is later wired into `grid_summation_node`.

## 10. Testing

Pure-function unit tests against `costs/pedestrian_costmap.py` (no ROS spin),
`pytest` style with factory helpers and bare `assert`, matching
`navigator_lane_change/test/`:
- `distance_to_cost` at boundaries: `0` → 100, mid-range, `D_MAX` → 0, beyond.
- `pose_to_grid_coords`: ego `(0,0)` → `(row=75, col=50)`; off-grid → `None`;
  sign conventions on all four quadrants.
- `paint_disk`: disk is centered and combines via `np.maximum` for two
  overlapping pedestrians; dtype stays `int16` in-grid.
- producer projection (in the perception package's test or a focused unit test):
  `center_x` left vs right of image center → correct `pos_y` sign and magnitude;
  and a regression assertion that the message `width`/`height` now carry **bbox**
  dimensions, not full-image dimensions (guards the §5.1 fix).

## 11. Phase 3 — Live wiring (follow-up PR)

PRs 1–2 leave `/grid/pedestrian` published but unconsumed (shadow). This phase
flips it live and is where the deferred accuracy decisions (#3 camera geometry,
#4 parameter defaults) finally matter — because once the grid feeds the planner,
wrong geometry or tuning changes how the vehicle steers and slows. Treat this as
its own PR (PR 3), gated on real-world/sim validation.

### 11.1 Route the grid into the planner (the §7 work)
Implement the §7 routing in `grid_summation_node`: a dedicated `'pedestrian'`
branch in `createCostMap` that applies `np.maximum` into **both** `steering_cost`
(in-road peds become obstacles the Dijkstra planner routes around) **and**
`speed_cost` (edge-of-road peds slow the vehicle without bending the path), plus
the subscription, newest-message callback, `SCALE` constant, and `grids`-list
entry. Not a one-line change (see §7).

### 11.2 Resolve #3 — real camera geometry (now behavior-affecting)
The identity-extrinsics and pinhole constants were acceptable while shadow-only;
live, a mislocated pedestrian paints cost in the wrong cell and can misroute the
planner. Before wiring:
- Set `cam_offset_x` / `cam_offset_y` from the **measured** camera→`base_link`
  mount (the params exist precisely so this needs no code change).
- Re-validate the in-node constants against the actual rig: focal `470` and
  `1.0913 = tan(HFOV/2)` are tied to the ~672×376 capture resolution (§5.2). If
  the deployed resolution or lens differs, correct them — they drive both
  `distance` (cost magnitude) and lateral `pos_y` (paint location).
- If a straight-forward, origin-mounted approximation proves too coarse in
  validation, add camera pitch/yaw to the projection rather than offset-only.

### 11.3 Resolve #4 — tune parameter defaults (now behavior-affecting)
Shadow-mode defaults only shaped the RViz grid; live, they shape motion. Tune on
real/sim data before merge:
- `d_max_m` — cost falloff distance (gap-to-road-edge at which a ped stops
  contributing). Set the shadow default conservatively; confirm it here.
- `inflation_radius_m` (`0.8`) — too small clips the footprint, too large
  phantom-blocks lanes. Validate against observed routing.
- `publish_rate_hz` (`15.0`) — confirm it keeps pace with `/pedestrians` and the
  planner without starving the summation loop.

### 11.4 Acceptance before going live
- In sim/replay: in-road pedestrian (cost ≈ 100 ≥ 90) makes the planner route
  around; edge-of-road pedestrian reduces target speed without re-routing.
- Projected `pos_x`/`pos_y` land in plausible cells against a known-geometry
  scene (extrinsics sanity check).
- No regression in baseline routing when no pedestrians are present (all-zero
  grid contributes nothing through `np.maximum`).

## 12. Risks & limitations

- **Monocular depth** (`depth = f·H / h_px`) is sensitive to bbox height noise;
  range is approximate. Acceptable for a shadow layer and inflation absorbs some
  error.
- **Camera extrinsics** approximated as identity in v1 (see §5.2).
- The producer only emits pedestrians *facing* the road (existing filter), so
  the layer reflects intent detections, not all pedestrians — by design.
- **Post-merge documentation** (out of scope for code, tracked as follow-up):
  add `/grid/pedestrian` to `docs/topics.md`, the new fields to
  `docs/messages.md`, and the new outputs to the pedestrian-intent perception
  doc.
