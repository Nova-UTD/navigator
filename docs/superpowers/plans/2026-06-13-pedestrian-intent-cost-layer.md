# Pedestrian-Intent Cost Layer (shadow mode) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Activate the dormant pedestrian-intent perception pipeline by projecting intent detections into a metric cost grid (`/grid/pedestrian`) that fits Navigator's layered-costmap architecture — published in shadow mode (observable in RViz, not yet wired into the planner).

**Architecture:** Two PRs. **PR 1** is a self-contained bugfix to the perception producer (a variable-shadowing bug that makes `width`/`height` carry full-image dimensions instead of bbox dimensions) plus its regression test. **PR 2** (rebases on PR 1) extends `PedestrianInfo.msg` with metric position fields, populates them in the producer via a monocular pinhole projection, adds a new `pedestrian_costmap_node` in the `costs` package (split into a pure-logic module + thin ROS node), and fully wires the node so it runs. `/grid/pedestrian` is **NOT** registered in `grid_summation_node` this round — the planned path is unaffected.

**Tech Stack:** Python 3, ROS 2 (rclpy), numpy, `nav_msgs/OccupancyGrid`, `navigator_msgs`, pytest. Source spec: `docs/superpowers/specs/2026-06-13-pedestrian-intent-cost-layer-design.md`.

---

## Design decisions made during planning (read before starting)

1. **PR 1 needs a pure test seam.** The producer node (`pedestrian_intent_to_enter_road.py`) imports `ultralytics` and `mmpose` at module load, so it cannot be imported in a unit test. To make the §5.1 fix testable (spec §10 requires a regression test in PR 1), the fix extracts the message-dimension logic into a new pure module `pedestrian_geometry.py` (no ROS / ML imports). The node calls it; the test exercises it directly. PR 2 grows the same module with the projection functions.

2. **`cam_offset_x` / `cam_offset_y` live on the producer, not the costmap node.** The spec lists these in `pedestrian_costmap_params.yaml` (§6.4/§8.4) but also says they are applied in the producer's projection (§5.2). Projection only happens in the producer, so the params are declared and consumed there. The costmap param file documents only the three params the costmap node actually uses (`publish_rate_hz`, `d_max_m`, `inflation_radius_m`); the producer declares `cam_offset_x/y` via `declare_parameter` (default `0.0`) so a measured extrinsic can be plugged in later (spec §11.2) with no code change. This is a deliberate, documented deviation from the literal param-file listing.

3. **`d_max_m` shadow default = `10.0` m.** The spec leaves the number open ("set conservatively", §11.3). `10.0` means a pedestrian 10 m from the road edge stops contributing cost; in-road (`distance = 0`) → cost 100. Tuned for real in Phase 3.

4. **Param file located via package share dir.** The launch file resolves the param path with `get_package_share_directory('costs')` and the param yaml is installed via `setup.py` `data_files`. This is robust to container layout (the existing `navigator_lane_change` launch hardcodes `/navigator/param/...`; we do not copy that fragility into new code, per spec §8.1's "do not propagate placeholders").

---

## File structure

**PR 1 (bugfix) — package `pedestrian_intent_to_enter_road`:**
- Create: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py` — pure geometry helpers (PR 1 adds `pedestrian_message_dims`).
- Modify: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road.py` — rename shadowed vars, call the helper.
- Create: `src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py` — regression test.

**PR 2 (feature):**
- Modify: `src/msg/navigator_msgs/msg/PedestrianInfo.msg` — append `pos_x`, `pos_y`.
- Modify: `pedestrian_geometry.py` — add `estimate_depth`, `project_to_base_link`.
- Modify: `pedestrian_intent_to_enter_road.py` — declare `cam_offset_x/y`, populate `pos_x/pos_y`.
- Modify: `test/test_pedestrian_geometry.py` — add projection tests.
- Create: `src/planning/costs/costs/pedestrian_costmap.py` — pure cost/projection/painting logic.
- Create: `src/planning/costs/costs/pedestrian_costmap_node.py` — thin ROS node.
- Create: `src/planning/costs/test/test_pedestrian_costmap.py` — pure-logic unit tests.
- Create: `src/planning/costs/launch/pedestrian_costmap.launch.py` — standalone launch.
- Create: `src/planning/costs/param/pedestrian_costmap_params.yaml` — documented params.
- Create: `src/planning/costs/README.md` — package README with node section.
- Modify: `src/planning/costs/setup.py` — entry point + install param/launch.
- Modify: `src/planning/costs/package.xml` — runtime + test deps.
- Modify (optional): `launches/launch_node_definitions.py` — add a Node definition.

---

# PR 1 — Bugfix: producer width/height variable shadowing

> Branch from `dev`: `git switch dev && git pull && git switch -c fix_pedestrian_bbox_dims`. This PR changes no live behavior (`PedestrianInfo` has no consumers) and merges independently.

## Task 1: Extract pure dimension helper + regression test

**Files:**
- Create: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py`
- Test: `src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py`

- [ ] **Step 1: Write the failing test**

Create `src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py`:

```python
"""Unit tests for pedestrian_geometry — no ROS / ML deps required."""

from pedestrian_intent_to_enter_road.pedestrian_geometry import (
    pedestrian_message_dims,
)


class TestPedestrianMessageDims:
    def test_dims_come_from_bbox_not_image(self):
        # YOLO xywh: a small 30x60 bbox centred at (100, 200).
        # Regression guard for the historical bug where the message
        # width/height were clobbered with the full image dimensions.
        xywh = (100.0, 200.0, 30.0, 60.0)
        x, y, w, h = pedestrian_message_dims(xywh)
        assert (w, h) == (30.0, 60.0)   # bbox dims, NOT a large image size

    def test_center_passthrough(self):
        xywh = (100.0, 200.0, 30.0, 60.0)
        x, y, w, h = pedestrian_message_dims(xywh)
        assert (x, y) == (100.0, 200.0)

    def test_returns_python_floats(self):
        # numpy float32 in -> plain float out (ROS msg fields are float32)
        import numpy as np
        xywh = np.array([1, 2, 3, 4], dtype=np.float32)
        result = pedestrian_message_dims(xywh)
        assert all(isinstance(v, float) for v in result)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/test_pedestrian_geometry.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'pedestrian_intent_to_enter_road.pedestrian_geometry'`

- [ ] **Step 3: Write minimal implementation**

Create `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py`:

```python
"""
Package: pedestrian_intent_to_enter_road
   File: pedestrian_geometry.py

Pure geometry helpers for the pedestrian-intent node. No ROS, no ML
dependencies, deterministic — unit-testable in isolation.
"""


def pedestrian_message_dims(xywh):
    """Return (x, y, width, height) for a PedestrianInfo message.

    Takes the YOLO ``xywh`` bounding box (center_x, center_y, box_width,
    box_height) and returns plain Python floats. The width/height are the
    BOUNDING BOX dimensions — never the full image dimensions. This is the
    single source of truth that guards against the historical variable-
    shadowing bug (image shape overwriting the bbox width/height).

    @param xywh  Iterable of 4 numbers: center_x, center_y, width, height (px).
    @return      Tuple of 4 Python floats: (x, y, width, height).
    """
    center_x, center_y, box_width, box_height = xywh
    return float(center_x), float(center_y), float(box_width), float(box_height)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/test_pedestrian_geometry.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py \
        src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py
git commit -m "test(perception): pure pedestrian_message_dims helper + regression test"
```

## Task 2: Wire the helper into the producer (fix the shadowing)

**Files:**
- Modify: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road.py`

- [ ] **Step 1: Add the import**

At the top of the file, after `from navigator_msgs.msg import PedestrianInfo` (line ~12), add:

```python
from pedestrian_intent_to_enter_road.pedestrian_geometry import (
    pedestrian_message_dims,
)
```

- [ ] **Step 2: Stop unpacking the shadowed bbox names**

In `detect_pedestrians()`, replace this line (currently ~L75):

```python
                    center_x, center_y, width, height = xywh
```

with (we only need `center_x` as a scalar for the facing logic; the message
dims now come from the helper):

```python
                    center_x = float(xywh[0])
```

- [ ] **Step 3: Rename the image-shape unpack so it cannot clobber bbox names**

Replace this line (currently ~L98):

```python
                    height, width, channels = self.image.shape
                    midpoint_x = width / 2
```

with:

```python
                    img_h, img_w = self.image.shape[:2]
                    midpoint_x = img_w / 2
```

- [ ] **Step 4: Assign the real bbox dimensions to the message via the helper**

Replace this block (currently ~L107-111):

```python
                        pedestrian_object.x = float(center_x)
                        pedestrian_object.y = float(center_y)
                        pedestrian_object.width = float(width)
                        pedestrian_object.height = float(height)
                        pedestrian_object.distance = float(horizontal_dist)
```

with:

```python
                        msg_x, msg_y, msg_w, msg_h = pedestrian_message_dims(xywh)
                        pedestrian_object.x = msg_x
                        pedestrian_object.y = msg_y
                        pedestrian_object.width = msg_w
                        pedestrian_object.height = msg_h
                        pedestrian_object.distance = float(horizontal_dist)
```

> Note: `calculate_horizontal_distance()` has its own local `height, width, channels = self.image.shape` near the end of the file — that one correctly uses the image width and is in a separate scope. **Leave it untouched.**

- [ ] **Step 5: Verify the regression test still passes and flake8 is clean**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/ -v`
Expected: PASS (the geometry test from Task 1 still passes; the node module is not imported by these tests so ML deps are not required).

Run (style gate the package already enforces): `python3 -m flake8 pedestrian_intent_to_enter_road/pedestrian_geometry.py`
Expected: no output (clean). If flake8 is unavailable locally, this runs in CI via the package's `ament_flake8` test.

- [ ] **Step 6: Commit**

```bash
git add src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road.py
git commit -m "fix(perception): publish bbox width/height, not full-image dims

The image-shape unpack shadowed the YOLO bbox width/height, so every
PedestrianInfo carried the full image dimensions. Rename the image-shape
locals (img_h/img_w) and source the message dims from the bbox via
pedestrian_message_dims()."
```

> **PR 1 is complete.** Open it against `dev`. PR 2 rebases on this branch.

---

# PR 2 — Feature: pedestrian cost layer (shadow mode)

> Branch: use the existing `feature_pedestrian_cost_layer` (rebased on PR 1's branch once merged). All work below is on this branch.

## Task 3: Extend `PedestrianInfo.msg` with metric position fields

**Files:**
- Modify: `src/msg/navigator_msgs/msg/PedestrianInfo.msg`

- [ ] **Step 1: Append the two fields (back-compatible — new fields at the end)**

The file currently is:

```
# MSG for one specific pedestrian intending on entering the road
float32 x               # X-coordinate of the pedestrian bounding box
float32 y               # Y-coordinate of the pedestrian bounding box
float32 width           # Width of the pedestrian bounding box
float32 height          # Height of the pedestrian bounding box
float32 distance        # Horizontal distance between the pedestrian and the road
```

Append after the `distance` line:

```
float32 pos_x           # metric forward distance in base_link (m), +x = ahead
float32 pos_y           # metric lateral offset in base_link (m), +y = left
```

- [ ] **Step 2: Rebuild the message package and verify the fields exist**

Run:
```bash
colcon build --packages-select navigator_msgs
source install/setup.bash
python3 -c "from navigator_msgs.msg import PedestrianInfo; m = PedestrianInfo(); print(m.pos_x, m.pos_y)"
```
Expected: `0.0 0.0` (fields exist, default zero). If the build environment is containerized, run the build inside the dev container per repo convention.

- [ ] **Step 3: Commit**

```bash
git add src/msg/navigator_msgs/msg/PedestrianInfo.msg
git commit -m "feat(msg): add metric pos_x/pos_y (base_link) to PedestrianInfo"
```

## Task 4: Add projection helpers to `pedestrian_geometry.py`

**Files:**
- Modify: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py`
- Test: `src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_pedestrian_geometry.py`:

```python
from pedestrian_intent_to_enter_road.pedestrian_geometry import (
    estimate_depth, project_to_base_link,
)

IMG_W = 672   # ZED 720p capture width (~672x376)


class TestEstimateDepth:
    def test_closer_pedestrian_has_larger_bbox_smaller_is_farther(self):
        # depth = f * H / h_px  -> taller bbox (more px) = nearer = smaller depth
        assert estimate_depth(200.0) < estimate_depth(100.0)

    def test_known_value(self):
        # 470 * 1.75 / 100 = 8.225 m
        assert abs(estimate_depth(100.0) - 8.225) < 1e-6

    def test_zero_or_negative_height_is_safe(self):
        assert estimate_depth(0.0) == 0.0
        assert estimate_depth(-5.0) == 0.0


class TestProjectToBaseLink:
    def test_centered_pedestrian_has_zero_lateral(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W)
        assert abs(pos_y) < 1e-6

    def test_right_of_center_is_negative_y(self):
        # center_x > cx  ->  vehicle's right  ->  negative pos_y
        _, pos_y = project_to_base_link(IMG_W / 2.0 + 100.0, 100.0, IMG_W)
        assert pos_y < 0.0

    def test_left_of_center_is_positive_y(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0 - 100.0, 100.0, IMG_W)
        assert pos_y > 0.0

    def test_pos_x_is_depth_plus_offset(self):
        pos_x, _ = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W,
                                        cam_offset_x=1.5)
        assert abs(pos_x - (estimate_depth(100.0) + 1.5)) < 1e-6

    def test_pos_y_offset_applied(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W,
                                        cam_offset_y=0.5)
        assert abs(pos_y - 0.5) < 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/test_pedestrian_geometry.py -v`
Expected: FAIL — `ImportError: cannot import name 'estimate_depth'`

- [ ] **Step 3: Write minimal implementation**

Append to `pedestrian_geometry.py`:

```python
# Monocular pinhole constants — tied to the ~672x376 capture resolution.
# These mirror the constants already used inside the producer's
# calculate_horizontal_distance() (focal 470, tan(HFOV/2) = 1.0913).
_FOCAL_PX = 470.0           # focal length (px)
_PERSON_HEIGHT_M = 1.75     # assumed real pedestrian height (m)
_TAN_HALF_HFOV = 1.0913     # tan(HFOV / 2)


def estimate_depth(bbox_pixel_height):
    """Forward range (m) from a monocular bbox pixel height.

    depth = focal * real_height / pixel_height. Returns 0.0 for a non-positive
    pixel height (degenerate bbox) so callers can skip it.

    @param bbox_pixel_height  Bounding-box height in pixels (BRy - TLy).
    @return                   Forward range in meters (float).
    """
    if bbox_pixel_height <= 0:
        return 0.0
    return _FOCAL_PX * _PERSON_HEIGHT_M / float(bbox_pixel_height)


def project_to_base_link(center_x, bbox_pixel_height, img_w,
                         cam_offset_x=0.0, cam_offset_y=0.0):
    """Project a pedestrian bbox to metric base_link coordinates.

    Sign convention: a pedestrian right of image center (center_x > img_w/2)
    maps to NEGATIVE pos_y (the vehicle's right). cam_offset_x/y add a static
    camera->base_link offset (default 0.0 = camera at base_link origin).

    @param center_x          Bbox center x in pixels.
    @param bbox_pixel_height  Bbox height in pixels (BRy - TLy).
    @param img_w             Image width in pixels.
    @param cam_offset_x      Forward camera offset (m).
    @param cam_offset_y      Lateral camera offset (m).
    @return                  Tuple (pos_x, pos_y) in meters, base_link.
    """
    depth = estimate_depth(bbox_pixel_height)
    cx = img_w / 2.0
    lateral = -((center_x - cx) / img_w) * (2.0 * depth * _TAN_HALF_HFOV)
    return depth + cam_offset_x, lateral + cam_offset_y
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/test_pedestrian_geometry.py -v`
Expected: PASS (all geometry + projection tests pass)

- [ ] **Step 5: Commit**

```bash
git add src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_geometry.py \
        src/perception/pedestrian_intent_to_enter_road/test/test_pedestrian_geometry.py
git commit -m "feat(perception): monocular base_link projection helpers + tests"
```

## Task 5: Populate `pos_x`/`pos_y` in the producer

**Files:**
- Modify: `src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road.py`

- [ ] **Step 1: Import the projection helper**

Extend the existing geometry import (added in PR 1) to:

```python
from pedestrian_intent_to_enter_road.pedestrian_geometry import (
    pedestrian_message_dims, project_to_base_link,
)
```

- [ ] **Step 2: Declare camera-offset parameters in `__init__`**

In `PedestrianIntentToEnterRoad.__init__`, after the existing variable setup
(after `self.binary_mask = None`), add:

```python
        # Static camera -> base_link offset (m). Defaults 0.0 = camera at the
        # base_link origin pointing straight forward (v1 shadow-mode assumption).
        # Set from the measured mount before the layer is wired live (Phase 3).
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        self.cam_offset_x = float(self.get_parameter('cam_offset_x').value)
        self.cam_offset_y = float(self.get_parameter('cam_offset_y').value)
```

- [ ] **Step 3: Compute and assign `pos_x`/`pos_y` where the message is built**

In `detect_pedestrians()`, the bbox corners `TLx, TLy, BRx, BRy = xyxy` and
`img_w` (from `self.image.shape[:2]`, renamed in PR 1) are already in scope. In
the message-construction block (the `pedestrian_object` assignments edited in
PR 1), after the `pedestrian_object.distance = ...` line, add:

```python
                        bbox_pixel_height = BRy - TLy
                        pos_x, pos_y = project_to_base_link(
                            center_x, bbox_pixel_height, img_w,
                            self.cam_offset_x, self.cam_offset_y)
                        pedestrian_object.pos_x = float(pos_x)
                        pedestrian_object.pos_y = float(pos_y)
```

- [ ] **Step 4: Verify geometry tests still pass**

Run: `cd src/perception/pedestrian_intent_to_enter_road && python3 -m pytest test/ -v`
Expected: PASS. (The projection math itself is covered by Task 4's unit tests; this step only confirms nothing in the pure module regressed.)

- [ ] **Step 5: Commit**

```bash
git add src/perception/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road/pedestrian_intent_to_enter_road.py
git commit -m "feat(perception): populate PedestrianInfo pos_x/pos_y from projection"
```

## Task 6: Pure cost mapping — `distance_to_cost`

**Files:**
- Create: `src/planning/costs/costs/pedestrian_costmap.py`
- Test: `src/planning/costs/test/test_pedestrian_costmap.py`

- [ ] **Step 1: Write the failing test**

Create `src/planning/costs/test/test_pedestrian_costmap.py`:

```python
"""Unit tests for costs/pedestrian_costmap.py — no ROS spin required."""

import numpy as np

from costs.pedestrian_costmap import distance_to_cost

D_MAX = 10.0


class TestDistanceToCost:
    def test_in_road_is_max_cost(self):
        # distance 0 (pedestrian in the road) -> full cost
        assert distance_to_cost(0.0, D_MAX) == 100

    def test_at_dmax_is_zero(self):
        assert distance_to_cost(D_MAX, D_MAX) == 0

    def test_beyond_dmax_is_clipped_to_zero(self):
        assert distance_to_cost(D_MAX + 5.0, D_MAX) == 0

    def test_midrange_is_proportional(self):
        assert distance_to_cost(5.0, 10.0) == 50

    def test_degenerate_dmax_is_safe(self):
        assert distance_to_cost(1.0, 0.0) == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'costs.pedestrian_costmap'`

- [ ] **Step 3: Write minimal implementation**

Create `src/planning/costs/costs/pedestrian_costmap.py`:

```python
"""
Package: costs
   File: pedestrian_costmap.py
 Author: Shrey Joshi

Pure-logic for the pedestrian-intent cost layer — cost mapping, base_link ->
grid projection, and disk painting. No ROS imports; deterministic and
unit-testable. The thin ROS node pedestrian_costmap_node.py delegates all math
here (mirrors how navigator_lane_change splits logic from the node).

Grid contract (must match grid_summation_node / the other cost layers):
  frame_id   = base_link
  size       = 151 x 151 cells, resolution 0.4 m/cell
  origin     = (x=-20.0 longitudinal, y=-30.0 lateral) in base_link
  ego cell   = (row=75, col=50)
  rows       = lateral axis    (+y = left,  -y = right)
  cols       = longitudinal    (+x = ahead, -x = behind)
"""

from typing import Optional, Tuple

import numpy as np


def distance_to_cost(distance_m: float, d_max_m: float) -> int:
    """Map a gap-to-road-edge distance to a cost magnitude in [0, 100].

    distance 0 (pedestrian in the road) -> 100; distance >= d_max -> 0; linear
    in between. This sets cost MAGNITUDE only — where the cost is painted is
    decided separately by pose_to_grid_coords.

    @param distance_m  Gap from pedestrian to road edge (m).
    @param d_max_m     Distance at which a pedestrian stops contributing (m).
    @return            Integer cost in [0, 100].
    """
    if d_max_m <= 0:
        return 0
    cost = 100.0 * (1.0 - distance_m / d_max_m)
    return int(round(float(np.clip(cost, 0, 100))))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add src/planning/costs/costs/pedestrian_costmap.py \
        src/planning/costs/test/test_pedestrian_costmap.py
git commit -m "feat(costs): distance_to_cost pure mapping + tests"
```

## Task 7: Pure projection — `pose_to_grid_coords`

**Files:**
- Modify: `src/planning/costs/costs/pedestrian_costmap.py`
- Test: `src/planning/costs/test/test_pedestrian_costmap.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_pedestrian_costmap.py`:

```python
from costs.pedestrian_costmap import pose_to_grid_coords

GRID_SIZE = 151
RESOLUTION = 0.4
ORIGIN_X = -20.0
ORIGIN_Y = -30.0


def grid_coords(pos_x, pos_y):
    return pose_to_grid_coords(pos_x, pos_y, ORIGIN_X, ORIGIN_Y,
                               RESOLUTION, GRID_SIZE)


class TestPoseToGridCoords:
    def test_ego_origin_maps_to_center_cell(self):
        # (0,0) -> col = (0 - -20)/0.4 = 50 ; row = (0 - -30)/0.4 = 75
        assert grid_coords(0.0, 0.0) == (75, 50)

    def test_ahead_increases_col(self):
        row, col = grid_coords(10.0, 0.0)
        assert col > 50 and row == 75

    def test_behind_decreases_col(self):
        row, col = grid_coords(-10.0, 0.0)
        assert col < 50 and row == 75

    def test_left_increases_row(self):
        row, col = grid_coords(0.0, 10.0)
        assert row > 75 and col == 50

    def test_right_decreases_row(self):
        row, col = grid_coords(0.0, -10.0)
        assert row < 75 and col == 50

    def test_off_grid_returns_none(self):
        assert grid_coords(1000.0, 0.0) is None
        assert grid_coords(0.0, 1000.0) is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'pose_to_grid_coords'`

- [ ] **Step 3: Write minimal implementation**

Append to `pedestrian_costmap.py`:

```python
def pose_to_grid_coords(pos_x_m: float, pos_y_m: float,
                        origin_x_m: float, origin_y_m: float,
                        resolution_m: float, grid_size: int
                        ) -> Optional[Tuple[int, int]]:
    """Convert a base_link metric pose to (row, col) grid indices.

    cols track longitudinal +x (ahead), rows track lateral +y (left), matching
    the layer's grid contract. Returns None if the pose falls outside the grid.

    @param pos_x_m       Forward position in base_link (m).
    @param pos_y_m       Lateral position in base_link (m).
    @param origin_x_m    Grid origin x (longitudinal min, m).
    @param origin_y_m    Grid origin y (lateral min, m).
    @param resolution_m  Cell size (m/cell).
    @param grid_size     Grid edge length (cells).
    @return              (row, col) tuple, or None if off-grid.
    """
    col = int(round((pos_x_m - origin_x_m) / resolution_m))
    row = int(round((pos_y_m - origin_y_m) / resolution_m))
    if 0 <= row < grid_size and 0 <= col < grid_size:
        return (row, col)
    return None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/planning/costs/costs/pedestrian_costmap.py \
        src/planning/costs/test/test_pedestrian_costmap.py
git commit -m "feat(costs): pose_to_grid_coords base_link->grid projection + tests"
```

## Task 8: Pure painting — `paint_disk`

**Files:**
- Modify: `src/planning/costs/costs/pedestrian_costmap.py`
- Test: `src/planning/costs/test/test_pedestrian_costmap.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_pedestrian_costmap.py`:

```python
from costs.pedestrian_costmap import paint_disk


class TestPaintDisk:
    def test_disk_is_centered_on_cell(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        paint_disk(grid, 75, 50, 0.8, RESOLUTION, 100)
        assert grid[75, 50] == 100

    def test_radius_extends_to_neighbors(self):
        # radius 0.8 m / 0.4 = 2 cells -> a cell 2 away is painted
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        paint_disk(grid, 75, 50, 0.8, RESOLUTION, 100)
        assert grid[75, 52] == 100   # 2 cells along the row, within radius
        assert grid[75, 53] == 0     # 3 cells away, outside radius

    def test_dtype_stays_int16(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        paint_disk(grid, 75, 50, 0.8, RESOLUTION, 100)
        assert grid.dtype == np.int16

    def test_overlap_combines_via_maximum(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        paint_disk(grid, 75, 50, 0.8, RESOLUTION, 40)
        paint_disk(grid, 75, 51, 0.8, RESOLUTION, 90)
        # cell (75,50) is inside both disks -> the larger cost wins
        assert grid[75, 50] == 90

    def test_disk_near_edge_is_clipped_safely(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        paint_disk(grid, 0, 0, 0.8, RESOLUTION, 100)  # must not raise
        assert grid[0, 0] == 100
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: FAIL — `ImportError: cannot import name 'paint_disk'`

- [ ] **Step 3: Write minimal implementation**

Append to `pedestrian_costmap.py`:

```python
def paint_disk(grid: np.ndarray, row: int, col: int,
               radius_m: float, resolution_m: float, cost: int) -> None:
    """Paint a filled disk of `cost` into `grid`, combined via np.maximum.

    Mutates `grid` in place. Overlapping disks (multiple pedestrians) combine
    by taking the per-cell maximum, never summing. Out-of-bounds cells are
    skipped. Grid dtype is preserved (work in int16).

    @param grid          2D int16 cost array (modified in place).
    @param row           Disk center row index.
    @param col           Disk center col index.
    @param radius_m      Disk radius in meters (footprint + safety pad).
    @param resolution_m  Cell size (m/cell).
    @param cost          Cost value to paint.
    """
    radius_cells = radius_m / resolution_m
    r_int = int(np.ceil(radius_cells))
    rows, cols = grid.shape

    r_lo, r_hi = max(0, row - r_int), min(rows, row + r_int + 1)
    c_lo, c_hi = max(0, col - r_int), min(cols, col + r_int + 1)
    if r_lo >= r_hi or c_lo >= c_hi:
        return

    rr, cc = np.ogrid[r_lo:r_hi, c_lo:c_hi]
    mask = (rr - row) ** 2 + (cc - col) ** 2 <= radius_cells ** 2
    sub = grid[r_lo:r_hi, c_lo:c_hi]
    sub[mask] = np.maximum(sub[mask], cost)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/planning/costs && python3 -m pytest test/test_pedestrian_costmap.py -v`
Expected: PASS (all distance/coords/disk tests)

- [ ] **Step 5: Commit**

```bash
git add src/planning/costs/costs/pedestrian_costmap.py \
        src/planning/costs/test/test_pedestrian_costmap.py
git commit -m "feat(costs): paint_disk inflated-disk painter (np.maximum) + tests"
```

## Task 9: Thin ROS node — `pedestrian_costmap_node`

**Files:**
- Create: `src/planning/costs/costs/pedestrian_costmap_node.py`

> No unit test here — the node is a thin shell; all logic is covered by Tasks 6-8. It is validated by running it (Task 11) and in Phase 3 acceptance.

- [ ] **Step 1: Write the node**

Create `src/planning/costs/costs/pedestrian_costmap_node.py`:

```python
"""
Package: costs
   File: pedestrian_costmap_node.py
 Author: Shrey Joshi

Publishes /grid/pedestrian — a pedestrian-intent cost layer. SHADOW MODE:
this grid is observable in RViz but is NOT registered in grid_summation_node,
so the planned path is unaffected. Subscribes /pedestrians
(navigator_msgs/PedestrianInfoDetections); all math is delegated to the pure
module pedestrian_costmap.py.

Grid contract (must match grid_summation_node / path_planner_node):
  frame_id   = base_link
  size       = 151 x 151 cells, resolution 0.4 m/cell
  origin     = (x=-20.0, y=-30.0) in base_link  (this exact order: #494 regression)
  width/height never transposed (#493 regression; square here but stated so)
  data       = arr.flatten().tolist(), row-major, clipped 0-100
  stamped from /clock; timer-driven publish; keeps only the newest /pedestrians
"""

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from navigator_msgs.msg import PedestrianInfoDetections

from costs.pedestrian_costmap import (
    distance_to_cost, pose_to_grid_coords, paint_disk,
)

# ── Grid constants (match the other cost layers) ───────────────────────────────
GRID_SIZE  = 151
RESOLUTION = 0.4      # m/cell
ORIGIN_X   = -20.0    # m in base_link (longitudinal)
ORIGIN_Y   = -30.0    # m in base_link (lateral)
FRAME_ID   = 'base_link'


class PedestrianCostmapNode(Node):

    def __init__(self):
        super().__init__('pedestrian_costmap_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('d_max_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.8)

        self._d_max     = float(self.get_parameter('d_max_m').value)
        self._inflation = float(self.get_parameter('inflation_radius_m').value)

        # ── Internal state ────────────────────────────────────────────────────
        self._sim_clock = Clock()   # avoids shadowing rclpy.Node._clock
        self._latest = None         # newest PedestrianInfoDetections

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Clock, '/clock', self._cb_clock, 1)
        self.create_subscription(
            PedestrianInfoDetections, '/pedestrians', self._cb_pedestrians, 1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(OccupancyGrid, '/grid/pedestrian', 1)

        # ── Publish timer ─────────────────────────────────────────────────────
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info('PedestrianCostmapNode ready (shadow mode).')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_clock(self, msg: Clock) -> None:
        self._sim_clock = msg

    def _cb_pedestrians(self, msg: PedestrianInfoDetections) -> None:
        self._latest = msg

    # ── Costmap painting ──────────────────────────────────────────────────────

    def _paint(self) -> np.ndarray:
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        if self._latest is None:
            return np.clip(grid, 0, 100).astype(np.int8)

        for ped in self._latest.pedestrians:
            cost = distance_to_cost(ped.distance, self._d_max)
            if cost <= 0:
                continue
            rc = pose_to_grid_coords(
                ped.pos_x, ped.pos_y, ORIGIN_X, ORIGIN_Y, RESOLUTION, GRID_SIZE)
            if rc is None:
                self.get_logger().debug(
                    f'pedestrian off-grid: pos=({ped.pos_x:.1f}, {ped.pos_y:.1f})')
                continue
            row, col = rc
            paint_disk(grid, row, col, self._inflation, RESOLUTION, cost)

        return np.clip(grid, 0, 100).astype(np.int8)

    # ── Timer callback ────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        arr = self._paint()

        msg = OccupancyGrid()
        msg.header.stamp           = self._sim_clock.clock
        msg.header.frame_id        = FRAME_ID
        msg.info.resolution        = RESOLUTION
        msg.info.width             = GRID_SIZE   # cols — never transposed (#493)
        msg.info.height            = GRID_SIZE   # rows
        msg.info.origin.position.x = ORIGIN_X    # this order — #494 regression
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.data = arr.flatten().tolist()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Sanity-check it imports (pure-logic path)**

Run: `cd src/planning/costs && python3 -c "import ast; ast.parse(open('costs/pedestrian_costmap_node.py').read()); print('parse ok')"`
Expected: `parse ok` (full import requires a sourced ROS env; that is exercised in Task 11).

- [ ] **Step 3: Commit**

```bash
git add src/planning/costs/costs/pedestrian_costmap_node.py
git commit -m "feat(costs): pedestrian_costmap_node (shadow mode) publishing /grid/pedestrian"
```

## Task 10: Package wiring — setup.py, package.xml, param, launch, README

**Files:**
- Modify: `src/planning/costs/setup.py`
- Modify: `src/planning/costs/package.xml`
- Create: `src/planning/costs/param/pedestrian_costmap_params.yaml`
- Create: `src/planning/costs/launch/pedestrian_costmap.launch.py`
- Create: `src/planning/costs/README.md`

- [ ] **Step 1: Add the entry point and install param/launch in `setup.py`**

In `src/planning/costs/setup.py`, update the `data_files` list to also install
the param directory. Replace:

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
```

with:

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
```

And add the entry point to `console_scripts`:

```python
    entry_points={
        'console_scripts': [
            'grid_summation_node = costs.grid_summation_node:main',
            'route_costmap_node = costs.route_costmap_node:main',
            'junction_manager = costs.junction_manager:main',
            'pedestrian_costmap_node = costs.pedestrian_costmap_node:main',
        ],
    },
```

- [ ] **Step 2: Add runtime + test deps in `package.xml`**

In `src/planning/costs/package.xml`, replace the dependency block:

```xml
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <depend>nav_msgs</depend>
  <depend>tf_transformations</depend>
```

with:

```xml
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <depend>nav_msgs</depend>
  <depend>tf_transformations</depend>
  <depend>rclpy</depend>
  <depend>navigator_msgs</depend>
  <depend>rosgraph_msgs</depend>
```

- [ ] **Step 3: Create the param file**

Create `src/planning/costs/param/pedestrian_costmap_params.yaml`:

```yaml
pedestrian_costmap_node:
  ros__parameters:
    publish_rate_hz: 15.0        # Hz — timer-driven publish rate of /grid/pedestrian
    d_max_m: 10.0                # m — gap-to-road-edge at which a pedestrian stops
                                 #     contributing cost (distance 0 -> cost 100)
    inflation_radius_m: 0.8      # m — painted disk radius (footprint + safety pad)

# NOTE: cam_offset_x / cam_offset_y are NOT here — they configure the monocular
# projection, which runs in the producer node (pedestrian_intent_to_enter_road),
# not this costmap node. Set them there (default 0.0) from the measured
# camera->base_link mount before the layer is wired live (Phase 3, spec 11.2).
```

- [ ] **Step 4: Create the launch file**

Create `src/planning/costs/launch/pedestrian_costmap.launch.py`:

```python
"""
Shadow-mode launch for pedestrian_costmap_node.

Starts the node with its param file. The node publishes /grid/pedestrian but is
NOT registered in grid_summation_node, so it does not affect the planned path —
it is observable in RViz only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('costs'),
        'param', 'pedestrian_costmap_params.yaml')

    pedestrian_costmap_node = Node(
        package='costs',
        executable='pedestrian_costmap_node',
        name='pedestrian_costmap_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([pedestrian_costmap_node])
```

- [ ] **Step 5: Create the package README**

Create `src/planning/costs/README.md`:

```markdown
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
```

- [ ] **Step 6: Build the costs package and confirm the executable registers**

Run:
```bash
colcon build --packages-select costs
source install/setup.bash
ros2 pkg executables costs | grep pedestrian_costmap_node
```
Expected: `costs pedestrian_costmap_node`

- [ ] **Step 7: Commit**

```bash
git add src/planning/costs/setup.py src/planning/costs/package.xml \
        src/planning/costs/param/pedestrian_costmap_params.yaml \
        src/planning/costs/launch/pedestrian_costmap.launch.py \
        src/planning/costs/README.md
git commit -m "feat(costs): wire pedestrian_costmap_node (entry point, launch, params, README)"
```

## Task 11: End-to-end shadow run verification

**Files:** none (verification only)

- [ ] **Step 1: Run the full costs test suite**

Run: `cd src/planning/costs && python3 -m pytest test/ -v`
Expected: PASS (distance_to_cost, pose_to_grid_coords, paint_disk — all green)

- [ ] **Step 2: Launch the node and confirm it publishes**

In a sourced ROS env (`source install/setup.bash`):
```bash
ros2 launch costs pedestrian_costmap.launch.py &
ros2 topic hz /grid/pedestrian
```
Expected: `~15.0` Hz publish rate; node logs `PedestrianCostmapNode ready (shadow mode).`

- [ ] **Step 3: Confirm grid shape/origin on the wire**

Run: `ros2 topic echo /grid/pedestrian --field info --once`
Expected: `resolution: 0.4`, `width: 151`, `height: 151`, `origin.position.x: -20.0`, `origin.position.y: -30.0`. With no `/pedestrians` upstream, `data` is all zeros (all-zero grid). Stop the node when done.

- [ ] **Step 4: Confirm shadow-mode invariant (NOT wired into summation)**

Run: `grep -n "pedestrian" src/planning/costs/costs/grid_summation_node.py`
Expected: **no matches** — `grid_summation_node` has no `/grid/pedestrian` subscription, callback, SCALE constant, or `grids`-list entry. This is the shadow-mode guarantee: the planned path is unaffected by this PR.

- [ ] **Step 5 (optional): Register the node in the stack launch definitions**

> Optional, low-risk. Adds a reusable Node definition so the shadow node can come
> up with the stack. It remains shadow (absent from `grid_summation_node`).

In `launches/launch_node_definitions.py`, near the other `costs` Node
definitions (e.g. after `grid_route_costmap`), add:

```python
pedestrian_costmap = Node(
    package='costs',
    executable='pedestrian_costmap_node'
)
```

Then commit:

```bash
git add launches/launch_node_definitions.py
git commit -m "chore(launch): add pedestrian_costmap_node definition (shadow)"
```

---

## Out of scope (Phase 3 — separate PR, spec §11)

Do **not** implement these now; they are listed so the boundary is explicit:
- Registering `/grid/pedestrian` in `grid_summation_node` with a dedicated
  `'pedestrian'` branch routing into **both** `steering_cost` and `speed_cost`
  (spec §7/§11.1 — not a one-line change).
- Real camera extrinsics / re-validating the pinhole constants against the rig
  (spec §11.2).
- Tuning `d_max_m`, `inflation_radius_m`, `publish_rate_hz` on real/sim data
  once the layer affects motion (spec §11.3).
- Docs updates: `/grid/pedestrian` in `docs/topics.md`, the new fields in
  `docs/messages.md`, the perception doc (spec §12).

---

## Self-review notes

- **Spec coverage:** §4 → Task 3; §5.1 → PR 1 (Tasks 1-2); §5.2 → Tasks 4-5;
  §6.1 module split → Tasks 6-9; §6.2 grid contract → Task 9; §6.3 painting →
  Tasks 6-8 + Task 9 `_paint`; §6.4 conventions/params → Tasks 9-10; §8 wiring →
  Task 10; §9 error handling → Task 9 (`None` skip + debug log, all-zero grid);
  §10 testing → Tasks 1,4,6,7,8; §2 PR split → PR 1 / PR 2 structure.
- **Deliberate deviation:** `cam_offset_x/y` placed on the producer, not the
  costmap param file (documented in "Design decisions" #2 and the param yaml).
- **Type consistency:** `distance_to_cost`/`pose_to_grid_coords`/`paint_disk`
  signatures are identical across the pure module (Tasks 6-8), the node import
  (Task 9), and the tests. Grid is `int16` in-work, `int8` on publish throughout.
```
