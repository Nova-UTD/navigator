# pedestrian_crossing

Detects pedestrians on or approaching the vehicle's planned route and publishes a
time-to-collision-based status signal for the behavior layer.

**Decision-only.** The node publishes `/pedestrian_status` and takes no vehicle
control authority. Nothing in the stack consumes it until someone wires
`PedestrianHandler` into a controller — see [Integration](#integration). This
mirrors how `navigator_lane_change` landed: full pipeline, published decisions,
zero control authority until validated.

---

## Interfaces

| Direction | Topic | Type | Required |
|---|---|---|---|
| in | `/tracked/objects3d` | `navigator_msgs/Object3DArray` | yes |
| in | `/planning/path` | `nav_msgs/Path` | yes |
| in | `/pedestrians` | `navigator_msgs/PedestrianInfoDetections` | no |
| in | `/carla/hero/rgb_front/camera_info` | `sensor_msgs/CameraInfo` | no |
| out | `/pedestrian_status` | `navigator_msgs/PedestrianStatusArray` | — |

Every topic is a ROS parameter, so retargeting needs no code change.

## Output

Each `PedestrianStatus` carries two independent classifications:

- **`intent`** — what the pedestrian appears to be doing: `IRRELEVANT`,
  `UNCERTAIN`, `INTENT_TO_CROSS`, `CROSSING`, `CLEARED`.
- **`status`** — what the vehicle should do about it: `CLEAR`, `SLOW`, `STOP`.

`status` is driven by TTC against the path corridor:

| TTC | status |
|---|---|
| `< 2 s` | `STOP` |
| `2–5 s` | `SLOW` |
| `> 5 s`, or no predicted intersection | `CLEAR` |

Confirmed intent then escalates it, which covers two cases TTC alone misses: a
pedestrian standing motionless in the corridor produces no velocity and therefore
no TTC, but is still a `STOP`; and a pedestrian confirmed to be walking into the
road is at least a `SLOW` before the 5 s horizon reaches them.

`ttc` is `-1.0` when no intersection is predicted. `camera_corroborated` records
whether the optional camera layer contributed to that entry.

## Frames

All geometry runs in the frame of `/planning/path`, which `path_planner_node`
publishes as `base_link`. Tracked objects arrive in whatever frame
`lidar_detection` stamped them with (`mot3d_node` passes it through unchanged),
so if that differs from the path frame the objects are transformed via TF first —
one lookup per frame, not per pedestrian. When both are `base_link`, no TF lookup
happens at all.

## Failure behavior

The node distinguishes two situations that both look like "no data":

- **Never received a message** — the pipeline simply is not running. The node
  publishes an empty array and stays transparent. Publishing `STOP` here would
  mean that merely building this package puts a vehicle into permanent braking.
- **Received messages, then lost them** — a real fault. The node publishes a
  single synthetic entry with `pedestrian_id == PedestrianStatus.FAILSAFE_ID`
  and `status == STOP`.

`PedestrianHandler` applies the same rule on the consuming side.

## The optional camera layer

`/pedestrians` comes from `pedestrian_intent_to_enter_road`, which contributes a
gaze and distance-to-road cue that can promote `UNCERTAIN` to `INTENT_TO_CROSS`.

Two things about that upstream are worth knowing:

1. It is **commented out** of `launches/launch.perception.py`, so it does not run
   by default.
2. Its `distance` field is computed from `/segmentation_mask`, and **nothing in
   the repo publishes that topic**. Without it, `calculate_horizontal_distance`
   returns a `1e6` sentinel. This node treats that value as "distance unknown"
   rather than "very far from the road".

So the camera layer is strictly enrichment. Every pedestrian is fully classified
from the 3D tracker alone. Set `use_camera_layer:=false` to skip the
subscriptions entirely.

## Running

```bash
colcon build --packages-select navigator_msgs pedestrian_crossing
. install/setup.bash
ros2 run pedestrian_crossing pedestrian_crossing_node
```

Live input requires the 3D detection pipeline, which is also commented out of
`launches/launch.perception.py` — uncomment `*object3d_launch_entities` and make
sure the `complex_yolo` model is present in `/navigator_binaries`.

## Tests

```bash
colcon test --packages-select pedestrian_crossing
# or directly, no ROS needed:
python3 -m pytest src/planning/pedestrian_crossing/test -q
```

`crossing_logic.py` and `pedestrian_handler.py` have no ROS dependencies, so the
whole decision pipeline is testable without a graph — the same split
`navigator_lane_change` uses. `pedestrian_crossing_node.py` is ROS I/O only.

---

## Integration

`PedestrianHandler` is a pure logic class with the same shape as
`autonomous_cruise/intersection_handler.py`, so it drops into a control loop the
same way. **No file in `autonomous_cruise` has been modified** — the patch below
is for whoever owns that node to apply once this has been validated in shadow.

```python
# imports
from navigator_msgs.msg import PedestrianStatusArray
from pedestrian_crossing.pedestrian_handler import PedestrianHandler

# __init__
self.pedestrian_handler = PedestrianHandler()
self.pedestrian_sub = self.create_subscription(
    PedestrianStatusArray, '/pedestrian_status',
    self.pedestrian_status_callback, qos_reliable)

# callback
def pedestrian_status_callback(self, msg: PedestrianStatusArray):
    now = self.get_clock().now().nanoseconds * 1e-9
    self.pedestrian_handler.update_pedestrian_status(msg, now)

# control_loop(), before longitudinal_controller.compute_control()
now = self.get_clock().now().nanoseconds * 1e-9
self.pedestrian_handler.tick(now)          # expires the post-stop hold
target_speed = self.pedestrian_handler.get_target_speed(target_speed)
ped_decel = self.pedestrian_handler.compute_pedestrian_deceleration(current_speed)
```

`tick()` must be called every loop, not only on message receipt — the post-stop
hold and the producer-failure timeout are both time-based.

`get_target_speed()` composes with whatever the adaptive cruise controller
already decided: pass its target in, use the result. `compute_pedestrian_deceleration()`
returns `None` for `CLEAR` and `SLOW`, where the existing PID handles the
approach on its own, and a clamped rate between `COMFORTABLE_DECEL` and
`EMERGENCY_DECEL` while stopping.

### Interaction with existing adaptive cruise

`autonomous_cruise/longitudinal_controller.py` already has lead-object detection,
but it filters to `Object3D.CAR` only, so pedestrians never reach it. This
package is a separate channel and the two do not currently overlap. If that
filter is ever widened to include pedestrians, the two paths will compete over
target speed and one of them should win explicitly.
