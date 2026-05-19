# navigator_lane_change

Route-aware lane-change and obstacle-bypass behavior module for Navigator.

Runs in **shadow mode by default** — publishes decisions and debug output with zero vehicle control authority. Execution mode is enabled by a single parameter change once the node has been validated in shadow.

---

## Modes

### Shadow mode (default)
The node observes the environment, runs the full decision pipeline, and publishes its intent — but takes no action on the vehicle. Use this to validate behavior in simulation or on a real vehicle before enabling execution.

Set in `param/lane_change_params.yaml`:
```yaml
shadow_mode: true
```

### Execution mode
When you are ready to allow the node to actually command lane changes, set:
```yaml
shadow_mode: false
```

In execution mode the FSM transitions through to `EXECUTE_LANE_CHANGE` and the downstream trajectory node (PR 2) picks up the decision from `/behavior/lane_change_decision` to generate and follow a quintic path. No other code changes are needed — the interface is identical between modes.

---

## Triggering a lane change from another node

Any node in the stack can request a lane change by publishing a single string to:

```
/behavior/lane_change_command   (std_msgs/String)
```

Valid values: `"left"`, `"right"`, `"cancel"`

### Python example

```python
from std_msgs.msg import String

# In your node's __init__:
self._lc_pub = self.create_publisher(String, '/behavior/lane_change_command', 10)

# Request a right lane change:
msg = String()
msg.data = 'right'
self._lc_pub.publish(msg)

# Request a left lane change:
msg.data = 'left'
self._lc_pub.publish(msg)

# Cancel / abort an in-progress lane change:
msg.data = 'cancel'
self._lc_pub.publish(msg)
```

Commands take priority over obstacle-triggered lane changes. A command expires automatically after **30 seconds** — if the publishing node goes offline, the vehicle will not be left trying to change lanes indefinitely.

### From the terminal (testing / manual override)

```bash
# Change to right lane
ros2 topic pub --once /behavior/lane_change_command std_msgs/msg/String "data: 'right'"

# Change to left lane
ros2 topic pub --once /behavior/lane_change_command std_msgs/msg/String "data: 'left'"

# Cancel
ros2 topic pub --once /behavior/lane_change_command std_msgs/msg/String "data: 'cancel'"
```

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/gnss/odometry` | `nav_msgs/Odometry` | Ego position and velocity |
| `/planning/path` | `nav_msgs/Path` | Current planned path |
| `/objdet3d_tracked` | `navigator_msgs/Object3DArray` | Tracked objects from perception |
| `/intersection_status` | `navigator_msgs/IntersectionStatus` | Suppresses lane changes at intersections |
| `/behavior/lane_change_command` | `std_msgs/String` | External lane change request (`left`, `right`, `cancel`) |

### Published

| Topic | Type | Description |
|---|---|---|
| `/behavior/lane_change_state` | `std_msgs/String` | Current FSM state name |
| `/behavior/lane_change_decision` | `std_msgs/String` | JSON decision summary (consumed by trajectory node in PR 2) |
| `/behavior/lane_change_debug` | `std_msgs/String` | JSON full debug dump — all inputs, blockers, gap values |

---

## FSM states

```
IDLE
  └─► PREPARE_LANE_CHANGE   (obstacle detected or command received)
        └─► CHECK_TARGET_LANE
              └─► FIND_GAP
                    ├─► WAIT_FOR_GAP   (gap unsafe, waiting up to timeout)
                    │     └─► COMMIT_LANE_CHANGE
                    └─► COMMIT_LANE_CHANGE
                          └─► EXECUTE_LANE_CHANGE
                                └─► COMPLETE_LANE_CHANGE ─► IDLE

  ABORT_LANE_CHANGE ─► IDLE or ROUTE_BLOCKED
  ROUTE_BLOCKED     ─► IDLE or PREPARE_LANE_CHANGE
  FAULT             ─► IDLE  (auto-recovers when odometry is restored)
```

---

## Parameters (`param/lane_change_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `shadow_mode` | `true` | Set `false` to enable execution |
| `loop_rate_hz` | `10.0` | Node update rate |
| `command_topic` | `/behavior/lane_change_command` | Topic for external commands |
| `min_front_gap_m` | `12.0` | Minimum gap ahead in target lane (m) |
| `min_rear_gap_m` | `10.0` | Minimum gap behind in target lane (m) |
| `min_rear_ttc_s` | `4.0` | Minimum time-to-collision from rear (s) |
| `max_lane_change_speed_mps` | `8.0` | Speed above which lane change is blocked |
| `merging_conflict_lat_speed_mps` | `0.4` | Lateral speed threshold for merging conflict detection |
| `wait_for_gap_timeout_s` | `8.0` | How long to wait for a gap before giving up |
| `stale_input_timeout_s` | `0.5` | Odometry age above which node enters FAULT |

---

## Launch

```bash
ros2 launch navigator_lane_change lane_change_shadow.launch.py
```

---

## Running tests

No ROS installation required:

```bash
cd src/planning/navigator_lane_change
python3 -m pytest test/ -v
```

12 tests covering: fault recovery, obstacle detection, full FSM flow, speed limiting, intersection suppression, left/right/cancel commands, and merging conflict detection.
