# Path planner benchmark

Runs every planner behind `path_planner_node` on the same costmaps and scores the paths with one ruler, so the five planners can be compared with numbers instead of opinions.

The costmap goes through the node's own pre-planning code (`planning_core.build_problem`: rescale, place start and goal, pad obstacles, snap a blocked goal), and each planner is called through the same `planner_registry` entry the node uses. A live-node check (publish a scenario on `/grid/steering_cost` and `/planning/path_goal`, read `/planning/path`) reproduces the offline Dijkstra path pose for pose.

## Run it

Inside the navigator container, after `colcon build`:

```bash
ros2 run path_planners planner_benchmark --out /tmp/planner_bench
```

Without ROS, from `src/planning/path_planners` (needs numpy, scipy, matplotlib; networkx for ARA*, torch and scikit-image for the neural network):

```bash
python3 -m path_planners.benchmark --out /tmp/planner_bench
```

Useful options:

| option | default | meaning |
|---|---|---|
| `--planners` | all | comma list, e.g. `dijkstra,trrt-star` |
| `--scenarios` | all | comma list of synthetic scenarios, or `none` |
| `--costmaps` | | recorded `.npz` frames or folders of them |
| `--repeats` | 3 | back-to-back calls on one planner instance, as the node makes them |
| `--timeout` | 15 | seconds per call before it counts as a timeout |
| `--jobs` | 1 | parallel runs; keep 1 when timings matter |
| `--nn-model` | none | UNet checkpoint for `neural-network` (the node passes none today) |

Output: `summary.md` (scorecard plus a table and plot per scenario), `calls.csv` (every call) and one PNG per scenario with each planner's published path over the padded costmap and the exact reference dashed.

## Scenarios

Synthetic frames use the published grid (60 m x 60 m at 0.2 m, vehicle 20 m from the back) and the cost layers' values: 50 off-route, 10 in the route corridor, 0 on the centreline, 100 occupied.

| scenario | what it tests |
|---|---|
| `straight_clear`, `curve_left` | baseline, no obstacles |
| `parked_car`, `slalom`, `lane_closure` | swerving around cars and cones |
| `wall_with_gap` | a 1 m channel after padding |
| `u_trap` | a local minimum for goal-greedy methods |
| `detour_beyond_margin` | a detour more than 40 cells off the start-goal box |
| `goal_in_obstacle` | the node's goal snap |
| `unreachable` | the right answer is no path |
| `clutter_s0..2` | seeded random obstacles |

## How paths are scored

- **Reference:** exact 8-connected shortest path over the whole padded grid (scipy csgraph), with the same edge cost the stack's Dijkstra uses. It decides whether a path exists.
- **Valid:** starts at the start, ends at the goal (within 1.5 cells), and keeps the path centre at least the vehicle half-width from every real obstacle, on the raw path and on the smoothed path the node publishes. The half-width is the node's padding (5 cells = 1.0 m) less half a cell for grid rounding.
- **Correct:** valid when a path exists, no path when none exists. **Usable:** correct, and a valid path costs at most 2x the optimum.
- **Cost:** the path rasterized to 8-connected cells and charged the graph's edge cost, divided by the reference cost. 1.00 is optimal, and nothing can score below it. This is the objective the costmap encodes and the one the stack's Dijkstra minimizes, so Dijkstra scores 1.00 by construction; the other planners optimize their own objectives, so a ratio above 1 means costlier under the costmap's objective, not invalid.
- **Timing:** runtime per call against the node's 100 ms replanning period. A timed-out call counts as the timeout (shown as `>`), so a planner that hangs cannot look fast. The scorecard also reports whether every back-to-back call was correct, since some planners carry state between calls.
- Also reported: length, clearance, largest heading change, and memory growth across calls.

## Recorded costmaps

Real frames come from a rosbag of a CARLA (or vehicle) run that recorded `/grid/steering_cost` and `/planning/path_goal`:

```bash
ros2 bag record /grid/steering_cost /planning/path_goal
ros2 run path_planners extract_costmaps <bag_dir> --out frames/
ros2 run path_planners planner_benchmark --costmaps frames/ --scenarios none
```

Each costmap is paired with the latest goal received at or before it; costmaps with no goal yet, a goal older than 3 s (the node drops those too), or less than `--every` seconds after the last kept frame are skipped. Goals the node's ghost filter would reject are never paired.
