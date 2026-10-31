"""
Extract benchmark costmap frames from a recorded rosbag2.

Package:   path_planners
Filename:  benchmark/extract_costmaps.py
Author:    Shifat Islam Santo

    python3 -m path_planners.benchmark.extract_costmaps BAG_DIR --out DIR
        [--every 1.0] [--max-goal-age 3.0] [--limit N]

Each /grid/steering_cost message is paired with the latest /planning/path_goal
received at or before it (bag receive time). It becomes a frame when that goal
is at most --max-goal-age seconds old (path_planner_node drops goals older than
3 s) and it is at least --every seconds after the last frame kept. Goals the
node's ghost filter would drop (header stamp older than a goal it already took)
are never paired. Frames are frame_NNNN.npz files for scenarios.load_recorded:

    python3 -m path_planners.benchmark --scenarios none --costmaps DIR

Needs a sourced ROS 2 Humble (rclpy, rosbag2_py). Only this module imports
them, and only inside functions, so the benchmark itself runs without ROS.
Reads sqlite3 bags and, with ros-humble-rosbag2-storage-mcap installed, mcap.
"""

import argparse
import bisect
import glob
import math
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

COSTMAP_TOPIC = '/grid/steering_cost'
COSTMAP_TYPE = 'nav_msgs/msg/OccupancyGrid'
GOAL_TOPIC = '/planning/path_goal'
GOAL_TYPE = 'geometry_msgs/msg/PoseStamped'

Goal = Tuple[int, float, float]  # bag receive time (ns), base_link x, y


def bag_metadata(bag: str):
    """Read the bag's metadata, which names its storage plugin (sqlite3 or mcap)."""
    import rosbag2_py

    if os.path.isdir(bag) and not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        raise ValueError(f'{bag} has no metadata.yaml (recording cut short?). Rebuild it '
                         f'with: ros2 bag reindex {bag}')
    # Same lookup SequentialReader.open does: metadata.yaml in a bag directory,
    # or for a lone .db3/.mcap file every installed storage plugin in turn
    try:
        meta = rosbag2_py.Info().read_metadata(bag, '')
    except RuntimeError as e:
        hint = ' (mcap needs ros-humble-rosbag2-storage-mcap)' if bag.endswith('.mcap') else ''
        raise ValueError(f'Cannot read {bag}: {e}{hint}') from e
    if meta.storage_identifier not in rosbag2_py.get_registered_readers():
        raise ValueError(f"{bag} is stored as '{meta.storage_identifier}' and no installed "
                         'rosbag2 plugin reads it (mcap: apt install '
                         'ros-humble-rosbag2-storage-mcap)')
    topics = {t.topic_metadata.name: t.topic_metadata.type
              for t in meta.topics_with_message_count}
    for topic, msg_type in ((COSTMAP_TOPIC, COSTMAP_TYPE), (GOAL_TOPIC, GOAL_TYPE)):
        if topic not in topics:
            raise ValueError(f"{bag} has no {topic}. Topics: {', '.join(sorted(topics))}")
        if topics[topic] != msg_type:
            raise ValueError(f'{topic} in {bag} is {topics[topic]}, expected {msg_type}')
    return meta


def open_reader(bag: str, meta, topic: str):
    """Open a reader that yields only `topic`, in bag receive-time order."""
    import rosbag2_py

    # The plain reader cannot read a bag recorded with --compression-mode. In
    # file mode Humble's reader leaves the decompressed .db3 beside the .zstd
    reader = (rosbag2_py.SequentialCompressionReader() if meta.compression_format
              else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta.storage_identifier),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def read_goals(bag: str, meta) -> Tuple[List[Goal], int]:
    """Every goal the node would accept, by receive time, and how many it would drop."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    pose_stamped = get_message(GOAL_TYPE)
    reader = open_reader(bag, meta, GOAL_TOPIC)
    received = []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        msg = deserialize_message(data, pose_stamped)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        received.append((t_ns, stamp, msg.pose.position.x, msg.pose.position.y))
    received.sort(key=lambda g: g[0])  # a split bag can roll over slightly out of order

    # path_planner_node.path_goal_callback: drop a goal stamped older than one it took
    goals, best_stamp, ghosts = [], 0.0, 0
    for t_ns, stamp, x, y in received:
        if stamp < best_stamp:
            ghosts += 1
            continue
        best_stamp = stamp
        goals.append((t_ns, float(x), float(y)))
    return goals, ghosts


def write_frame(path: str, **arrays):
    """Write one .npz atomically, so an interrupted run never leaves half a frame."""
    part = path + '.part'
    with open(part, 'wb') as f:
        np.savez_compressed(f, **arrays)
    os.replace(part, path)


def extract(bag: str, out: str, every: float = 1.0, max_goal_age: float = 3.0,
            limit: Optional[int] = None) -> dict:
    """Write frame_NNNN.npz files to `out` and return what was kept and skipped."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = os.path.normpath(bag)
    meta = bag_metadata(bag)
    os.makedirs(out, exist_ok=True)
    if glob.glob(os.path.join(out, 'frame_*.npz')):
        raise ValueError(f'{out} already holds frame_*.npz; the benchmark would mix them '
                         'with this bag. Use an empty --out.')
    bag_name = os.path.basename(bag)
    if os.path.isfile(bag):  # a lone .db3/.mcap
        bag_name = os.path.splitext(bag_name)[0]

    goals, ghosts = read_goals(bag, meta)
    goal_times = [g[0] for g in goals]
    every_ns = int(round(every * 1e9))
    max_age_ns = int(round(max_goal_age * 1e9))
    occupancy_grid = get_message(COSTMAP_TYPE)
    stats = {'costmaps': 0, 'written': 0, 'no_goal': 0, 'stale_goal': 0, 'throttled': 0,
             'malformed': 0, 'ghost_goals': ghosts, 'limited': False}

    reader = open_reader(bag, meta, COSTMAP_TOPIC)
    last_ns = None
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        stats['costmaps'] += 1
        k = bisect.bisect_right(goal_times, t_ns) - 1  # latest goal at or before t_ns
        if k < 0:
            stats['no_goal'] += 1
            continue
        goal_ns, goal_x, goal_y = goals[k]
        if t_ns - goal_ns > max_age_ns:
            stats['stale_goal'] += 1
            continue
        if last_ns is not None and t_ns - last_ns < every_ns:
            stats['throttled'] += 1
            continue

        # Only kept frames are deserialized: a 300x300 grid at 20 Hz adds up
        msg = deserialize_message(data, occupancy_grid)
        h, w = msg.info.height, msg.info.width
        if h == 0 or w == 0 or len(msg.data) != h * w:
            stats['malformed'] += 1  # the node rejects these too
            continue
        n = stats['written']
        write_frame(os.path.join(out, f'frame_{n:04d}.npz'),
                    costmap=np.asarray(msg.data, dtype=np.int8).reshape(h, w),
                    resolution=np.float64(msg.info.resolution),
                    goal_x=np.float64(goal_x), goal_y=np.float64(goal_y),
                    name=np.str_(f'{bag_name}_{n:04d}'),
                    bag_time_ns=np.int64(t_ns),
                    goal_age_s=np.float64((t_ns - goal_ns) * 1e-9),
                    origin_x=np.float64(msg.info.origin.position.x),
                    origin_y=np.float64(msg.info.origin.position.y))
        stats['written'] += 1
        last_ns = t_ns
        if limit is not None and stats['written'] >= limit:
            stats['limited'] = reader.has_next()
            break
    return stats


def summary(stats: dict, out: str, every: float, max_goal_age: float) -> str:
    line = (f"Wrote {stats['written']} frames to {out} from {stats['costmaps']} "
            f"{COSTMAP_TOPIC} messages; skipped {stats['no_goal']} with no goal yet, "
            f"{stats['stale_goal']} with a goal older than {max_goal_age:g} s, "
            f"{stats['throttled']} within {every:g} s of the last frame, "
            f"{stats['malformed']} malformed; ignored {stats['ghost_goals']} ghost goals")
    return line + ('; stopped at --limit' if stats['limited'] else '')


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', metavar='BAG_DIR', help='rosbag2 directory (or a lone .db3/.mcap)')
    ap.add_argument('--out', required=True, help='directory for frame_NNNN.npz')
    ap.add_argument('--every', type=float, default=1.0,
                    help='keep at most one frame per this many seconds (default 1.0)')
    ap.add_argument('--max-goal-age', type=float, default=3.0,
                    help='skip costmaps whose latest goal is older than this (default 3.0)')
    ap.add_argument('--limit', type=int, default=None, help='stop after N frames')
    args = ap.parse_args(argv)
    for flag, value in (('--every', args.every), ('--max-goal-age', args.max_goal_age)):
        if not (math.isfinite(value) and value >= 0):
            ap.error(f'{flag} must be a number >= 0')
    if args.limit is not None and args.limit < 1:
        ap.error('--limit must be at least 1')
    return args


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        stats = extract(args.bag, args.out, args.every, args.max_goal_age, args.limit)
    except (ValueError, RuntimeError) as e:  # rosbag2 errors arrive as RuntimeError
        sys.exit(f'extract_costmaps: {e}')
    print(summary(stats, args.out, args.every, args.max_goal_age))
    return 0 if stats['written'] else 1


if __name__ == '__main__':
    sys.exit(main())
