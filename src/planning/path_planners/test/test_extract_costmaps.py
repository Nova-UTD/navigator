"""Tests for extract_costmaps: a real rosbag2 in, benchmark frames out."""

import array
import os
import subprocess
import sys

import numpy as np
from path_planners.benchmark import cli, extract_costmaps, scenarios
import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')
serialization = pytest.importorskip('rclpy.serialization')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
nav_msgs = pytest.importorskip('nav_msgs.msg')

T0_NS = 1_760_000_000 * 10 ** 9  # bag receive times are wall-clock nanoseconds

CLEAR = scenarios.Canvas().route(scenarios.straight_route()).grid
PARKED = scenarios.SYNTHETIC['parked_car']().costmap
SLALOM = scenarios.SYNTHETIC['slalom']().costmap
WIDE = np.full((150, 200), 50, dtype=np.int32)  # non-square, so a transposed reshape fails
WIDE[70:80, :] = 10
WIDE[:40, 120:125] = 100

G1 = (25.0, 0.0)
G2 = (30.0, 0.5)
GHOST = (-5.0, 12.0)
TOPICS = ((extract_costmaps.COSTMAP_TOPIC, extract_costmaps.COSTMAP_TYPE),
          (extract_costmaps.GOAL_TOPIC, extract_costmaps.GOAL_TYPE))


def _grid(values, resolution=0.2, data=None):
    msg = nav_msgs.OccupancyGrid()
    msg.header.frame_id = 'base_link'
    msg.info.height, msg.info.width = values.shape
    msg.info.resolution = resolution
    msg.info.origin.position.x = -20.0
    msg.info.origin.position.y = -30.0
    msg.data = array.array('b', values.astype(np.int8).tobytes()) if data is None else data
    return msg


def _goal(xy, stamp):
    msg = geometry_msgs.PoseStamped()
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    msg.pose.position.x, msg.pose.position.y = xy
    return msg


def _timeline():
    """(seconds after T0, topic, message), in write order."""
    cm, goal = extract_costmaps.COSTMAP_TOPIC, extract_costmaps.GOAL_TOPIC
    return [
        (0.0, cm, _grid(CLEAR)),                 # before any goal: no goal
        (0.5, cm, _grid(PARKED)),                # same receive time as G1: frame 0
        (0.5, goal, _goal(G1, 100.5)),
        (1.0, cm, _grid(CLEAR)),                 # 0.5 s after frame 0: throttled
        (1.5, cm, _grid(CLEAR)),                 # exactly --every later: frame 1
        (5.0, cm, _grid(CLEAR)),                 # G1 is 4.5 s old: stale
        (6.0, goal, _goal(G2, 106.0)),
        (6.2, goal, _goal(GHOST, 99.0)),         # stamped older than G2: the node drops it
        (6.5, cm, _grid(SLALOM)),                # paired with G2, not the ghost: frame 2
        (8.0, cm, _grid(CLEAR, data=array.array('b', bytes(100)))),  # malformed
        (8.5, cm, _grid(WIDE, resolution=0.4)),  # frame 3
        (12.0, cm, _grid(CLEAR)),                # G2 is 6 s old: stale
    ]


def write_bag(path, timeline, topics=TOPICS):
    """Record `timeline` into a sqlite3 rosbag2 at `path` with the real writer."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    for name, msg_type in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=msg_type,
                                                     serialization_format='cdr'))
    for t, topic, msg in timeline:
        writer.write(topic, serialization.serialize_message(msg), T0_NS + int(round(t * 1e9)))
    writer.close()  # writes metadata.yaml
    return path


@pytest.fixture
def bag(tmp_path):
    return write_bag(str(tmp_path / 'drive'), _timeline())


def _stats(**changes):
    stats = {'costmaps': 9, 'written': 4, 'no_goal': 1, 'stale_goal': 2, 'throttled': 1,
             'malformed': 1, 'ghost_goals': 1, 'limited': False}
    stats.update(changes)
    return stats


def test_frames_load_through_the_benchmark(bag, tmp_path):
    out = str(tmp_path / 'frames')
    assert extract_costmaps.extract(bag, out) == _stats()
    assert sorted(os.listdir(out)) == [f'frame_{k:04d}.npz' for k in range(4)]

    loaded = scenarios.collect([], [out])
    # (grid, resolution, goal, bag seconds, goal age) of frames 0-3 in _timeline
    expected = [(PARKED, 0.2, G1, 0.5, 0.0), (CLEAR, 0.2, G1, 1.5, 1.0),
                (SLALOM, 0.2, G2, 6.5, 0.5), (WIDE, 0.4, G2, 8.5, 2.5)]
    assert len(loaded) == len(expected)
    for k, (sc, (grid, res, goal, t, age)) in enumerate(zip(loaded, expected)):
        assert sc.name == f'drive_{k:04d}'
        assert sc.costmap.shape == grid.shape
        np.testing.assert_array_equal(sc.costmap, grid)
        assert sc.resolution == float(np.float32(res))  # OccupancyGrid resolution is float32
        assert sc.goal_xy == goal
        with np.load(os.path.join(out, f'frame_{k:04d}.npz')) as f:
            assert float(f['goal_age_s']) == pytest.approx(age)
            assert int(f['bag_time_ns']) == T0_NS + int(round(t * 1e9))
        # The frame goes through the node's own pipeline and has a path
        prepared = cli.prepare(sc)
        assert prepared is not None and prepared.reachable


def test_every_limit_and_goal_age(bag, tmp_path):
    keep_all = extract_costmaps.extract(bag, str(tmp_path / 'a'), every=0.0, max_goal_age=10.0)
    assert keep_all == _stats(written=7, stale_goal=0, throttled=0)

    limited = extract_costmaps.extract(bag, str(tmp_path / 'b'), every=0.0, max_goal_age=10.0,
                                       limit=2)
    assert limited == _stats(costmaps=3, written=2, stale_goal=0, throttled=0, malformed=0,
                             limited=True)
    assert len(os.listdir(tmp_path / 'b')) == 2

    # A goal received at the same instant as a costmap counts as at-or-before it
    exact = extract_costmaps.extract(bag, str(tmp_path / 'c'), max_goal_age=0.0)
    assert exact == _stats(written=1, stale_goal=7, throttled=0, malformed=0)


def test_lone_db3_file(bag, tmp_path):
    db3 = os.path.join(bag, 'drive_0.db3')
    assert extract_costmaps.extract(db3, str(tmp_path / 'frames')) == _stats()
    assert scenarios.collect([], [str(tmp_path / 'frames')])[0].name == 'drive_0_0000'


def test_cli_summary_and_refusals(bag, tmp_path, capsys):
    out = str(tmp_path / 'frames')
    assert extract_costmaps.main([bag, '--out', out]) == 0
    line = [s for s in capsys.readouterr().out.splitlines() if s.startswith('Wrote')]
    assert line == [f'Wrote 4 frames to {out} from 9 /grid/steering_cost messages; skipped '
                    '1 with no goal yet, 2 with a goal older than 3 s, 1 within 1 s of the '
                    'last frame, 1 malformed; ignored 1 ghost goals']

    with pytest.raises(SystemExit, match='already holds frame_'):
        extract_costmaps.main([bag, '--out', out])
    with pytest.raises(SystemExit):
        extract_costmaps.main([bag, '--out', str(tmp_path / 'x'), '--every', 'nan'])

    no_goals = write_bag(str(tmp_path / 'no_goals'), _timeline()[:2],
                         TOPICS[:1])
    with pytest.raises(SystemExit, match='has no /planning/path_goal'):
        extract_costmaps.main([no_goals, '--out', str(tmp_path / 'y')])
    os.remove(os.path.join(bag, 'metadata.yaml'))
    with pytest.raises(SystemExit, match='ros2 bag reindex'):
        extract_costmaps.main([bag, '--out', str(tmp_path / 'z')])


def test_benchmark_import_pulls_in_no_ros():
    code = ('import sys\n'
            'from path_planners.benchmark import cli, extract_costmaps\n'
            "ros = [m for m in sys.modules if m.split('.')[0] in "
            "('rclpy', 'rosbag2_py', 'rosidl_runtime_py')]\n"
            'assert not ros, ros\n')
    subprocess.run([sys.executable, '-c', code], check=True)
