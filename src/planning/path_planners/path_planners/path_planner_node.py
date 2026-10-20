#!/usr/bin/env python3
"""
Package:   path_planners
Filename:  path_planner_node.py
Author:    Bennett Daniel

Unified path planner node that incorporates multiple planning algorithms:
- ARA* path planner
- Dijkstra path planner
- Dynamic Programming path planner
- Neural Network path planner (pth model file is found in Nova Repository: https://github.com/Nova-UTD/neural_network_path_planner_training)
- TRRT* path planner

Subscribes to:
✅ /grid/steering_cost (OccupancyGrid)
✅ /planning/path_goal (PoseStamped)
✅ /clock (Clock)

Publishes:
✅ /planning/path (Path) in base_link frame

Choose the planner with the `planner` parameter (launch argument `planner:=<name>`).
Valid names live in planner_registry.PLANNER_NAMES; an unknown name stops the node.
"""

import math
import threading
import numpy as np
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# Planner selection and the costmap pipeline are shared with the offline benchmark
from path_planners.planner_registry import DEFAULT_PLANNER, UnknownPlannerError, make_planner
from path_planners.planning_core import (
    OBSTACLE_PADDING, OBSTACLE_THRESHOLD, ORIGIN_X, ORIGIN_Y,
    build_problem, costmap_to_array, rolling_smoothing)

def spline_smoothing(path, s=10):
    k = np.arange(len(path))
    x = np.array([p[0] for p in path])
    y = np.array([p[1] for p in path])

    tck_x = splrep(k, x, s=s)
    tck_y = splrep(k, y, s=s)
    xnew = BSpline(*tck_x)(k)
    ynew = BSpline(*tck_y)(k)
    return [ (xnew[i],ynew[i]) for i in range(len(path)) ]

# see: https://www.cs.unc.edu/~dm/UNC/COMP258/LECTURES/Chaikins-Algorithm.pdf
def chaikin_smoothing(path, depth=2):
    
    for d in range(depth):
        new_path = [ path[0] ] # keep the first point in the path
        for i in range(1,len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]

            # interpolate 1/4 and 3/4 of the way along the segement
            new_path.append( (path[i-1][0] + 0.25*dx, path[i-1][1] + 0.25*dy) )
            new_path.append( (path[i-1][0] + 0.75*dx, path[i-1][1] + 0.75*dy) )
        new_path.append( path[-1] ) # keep the last point in the path
        path = new_path
    
    return path

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        # Parameters
        self.grid_res = 0.4  # Grid resolution in meters/cell
        self.origin_x = ORIGIN_X  # Origin offset in X
        self.origin_y = ORIGIN_Y  # Origin offset in Y
        self.obstacle_threshold = OBSTACLE_THRESHOLD  # Values above this are considered obstacles
        self.obstacle_padding = OBSTACLE_PADDING  # Cells to pad around obstacles (5 = 1.0m, ~vehicle half-width)

        # Path re-use: cache the last valid Dijkstra result.
        # Replan only when an obstacle blocks the cached path or the goal moves.
        self._cached_path_cells = None
        self._cached_goal = None

        # Thread safety: costmap callback and generate_path run on different threads
        self._costmap_lock = threading.Lock()

        # Ghost-rejection: track the best (most recent) path_goal stamp seen
        self._best_goal_stamp = 0.0
        
        # Path smoothing parameters
        self.smoothing_look_ahead = 2
        self.smoothing_depth = 3

        # Parameters
        self.declare_parameter('planner', DEFAULT_PLANNER)
        self.planner_type_obj = self.get_parameter('planner')
        self.planner_type = self.planner_type_obj.value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 1
        )
        self.clock = Clock()

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, "/grid/steering_cost", self.costmap_callback, 1
        )
        self.costmap = None

        self.path_goal_sub = self.create_subscription(
            PoseStamped, "/planning/path_goal", self.path_goal_callback, 1
        )
        self.path_goal = None

        # Publishers
        self.path_pub = self.create_publisher(Path, "/planning/path", 1)
        self.debug_marker_pub = self.create_publisher(
            Marker, "/planning/debug_markers", 1
        )

        # Planning timer
        self.path_timer = self.create_timer(
            0.1, self.generate_path, callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Initialize the selected path planner; fail loudly on a typo
        try:
            self.planner = make_planner(self.planner_type)
        except UnknownPlannerError as e:
            self.get_logger().fatal(str(e))
            raise

        self.get_logger().info(
            f"Path Planner Node initialized using {self.planner.impl.__class__.__name__} "
            f"(planner:={self.planner.name})")

    def clock_callback(self, msg: Clock):
        self.clock = msg

    def costmap_callback(self, msg: OccupancyGrid):
        if msg.info.height == 0 or msg.info.width == 0:
            self.get_logger().warning("Incoming cost map dimensions were zero.")
            return
        # Guard against race where generate_path reads costmap mid-update
        with self._costmap_lock:
            self.costmap = msg

    def path_goal_callback(self, msg: PoseStamped):
        # Ghost rejection — two layers:
        # 1. Monotonic: never accept a goal older than the best seen so far.
        #    Once we have a fresh goal from the live publisher, all ghost
        #    retransmissions (which carry an older stamp) are silently dropped.
        # 2. Clock-relative: drop anything more than 3 s behind sim time.
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Monotonic check
        if msg_sec < self._best_goal_stamp:
            return
        # Clock-relative check (only once sim clock is valid)
        clk_sec = self.clock.clock.sec + self.clock.clock.nanosec * 1e-9
        if clk_sec > 0 and (clk_sec - msg_sec) > 3.0:
            return
        self._best_goal_stamp = msg_sec
        self.path_goal = msg

    def generate_path(self):
        # Snapshot costmap under lock to prevent race with callback thread
        with self._costmap_lock:
            costmap_snap = self.costmap

        if costmap_snap is None:
            self.get_logger().warning("Have not received costmap yet...")
            return
        if self.path_goal is None:
            self.get_logger().warning("Have not received goal for path yet...")
            return

        # Use-time ghost check: if cached goal is now stale vs sim clock, clear it
        clk_sec  = self.clock.clock.sec  + self.clock.clock.nanosec  * 1e-9
        goal_sec = self.path_goal.header.stamp.sec + self.path_goal.header.stamp.nanosec * 1e-9
        if clk_sec > 0 and (clk_sec - goal_sec) > 3.0:
            self.get_logger().warning(
                "Stale path_goal cleared (ghost?)", throttle_duration_sec=5.0)
            self.path_goal = None
            return

        resolution = costmap_snap.info.resolution
        raw_height = costmap_snap.info.height
        raw_width  = costmap_snap.info.width

        if len(costmap_snap.data) != raw_height * raw_width:
            self.get_logger().error("Costmap size mismatch.")
            return

        # ----------------------------
        # Scale to 60m x 60m, place start and goal, pad obstacles, and snap
        # a goal that landed in an obstacle cell (e.g. grid edge dilation or
        # boundary) back along the line toward start. Shared with the offline
        # benchmark: see planning_core.build_problem.
        # ----------------------------
        costmap_np = costmap_to_array(costmap_snap.data, raw_height, raw_width, resolution)
        problem = build_problem(
            costmap_np,
            resolution,
            self.path_goal.pose.position.x,
            self.path_goal.pose.position.y,
            self.origin_x,
            self.origin_y,
            self.obstacle_threshold,
            self.obstacle_padding
        )
        if problem is None:
            self.get_logger().warning(
                'No free cell found along start→goal line; path will be empty.',
                throttle_duration_sec=2.0)
            return
        if problem.goal_snapped:
            self.get_logger().info(
                f'Goal snapped to nearest free cell: {problem.goal}',
                throttle_duration_sec=2.0)

        # ----------------------------
        # Run Planner
        # ----------------------------
        path = self.planner(
            problem.costmap,
            problem.start,
            problem.goal,
            self.obstacle_threshold
        )

        if path is None or len(path) == 0:
            self.get_logger().warning("!!! Pathfinding returned a path as None !!!",
                                      throttle_duration_sec=2.0)
            return

        # ----------------------------
        # Smooth Path
        # ----------------------------
        path = rolling_smoothing(
            path,
            look_ahead=self.smoothing_look_ahead,
            depth=self.smoothing_depth
        )

        # ----------------------------
        # Convert to ROS Path
        # ----------------------------
        path_msg = Path()
        path_msg.header.stamp = self.clock.clock
        path_msg.header.frame_id = "base_link"

        for node in path:
            p = PoseStamped()
            p.header.stamp = self.clock.clock
            p.header.frame_id = "base_link"

            p.pose.position.x = node[1] * resolution - self.origin_x
            p.pose.position.y = node[0] * resolution - self.origin_y
            p.pose.position.z = 0.0

            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()