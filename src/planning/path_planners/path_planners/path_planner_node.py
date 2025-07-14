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
"""

import math
import numpy as np
import time
from typing import List, Tuple, Optional
import scipy.ndimage

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

# Import all path planner classes
from path_planners.arastar_path_planner import ARAStarPlanner
from path_planners.dijkstra_path_planner import DijkstraPathPlanner
from path_planners.dp_path_planner import DPPathPlanner
from path_planners.neural_network_path_planner import NeuralPathPlanner
from path_planners.trrtstar_path_planner import TRRTStarPathPlanner

def pad_obstacles(cmap, threshold, padding):
    """Apply padding to obstacles in the costmap."""
    obstacles = 0 * cmap
    # Find the location of obstacles as binary values
    obstacles[np.where(cmap >= threshold)] = 1
    # Expand obstacles using binary dilation
    #struct1 = scipy.ndimage.generate_binary_structure(2, 1)  # 3x3 grid true in + shape
    struct2 = scipy.ndimage.generate_binary_structure(2, 2)  # 3x3 grid all true
    obstacles = scipy.ndimage.binary_dilation(obstacles, structure=struct2, iterations=padding).astype(cmap.dtype)
    
    # Create padded costmap - set obstacle areas to 100 (white)
    padded_costmap = cmap.copy()
    padded_costmap[obstacles > 0] = 100
    
    return padded_costmap

def rolling_smoothing(path, look_ahead=2, depth=3):
    """
    Smooths a path by moving each point toward a future point.
    
    Parameters:
    - path: List of (row, col) tuples representing the path
    - look_ahead: How many points ahead to look
    - depth: Number of smoothing iterations
    
    Returns:
    - Smoothed path as list of (row, col) tuples
    """
    if len(path) < look_ahead:
        return path

    t = 1.0/look_ahead
    for d in range(depth):
        new_path = [path[0]]
        for i in range(1, len(path)-look_ahead):
            x0 = path[i-1][0]
            y0 = path[i-1][1]
            
            dx = path[i-1+look_ahead][0] - x0
            dy = path[i-1+look_ahead][1] - y0
            new_path.append((x0 + t*dx, y0 + t*dy))
        new_path.extend(path[-look_ahead:])
        path = new_path
    
    return path

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
        self.origin_x = 20.0  # Origin offset in X
        self.origin_y = 30.0  # Origin offset in Y
        self.obstacle_threshold = 90  # Values above this are considered obstacles
        self.obstacle_padding = 3  # Cells to pad around obstacles
        
        # Path smoothing parameters
        self.smoothing_look_ahead = 2
        self.smoothing_depth = 3

        # Parameters
        self.declare_parameter('planner', "dijkstra")
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

        # Initialize path planners
        if self.planner_type == 'dijkstra':
            self.planner = DijkstraPathPlanner()
        elif self.planner_type == 'ara-star':
            self.planner = ARAStarPlanner()
        elif self.planner_type == 'dynamic-programming':
            self.planner = DPPathPlanner()
        elif self.planner_type == 'neural-network':
            self.planner = NeuralPathPlanner()
        elif self.planner_type == 'trrt-star':
            self.planner = TRRTStarPathPlanner()

        self.get_logger().info(f"Path Planner Node initialized using {self.planner.__class__.__name__}")

    def clock_callback(self, msg: Clock):
        self.clock = msg

    def costmap_callback(self, msg: OccupancyGrid):
        if msg.info.height == 0 or msg.info.width == 0:
            self.get_logger().warning("Incoming cost map dimensions were zero.")
            return
        self.costmap = msg
        self.get_logger().debug(f"Received costmap: {msg.info.width}x{msg.info.height}")

    def path_goal_callback(self, msg: PoseStamped):
        self.path_goal = msg
        self.get_logger().debug(
            f"Received goal: ({msg.pose.position.x}, {msg.pose.position.y})"
        )

    def generate_path(self):
        """Main function to generate the path using the selected planner."""
        if self.costmap is None:
            self.get_logger().warning("Have not received costmap yet...")
            return
        if self.path_goal is None:
            self.get_logger().warning("Have not received goal for path yet...")
            return

        # Check if we're already at the goal
        if (
            np.sqrt(
                self.path_goal.pose.position.x**2 + self.path_goal.pose.position.y**2
            )
            < 0.5
        ):
            self.get_logger().info(
                f"Vehicle has reached the goal ({self.path_goal.pose.position.x:.2f},{self.path_goal.pose.position.y:.2f})!"
            )
            return

        # Convert from ROS coordinates to grid indices
        start_i = int(round(self.origin_y / self.grid_res))
        start_j = int(round(self.origin_x / self.grid_res))

        goal_i = int(
            round((self.path_goal.pose.position.y + self.origin_y) / self.grid_res)
        )
        goal_j = int(
            round((self.path_goal.pose.position.x + self.origin_x) / self.grid_res)
        )

        # Prepare costmap data
        costmap_np = np.asarray(self.costmap.data, dtype=np.int32).reshape(
            self.costmap.info.height, self.costmap.info.width
        )

        # Pad obstacles for safety
        padded_costmap = pad_obstacles(costmap_np, self.obstacle_threshold, self.obstacle_padding)

        # Plan path using the selected planner
        path = None
        
        # Different planners have slightly different interfaces, handle each case
        if isinstance(self.planner, ARAStarPlanner):
            self.planner.s_start = (start_i, start_j)
            self.planner.s_goal = (goal_i, goal_j)
            self.planner.cmap = padded_costmap
            self.planner.obstacle_threshold = self.obstacle_threshold
            self.planner.create_graph_from_costmap()
            path = self.planner.arastar()
            
        elif isinstance(self.planner, DijkstraPathPlanner):
            path = self.planner.shortest_path(padded_costmap, (start_i, start_j), (goal_i, goal_j), self.obstacle_threshold)
            
        elif isinstance(self.planner, DPPathPlanner):
            # For DP planner, we need to set parameters and run value iteration
            self.planner.costmap_data = padded_costmap
            self.planner.obstacle_threshold = self.obstacle_threshold
            success = self.planner.run_value_iteration(start_i, start_j, goal_i, goal_j)
            if success:
                path = self.planner.extract_path((start_i, start_j), (goal_i, goal_j))
                
        elif isinstance(self.planner, NeuralPathPlanner):
            # Neural network planner has a different interface
            path = self.planner.predict_path(padded_costmap, (start_i, start_j), (goal_i, goal_j))
            
        elif isinstance(self.planner, TRRTStarPathPlanner):
            # TRRT* planner has a specific method
            path = self.planner.trrtstar_path(padded_costmap, (start_i, start_j), (goal_i, goal_j), self.obstacle_threshold)

        if path is None or len(path) == 0:
            self.get_logger().warning("!!! Pathfinding returned a path as None !!!")
            self.get_logger().debug(f"No path for ({start_i},{start_j})-->({goal_i},{goal_j})")
            return

        # Apply path smoothing
        path = rolling_smoothing(path, look_ahead=self.smoothing_look_ahead, depth=self.smoothing_depth)
        
        self.get_logger().debug(
            f"Path ({start_i},{start_j})-->({goal_i},{goal_j}) returned with {len(path)} elements"
        )

        # Convert path to ROS message
        path_msg = Path()
        path_msg.header.stamp = self.clock.clock
        path_msg.header.frame_id = "base_link"
        path_msg.poses = []

        for i in range(len(path)):
            p = PoseStamped()
            p.header.stamp = self.clock.clock
            p.header.frame_id = "base_link"

            # Convert grid coordinates back to base_link frame
            # Note: Different planners may return paths in different formats (i,j) vs (j,i)
            # Handle both cases by checking the path format
            if isinstance(path[i], tuple) and len(path[i]) == 2:
                # Most planners return (i, j) format
                p.pose.position.x = path[i][1] * self.grid_res - self.origin_x
                p.pose.position.y = path[i][0] * self.grid_res - self.origin_y
            else:
                # Some planners might return a different format
                p.pose.position.x = path[i][1] * self.grid_res - self.origin_x
                p.pose.position.y = path[i][0] * self.grid_res - self.origin_y
                
            p.pose.position.z = 0.0
            path_msg.poses.append(p)

        # Publish the path
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