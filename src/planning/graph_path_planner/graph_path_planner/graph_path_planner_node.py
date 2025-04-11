'''
Package:   graph_path_planner
Filename:  graph_path_planner_node.py
Author:    Justin Ruths

Plans a path using the shortest (weight) path on a graph

Subscribes to:
✅ /grid/cost (OccupancyGrid)
✅ /gnss/odometry_processed (Odometry)

Publishes:
✅ /planning/path (Path) in map frame

Minimum update rate: 2 Hz, ideally 5 Hz
'''

from matplotlib import pyplot as plt
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from navigator_msgs.msg import Mode
import numpy as np
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from dataclasses import dataclass
import random
import time
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, TransformStamped
from navigator_msgs.msg import VehicleControl, VehicleSpeed
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Imu

import networkx as nx
from scipy.interpolate import splrep, BSpline

from ros2_numpy.occupancy_grid import occupancygrid_to_numpy, numpy_to_occupancy_grid
from tf_transformations import quaternion_from_euler

from visualization_msgs.msg import Marker

from skimage.draw import line

import scipy.ndimage
from PIL import Image

from matplotlib.patches import Rectangle


class GraphPathPlanner(Node):
    def __init__(self):
        super().__init__('graph_path_planner_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_broadcast_time = None

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)
        self.clock = Clock()

        steering_cost_map_sub = self.create_subscription(
            OccupancyGrid, '/grid/steering_cost', self.costMapCb, 1)
        self.costmap = None

        # TODO: Currently not doing anything with the speedCostMap - not sure if we need to keep?
        # speed_cost_map_sub = self.create_subscription(
        #     OccupancyGrid, '/grid/speed_cost', self.speedCostMapCb, 1)

        # self.status_pub = self.create_publisher(
        #     DiagnosticStatus, '/node_statuses', 1)

        path_goal_sub = self.create_subscription(
            PoseStamped, '/planning/path_goal', self.pathGoalCb, 1)
        self.path_goal = None

        self.path_pub = self.create_publisher(
            Path, '/planning/path', 1)

        self.path_timer = self.create_timer(0.1, self.generate_path, callback_group=MutuallyExclusiveCallbackGroup())

        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/planning/look_ahead_marker', 1)

    def speedCb(self, msg: VehicleSpeed):
        self.speed = msg.speed

    # def speedCostMapCb(self, msg: OccupancyGrid):
    #     if msg.info.height == 0:
    #         self.speed_costmap = np.asarray(msg.data, dtype=np.int8).reshape(
    #             int(np.sqrt(len(msg.data))), -1)

    #     else:
    #         self.speed_costmap = np.asarray(msg.data, dtype=np.int8).reshape(
    #             msg.info.height, msg.info.width)

    def clockCb(self, msg: Clock):
        self.clock = msg

    # saves the point we want to navigate to (a point along the route)
    def pathGoalCb(self, msg: PoseStamped):
        self.path_goal = msg

    # saves the new costmap for planning
    def costMapCb(self, msg: OccupancyGrid):
        if msg.info.height == 0 or msg.info.width == 0:
            self.get_logger().warning("Incoming cost map dimensions were zero.")
            return
        self.costmap = msg

    # any cells above the threshold get expanded by padding number of cells
    def pad_obstacles(self,cmap,threshold,padding):
        obstacles = 0*cmap
        # finds the location of obstacles as binary values
        obstacles[np.where(cmap>=threshold)] = 1 
        # obstacle_frac = 100.0*np.count_nonzero(obstacles)/len(obstacles)**2
        # self.get_logger().info(">>>>> %1.2f percent obstacle points BEFORE" % obstacle_frac)
        # expands the obstacles using a given structure, repeating this padding number of times
        struct1 = scipy.ndimage.generate_binary_structure(2, 1)  # 3x3 grid true in + shape
        struct2 = scipy.ndimage.generate_binary_structure(2, 2)  # 3x3 grid all true
        obstacles = scipy.ndimage.binary_dilation(obstacles, structure=struct2,iterations=padding).astype(cmap.dtype)
        obstacles = 100*obstacles # rescale up to 0-100
        # obstacle_frac = 100.0*np.count_nonzero(obstacles)/len(obstacles)**2
        # self.get_logger().info(">>>>> %1.2f percent obstacle points AFTER" % obstacle_frac)
        return np.maximum(cmap,obstacles)

    # generate the path
    def generate_path(self):

        if self.costmap is None:
            self.get_logger().warning("Have not received costmap in path planner yet...")
            return
        if self.path_goal is None:
            self.get_logger().warning("Have not received goal for path in path planner yet...")
            return
        if np.sqrt(self.path_goal.pose.position.x**2 + self.path_goal.pose.position.y**2) < 0.5:
            self.get_logger().info("Vehicle has reached the goal (%1.2f,%1.2f)!" % (self.path_goal.pose.position.x,self.path_goal.pose.position.y))
            return

        grid_res = 0.4
        origin_x = 20.0
        origin_y = 30.0

        # conversion from x,y in base_link to grid indices 
        # TODO: Avoid hard coding this
        i0 = int(round(origin_y/grid_res))
        j0 = int(round(origin_x/grid_res))
        i = int(round((self.path_goal.pose.position.y+origin_y)/grid_res))
        j = int(round((self.path_goal.pose.position.x+origin_x)/grid_res))

        # costmap for planning
        cmap = np.asarray(self.costmap.data, dtype=np.int32).reshape(
                    self.costmap.info.height, self.costmap.info.width)
        
        cmap = self.pad_obstacles(cmap,85,3)

        path = self.shortest_path(cmap, (i0,j0), (i,j), obstacle_threshold=95)

        if path is None or len(path)==0:
            self.get_logger().warning("!!! Pathfinding returned a path as None !!!")
            self.get_logger().debug("No path for (%i,%i)-->(%i,%i)" % (i0,j0,i,j))    
            return

        #path = self.smooth_path(path,2)  # didn't work great
        #path = self.chaikin_smoothing(path,depth=5)   # didn't work well
        path = self.rolling_smoothing(path, look_ahead=2, depth=3)
        
        self.get_logger().debug("Path (%i,%i)-->(%i,%i) Returned with %i elements" % (i0,j0,i,j,len(path)))

        path_base_link = Path()
        path_base_link.header.stamp = self.clock.clock
        path_base_link.header.frame_id = 'base_link'
        path_base_link.poses = []
        for i in range(len(path)):
            p = PoseStamped()
            p.header.stamp = self.clock.clock
            p.header.frame_id = 'base_link'
            p.pose.position.x = path[i][1]*grid_res - origin_x
            p.pose.position.y = path[i][0]*grid_res - origin_y
            path_base_link.poses.append(p)

        self.path_pub.publish(path_base_link)

        # TODO: Revist this status stuff once 
        # status.level = DiagnosticStatus.ERROR
        # status.message = "Could not find viable path. Likely too far off course."
        # self.get_logger().error("Could not find viable path")
        # self.status_pub.publish(status)

        # self.status_pub.publish(status)
        # self.last_status_time = time.time()

    def shortest_path(self, costmap, start, end, obstacle_threshold=90):

        if costmap[start] > obstacle_threshold or costmap[end] > obstacle_threshold:
            self.get_logger().warn('Start or goal point are infeasible because they are in a high cost region (obstacle).')
            return None

        G = self.costmap_adjacency(costmap,obstacle_threshold)
        try:
            path = nx.dijkstra_path(G, start, end, 'weight')
            return path
        except:
            return None

    def costmap_adjacency(self, costmap, obstacle_threshold):
        neighbors = [(-1,-1), (-1,0), (-1,1), (0,-1), (0, 1), (1,-1), (1,0), (1,1)]

        # create graph and add nodes (nodes are costmap grid cells)
        G = nx.DiGraph()
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                if costmap[i,j] <= obstacle_threshold:
                    G.add_node( (i,j) )
        
        # add edges from adjacent costmap cells
        for i,j in G.nodes:
            for di, dj in neighbors:
                if (i+di,j+dj) in G:
                    G.add_edge( (i+di,j+dj) , (i,j) , weight=costmap[i,j] )
        
        return G

    def smooth_path(self,path, s=10):
        k = np.arange(len(path))
        x = np.array([p[0] for p in path])
        y = np.array([p[1] for p in path])

        tck_x = splrep(k, x, s=s)
        tck_y = splrep(k, y, s=s)
        xnew = BSpline(*tck_x)(k)
        ynew = BSpline(*tck_y)(k)
        return [ (xnew[i],ynew[i]) for i in range(len(path)) ]
    
    # see: https://www.cs.unc.edu/~dm/UNC/COMP258/LECTURES/Chaikins-Algorithm.pdf
    def chaikin_smoothing(self, path, depth=2):
        
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

    def rolling_smoothing(self, path, look_ahead=2, depth=3):
        if len(path) < look_ahead:
            return path

        t = 1.0/look_ahead
        for d in range(depth):
            new_path = [ path[0] ]
            for i in range(1,len(path)-look_ahead):
                x0 = path[i-1][0]
                y0 = path[i-1][1]
                
                dx = path[i-1+look_ahead][0] - x0
                dy = path[i-1+look_ahead][1] - y0
                new_path.append( (x0 + t*dx, y0 + t*dy) )
            new_path.extend( path[-look_ahead:] )
            path = new_path
        
        return path

def main(args=None):
    rclpy.init(args=args)
    node = GraphPathPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
