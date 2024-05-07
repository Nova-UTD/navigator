'''
Package:   astar_path_planner
Filename:  astar_path_planner_node.py
Author:    Justin Ruths

Plans a path using A*, specifically the SMAC Hybrid Path Planner

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

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder

from ros2_numpy.occupancy_grid import occupancygrid_to_numpy, numpy_to_occupancy_grid
from tf_transformations import quaternion_from_euler

from visualization_msgs.msg import Marker

from skimage.draw import line

from PIL import Image

from matplotlib.patches import Rectangle


class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner_node')

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

    # generate the path using Nav2
    def generate_path(self):

        if self.costmap is None:
            self.get_logger().warning("Have not received costmap in path planner yet...")
            return
        if self.path_goal is None:
            self.get_logger().warning("Have not received goal for path in path planner yet...")
            return
        if np.sqrt(self.path_goal.pose.position.x**2 + self.path_goal.pose.position.y**2) < 0.5:
            self.get_logger().warning("Vehicle has reached the goal (%1.2f,%1.2f)!" % (self.path_goal.pose.position.x,self.path_goal.pose.position.y))
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
        # pathfinding library expects obstacles as 0, so just shfit everything by a small fractional value:
        # cmap = cmap/10
        cmap += 1

        # anything bigger than a threshold we zero out to make it an obstacle
        obstacle_threshold = 90
        obstacle_idxs = np.asarray(cmap>=obstacle_threshold).nonzero()
        cmap[obstacle_idxs] = cmap[obstacle_idxs]+800

        f = open('costmap_dump.csv','w')
        for k in range(len(cmap)):
            f.write( '\t'.join([str(elem) for elem in cmap[k,:]])+'\n')
        f.close()

        # self.get_logger().info('================================================')
        # for k in range(len(cmap)):
        #     self.get_logger().info(str(list(cmap[k])))
        # self.get_logger().info('================================================')

        # Pass to Pathfinding libarary:
        grid = Grid(matrix=cmap)
        grid.cleanup()
        start = grid.node(i0, j0)
        end = grid.node(i, j)

        # finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        finder = DijkstraFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)

        if path is None or len(path)==0:
            self.get_logger().warning("!!! Pathfinding returned a path as None !!!")
            self.get_logger().debug("No path for (%i,%i)-->(%i,%i)" % (i0,j0,i,j))    
            return
        path_cost = sum([ grid.calc_cost(path[i], path[i+1], weighted=True) for i in range(len(path)-1)])
        self.get_logger().info("Path (%i,%i)-->(%i,%i) Returned with %i elements, cost=%i, runs=%i" % (i0,j0,i,j,len(path),path_cost,runs))

        path_base_link = Path()
        path_base_link.header.stamp = self.clock.clock
        path_base_link.header.frame_id = 'base_link'
        path_base_link.poses = []
        for i in range(len(path)):
            p = PoseStamped()
            p.header.stamp = self.clock.clock
            p.header.frame_id = 'base_link'
            p.pose.position.x = path[i].y*grid_res - origin_x
            p.pose.position.y = path[i].x*grid_res - origin_y
            path_base_link.poses.append(p)

        self.path_pub.publish(path_base_link)

        # TODO: Revist this status stuff once 
        # status.level = DiagnosticStatus.ERROR
        # status.message = "Could not find viable path. Likely too far off course."
        # self.get_logger().error("Could not find viable path")
        # self.status_pub.publish(status)

        # self.status_pub.publish(status)
        # self.last_status_time = time.time()

        # except(Exception) as e:
        #     self.get_logger().info('Issue generating path...')
        #     self.get_logger().info(str(e))
        #     self.get_logger().info('line number: %s' % str(e.__traceback__.tb_lineno))
        #     return

    # def get_transform(self, source_frame, target_frame):
    #     ''' Lookup latest transform between source_frame and target_frame from the buffer '''
    #     try:
    #         trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), rclpy.time.Duration(seconds=5.0) )

    #     except(LookupException, ConnectivityException, ExtrapolationException) as e:
    #         raise Exception('Cannot find transformation from %s to %s' % (source_frame,target_frame))
    #         #self.get_logger().warning('Cannot find transformation from %s to %s' % (source_frame,target_frame))
    #     return trans     # Type: TransformStamped

    # def pose_transform(self, pose, target_frame):
    #     ''' pose: will be transformed to target_frame '''
    #     trans = self.get_transform( pose.header.frame_id, target_frame )

    #     pose2 = PoseStamped()
    #     pose2.header.frame_id = target_frame
    #     pose2.header.stamp = pose.header.stamp
    #     pose2.pose = do_transform_pose(pose.pose, trans)
        
    #     return pose2

    # def path_transform(self, path, target_frame):
    #     ''' path: will be transformed to target_frame '''
    #     trans = self.get_transform( path.header.frame_id, target_frame )

    #     path2 = Path()
    #     path2.header.frame_id = target_frame
    #     path2.header.stamp = path.header.stamp

    #     for pose in path.poses:
    #         pose2 = PoseStamped()
    #         pose2.header.frame_id = target_frame
    #         pose2.header.stamp = pose.header.stamp
    #         pose2.pose = do_transform_pose(pose.pose, trans)
    #         path2.poses.append( pose2 )
    #     return path2

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
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
