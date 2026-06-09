'''
Package: costs
   File: route_costmap_node.py
 Author: Justin Ruths

Subscribes to the global route and
publishes the local route as a cost map.
'''

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml

# Message definitions
from navigator_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Path
from navigator_msgs.msg import Egma
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import LookupException, ExtrapolationException, TransformException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ros2_numpy.geometry import quat_to_numpy, numpy_to_quat
from tf_transformations import quaternion_multiply, euler_from_quaternion
from ros2_numpy.occupancy_grid import occupancygrid_to_numpy, numpy_to_occupancy_grid
# to do image rotations/shifts for fast forwarding occupancy grids
from scipy import ndimage
import cv2

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from skimage.morphology import erosion

import matplotlib.pyplot as plt

class RouteCostmapNode(Node):

    def __init__(self):
        super().__init__('route_costmap_node')

        # Set up the global config file
        self.declare_parameter('global_config', 'temp_value')
        self.file_path = self.get_parameter('global_config').value

        # Subscriptions and publishers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.route_dist_grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/route_distance', 1)

        self.path_goal_pub = self.create_publisher(
            PoseStamped, '/planning/path_goal', 1)

        self.goal_marker_pub = self.create_publisher(
            Marker, '/planning/goal_marker', 1)

        route_sub = self.create_subscription(
            Path, '/planning/route', self.routeCb, 1)
        self.route = None
        self.not_visited = None

        # Subscribe to drivable grid so we can pick the furthest
        # visible/mapped goal point on the route.
        self.drivable_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self._drivable_cb, 1)
        self._drivable_grid: np.ndarray | None = None

        # TODO: implement status book keeping
        # self.status_pub = self.create_publisher(
        #     DiagnosticStatus, '/node_status', 1)
        # self.status = DiagnosticStatus()

        self.costmap_timer = self.create_timer(0.05, self.buildRouteCostmap, callback_group=MutuallyExclusiveCallbackGroup())

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    def _drivable_cb(self, msg: OccupancyGrid):
        """Cache the latest drivable grid for goal visibility checks."""
        if msg.info.height > 0 and msg.info.width > 0:
            self._drivable_grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)

    def _is_drivable(self, x_bl: float, y_bl: float) -> bool:
        """Return True if (x, y) in base_link maps to a drivable cell (<90) in
        the 300×300 perception/HD-map drivable grid (origin -20, -30, res 0.2)."""
        if self._drivable_grid is None:
            return True  # no data yet — optimistic
        col = int(round((x_bl + 20.0) / 0.2))
        row = int(round((y_bl + 30.0) / 0.2))
        if row < 0 or row >= self._drivable_grid.shape[0]:
            return False
        if col < 0 or col >= self._drivable_grid.shape[1]:
            return False
        return int(self._drivable_grid[row, col]) < 90

    # TODO: currently implemented, the route cannot be changed once it is first received
    def routeCb(self, msg: Path):
        if self.route is None:
            self.get_logger().info('Received the route.')
            self.route = msg.poses
            self.route_remaining = msg.poses

    def _load_config(self):
        """Load and cache the global parameters yaml once."""
        if hasattr(self, '_cfg'):
            return self._cfg
        try:
            with open(self.file_path, 'r') as f:
                self._cfg = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Cannot load config: {e}')
            self._cfg = None
        return self._cfg

    def buildRouteCostmap(self):
        # ── grid dimensions from config ──────────────────────────────────────
        cfg = self._load_config()
        if cfg is None:
            return

        resolution  = cfg['occupancy_grids']['resolution']            # 0.2 m
        grid_cols   = int(cfg['occupancy_grids']['width']   / resolution)  # 300
        grid_rows   = int(cfg['occupancy_grids']['length']  / resolution)  # 300
        veh_long    = cfg['occupancy_grids']['vehicle_longitudinal_location']  # 20 m
        veh_lat     = cfg['occupancy_grids']['vehicle_latitudinal_location']   # 30 m

        # Baseline: gray (50) — off-route areas have medium cost.
        # Route corridor will be painted white (0) below.
        # Canvas is now the SAME size as the published OccupancyGrid (300×300)
        # so no resize is needed and every coordinate maps exactly.
        routemap = np.full((grid_rows, grid_cols), 50.0)

        if self.route is None:
            self.get_logger().warning('Route Costmap Node has not received route yet.')
            self.publish(routemap, (0.0, 0.0), cfg)
            return

        try:
            # get the transform from map to base_link
            ego_tf = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=5.0))
            
            roll, pitch, yaw = euler_from_quaternion(quat_to_numpy(ego_tf.transform.rotation))

            xmax    =  cfg['occupancy_grids']['length'] - veh_long   # +40 m ahead
            xmin    = -veh_long                                        # -20 m behind
            ymin    = -veh_lat                                         # -30 m right
            ymax    =  veh_lat                                         # +30 m left
            gridres =  resolution

            # transform the route points to base_link 
            route_baselink_x = np.zeros(len(self.route_remaining))
            route_baselink_y = np.zeros(len(self.route_remaining))
            # dist_to_car = np.zeros(len(self.route_remaining))
            for i,pose in enumerate(self.route_remaining):
                x = pose.pose.position.x*np.cos(yaw) - pose.pose.position.y*np.sin(yaw) + ego_tf.transform.translation.x
                y = pose.pose.position.x*np.sin(yaw) + pose.pose.position.y*np.cos(yaw) + ego_tf.transform.translation.y
                # dist_to_car[i] = np.sqrt(x**2+y**2)

                route_baselink_x[i] = x
                route_baselink_y[i] = y

            # Anything behind the vehicle we say we have visited already
            keep_idxs = np.flatnonzero(np.array(route_baselink_x) > 0) 

            # none of the remaining route is ahead of the vehicle, so we are done
            if len(keep_idxs) == 0:
                self.get_logger().warning('Did not find any route points ahead of the vehicle.')
                self.route_remaining = []
                self.publish(routemap, (0.0, 0.0), cfg)
                return
            
            # the route may make some turns such that part of the future path goes behind the vehicle
            # so we keep everything starting with the first route point in front of the vehicle
            # select the one prior to this, so we can interpolate from that one to the one we care about
            start_idx = max(0,keep_idxs[0]-1) # pick the one prior to the first one

            # keep only the route starting from that index (adding in the origin, where the vehicle is)
            self.route_remaining = self.route_remaining[start_idx:]
            route_baselink_x = route_baselink_x[start_idx:]
            route_baselink_y = route_baselink_y[start_idx:]
            self.get_logger().debug('route has %i points' % len(route_baselink_x))

            # ------------------------------------------------------------------
            # Interpolate route into per-cell points (gridxs, gridys in m,
            # base_link).  Collect ALL points in the costmap.
            # ------------------------------------------------------------------
            gridxs = []
            gridys = []
            for idx in range(1, len(route_baselink_x)):
                dx = route_baselink_x[idx] - route_baselink_x[idx-1]
                dy = route_baselink_y[idx] - route_baselink_y[idx-1]
                steps = int(np.ceil(max(abs(dx), abs(dy)) / gridres))
                if steps == 0:
                    continue
                for t in np.linspace(0, 1, steps + 1)[1:]:
                    newx = route_baselink_x[idx-1] + t * dx
                    newy = route_baselink_y[idx-1] + t * dy
                    if self.is_within_costmap(newx, newy):
                        gridxs.append(newx)
                        gridys.append(newy)

            self.get_logger().debug('grid route has %i points' % len(gridxs))

            if len(gridxs) < 2:
                self.get_logger().info('You have reached the end of the route.')
                self.publish(routemap, (0.0, 0.0))
                return

            # ------------------------------------------------------------------
            # Paint the route corridor: white (0) with half-width = HALF_W cells.
            # Canvas is 300×300 so the coordinate formula maps exactly:
            #   row ci = (y_baselink + veh_lat)  / resolution   (0 = -30 m, 299 = +29.8 m)
            #   col cj = (x_baselink + veh_long) / resolution   (0 = -20 m, 299 = +39.8 m)
            # Vehicle sits at row=150, col=100 — matching the drivable/occupancy grids.
            # ------------------------------------------------------------------
            HALF_W = 6  # cells each side = 1.2 m corridor half-width at 0.2 m/cell
            for r in range(len(gridxs)):
                ci = int(round((gridys[r] + veh_lat)  / gridres))
                cj = int(round((gridxs[r] + veh_long) / gridres))
                for di in range(-HALF_W, HALF_W + 1):
                    for dj in range(-HALF_W, HALF_W + 1):
                        ni, nj = ci + di, cj + dj
                        if 0 <= ni < grid_rows and 0 <= nj < grid_cols:
                            routemap[ni, nj] = 0

            # ------------------------------------------------------------------
            # Goal selection: furthest point along the route that is within
            # the camera-mapped drivable area (drivable_grid cell < 90).
            # Walk from far end toward vehicle; first hit is the goal.
            # Fall back to last in-costmap point if no drivable cell found.
            # ------------------------------------------------------------------
            goal = (gridxs[-1], gridys[-1])  # fallback: furthest costmap point
            for r in range(len(gridxs) - 1, -1, -1):
                if self._is_drivable(gridxs[r], gridys[r]):
                    goal = (gridxs[r], gridys[r])
                    break

            self.get_logger().debug(
                'path goal (drivable-snapped): %.2f, %.2f' % goal)

            self.publish(routemap, goal, cfg)

        except(LookupException, ExtrapolationException, ConnectivityException) as e: # typically get some errors on startup as the tf buffer fills
            self.get_logger().warning("!!! Error finding transform to build route grid !!!")
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

    def publish(self, routemap, goal, cfg):
        # Publish path goal
        path_goal = PoseStamped()
        path_goal.header.stamp = self.clock.clock
        path_goal.header.frame_id = 'base_link'
        path_goal.pose.position.x = goal[0]
        path_goal.pose.position.y = goal[1]
        self.path_goal_pub.publish(path_goal)

        # Marker for RViz goal arrow
        self.publish_marker(path_goal, (0.0, 1.0, 0.4), self.goal_marker_pub)

        # Publish OccupancyGrid.
        # routemap is already 300×300 — no resize needed.
        resolution = cfg['occupancy_grids']['resolution']
        grid_cols  = int(cfg['occupancy_grids']['width']   / resolution)
        grid_rows  = int(cfg['occupancy_grids']['length']  / resolution)

        route_cost_msg = OccupancyGrid()
        route_cost_msg.info.map_load_time               = self.clock.clock
        route_cost_msg.info.resolution                   = resolution
        route_cost_msg.info.width                        = grid_cols
        route_cost_msg.info.height                       = grid_rows
        route_cost_msg.info.origin.position.x            = -cfg['occupancy_grids']['vehicle_longitudinal_location']
        route_cost_msg.info.origin.position.y            = -cfg['occupancy_grids']['vehicle_latitudinal_location']
        route_cost_msg.info.origin.orientation.w         = 1.0
        route_cost_msg.header.stamp                      = self.clock.clock
        route_cost_msg.header.frame_id                   = 'base_link'

        # Clamp and flatten — routemap shape matches (grid_rows, grid_cols)
        route_cost_msg.data = (
            np.clip(routemap, -128, 127).astype(np.int8).flatten().tolist()
        )

        self.route_dist_grid_pub.publish(route_cost_msg)

    def is_within_costmap(self, x, y):
        cfg  = self._load_config()
        if cfg is None:
            return False
        xmax =  cfg['occupancy_grids']['length'] - cfg['occupancy_grids']['vehicle_longitudinal_location']
        xmin = -cfg['occupancy_grids']['vehicle_longitudinal_location']
        ymin = -cfg['occupancy_grids']['vehicle_latitudinal_location']
        ymax =  cfg['occupancy_grids']['vehicle_latitudinal_location']
        return xmin <= x <= xmax and ymin <= y <= ymax

    # this creates a radial costmap centered on the goal waypoint
    # creates a gradual cost landscape to drive the path towards the end
    def make_waypoint_costmap(self,waypoint):
        # Open the config file
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
        except FileNotFoundError:
            print("Error: config.yaml not found.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")
        
        xmax = data['occupancy_grids']['length'] - data['occupancy_grids']['vehicle_longitudinal_location'] # 40m in front of the car
        xmin = -1 * data['occupancy_grids']['vehicle_longitudinal_location'] # 20m in back of the car
        ymin = -1 * data['occupancy_grids']['vehicle_latitudinal_location'] # 40m left of the car
        ymax = data['occupancy_grids']['vehicle_latitudinal_location'] # 40m right of the car
        gridres = data['occupancy_grids']['resolution']
        
        costmap = np.zeros((151,151))
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                x,y = gridres*j +xmin , gridres*i + ymin
                dist = np.sqrt( (x-waypoint.position.x)**2 + (y-waypoint.position.y)**2 )
                costmap[i,j] = 50.0 * dist / np.sqrt(2*60.0**2)

        return costmap

    def publish_marker(self, target, c, publisher):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.clock.clock
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 1.0
        marker.scale.z = 0.75

        color = ColorRGBA()
        color.a = 0.85
        color.r = c[0]
        color.g = c[1]
        color.b = c[2]
        marker.color = color

        pt_a = Point()
        marker.points.append(pt_a)

        pt_b = Point()
        pt_b.x = target.pose.position.x
        pt_b.y = target.pose.position.y
        pt_b.z = 0.3
        marker.points.append(pt_b)

        publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RouteCostmapNode()
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
