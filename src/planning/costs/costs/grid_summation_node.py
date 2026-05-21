'''
Package: grids
   File: grid_summation_node.py
 Author: Will Heitman (w at heit dot mn)

Subscribes to cost maps, calculates their weighted sum, and
publishes the result as a finished cost map.

[Nova UTD — laneControlledCostmap PR]
Added /grid/lane_control subscription and entry in createCostMap grids list.
The lane_control layer goes to steering_cost via the existing else branch
(np.maximum), so no routing logic changes were needed.
'''

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
import yaml
import cv2

from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from navigator_msgs.msg import Egma
from rosgraph_msgs.msg import Clock
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ros2_numpy.geometry import quat_to_numpy, numpy_to_quat
from tf_transformations import quaternion_multiply, euler_from_quaternion
from ros2_numpy.occupancy_grid import occupancygrid_to_numpy

from scipy import ndimage

import matplotlib.pyplot as plt


STALENESS_TOLERANCE = 0.25  # seconds. Grids older than this will be ignored.

CURRENT_OCCUPANCY_SCALE = 1.0 # 0.75
FUTURE_OCCUPANCY_SCALE = 1.0 #3.0
DRIVABLE_GRID_SCALE = 1.0 #0.75
ROUTE_DISTANCE_GRID_SCALE = 1.0
JUNCTION_GRID_SCALE = 1.0
LANE_CONTROL_SCALE      = 1.0  # lane_controlled_costmap_node output


class GridSummationNode(Node):

    def __init__(self):
        """Subscribe to the desired cost maps

        - Drivable surface  (~5 Hz)
        - Route distance
        - Current occupancy (~8 Hz)
        - Lane control      (~20 Hz)  ← new

        """
        super().__init__('grid_summation_node')

        # Set up the global config file
        self.declare_parameter('global_config', 'temp_value')
        self.file_path = self.get_parameter('global_config').value

        # Subscriptions and publishers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccupancyCb, 1)
        self.current_occupancy_grid = None

        self.future_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/predictions_combined', self.futureOccupancyCb, 1)
        self.future_occupancy_grid = None

        self.drivable_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.drivableGridCb, 1)
        self.drivable_grid = None

        self.junction_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/stateful_junction', self.junctionGridCb, 1)
        self.junction_grid = None

        self.route_dist_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/route_distance', self.routeDistGridCb, 1)
        self.route_dist_grid = None

        # ── Lane control layer ─────────────────────────────────────────────────
        self.lane_control_sub = self.create_subscription(
            OccupancyGrid, '/grid/lane_control', self.laneControlCb, 1)
        self.lane_control_grid = None
        # ──────────────────────────────────────────────────────────────────────

        self.steering_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/steering_cost', 1)

        self.speed_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/speed_cost', 1)

        self.combined_egma_pub = self.create_publisher(
            Egma, '/egma/cost', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_status', 1)
        self.status = DiagnosticStatus()
        self.status.level = DiagnosticStatus.OK

        self.combine_timer = self.create_timer(0.05, self.createCostMap, callback_group=MutuallyExclusiveCallbackGroup())

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    # Make sure we're only keeping the newest grid messages
    def currentOccupancyCb(self, msg: OccupancyGrid):
        if self.current_occupancy_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.current_occupancy_grid.header.stamp.sec + self.current_occupancy_grid.header.stamp.nanosec*1e-9:
            self.current_occupancy_grid = msg

    def futureOccupancyCb(self, msg: OccupancyGrid):
        if self.future_occupancy_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.future_occupancy_grid.header.stamp.sec + self.future_occupancy_grid.header.stamp.nanosec*1e-9:
            self.future_occupancy_grid = msg

    def drivableGridCb(self, msg: OccupancyGrid):
        if self.drivable_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.drivable_grid.header.stamp.sec + self.drivable_grid.header.stamp.nanosec*1e-9:
            self.drivable_grid = msg

    def junctionGridCb(self, msg: OccupancyGrid):
        if self.junction_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.junction_grid.header.stamp.sec + self.junction_grid.header.stamp.nanosec*1e-9:
            self.junction_grid = msg

    def routeDistGridCb(self, msg: OccupancyGrid):
        if self.route_dist_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.route_dist_grid.header.stamp.sec + self.route_dist_grid.header.stamp.nanosec*1e-9:
            self.route_dist_grid = msg

    def laneControlCb(self, msg: OccupancyGrid):
        if self.lane_control_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.lane_control_grid.header.stamp.sec + self.lane_control_grid.header.stamp.nanosec*1e-9:
            self.lane_control_grid = msg

    def checkForStaleness(self, grid: OccupancyGrid):
        stamp = grid.header.stamp
        stamp_in_seconds = stamp.sec + stamp.nanosec*1e-9
        current_time_in_seconds = self.clock.clock.sec + self.clock.clock.nanosec*1e-9

        stale_time = current_time_in_seconds - stamp_in_seconds
        stale = stale_time > STALENESS_TOLERANCE
        print("Stale for " + str(stale_time) + " seconds")

        if stale:
            self.status.level = DiagnosticStatus.WARN
            self.status.message = "Current occupancy was stale."
            return current_time_in_seconds - stamp_in_seconds
        else:
            return 0

    def fastforward(self, grid: OccupancyGrid):
        if grid is None:
            return grid

        # convert the grid to numpy array, we'll treat it as an image
        grid_img = occupancygrid_to_numpy(grid)
        old_dim = grid_img.shape[0]
        grid_res = grid.info.resolution

        if old_dim == 128:
            dim_bef_resize = old_dim

            grid_img = self.resizeOccupancyGrid(grid_img)
            old_dim = grid_img.shape[0]
            grid_res = dim_bef_resize * grid_res / old_dim

        # find that transform between base_link frames from when the message was made until now
        t = self.tf_buffer.lookup_transform_full(
            target_frame='base_link',
            target_time=rclpy.time.Time(),
            source_frame='base_link',
            source_time=grid.header.stamp,
            fixed_frame='map',
            timeout=rclpy.duration.Duration(seconds=5.0))

        roll, pitch, yaw = euler_from_quaternion(quat_to_numpy(t.transform.rotation))

        # Open the config file
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
        except FileNotFoundError:
            print("Error: config.yaml not found.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")

        x = t.transform.translation.x + ((0.5 * data['occupancy_grids']['vehicle_longitudinal_location']) - (0.5 * data['occupancy_grids']['vehicle_longitudinal_location'])*np.cos(-yaw))
        y = t.transform.translation.y - (0.5 * data['occupancy_grids']['vehicle_longitudinal_location'])*np.sin(-yaw)

        shift = [y/grid_res, x/grid_res]

        grid_img = ndimage.rotate(grid_img, np.degrees(-yaw), reshape=True)
        new_dim = grid_img.shape[0]
        diff = int((new_dim-old_dim)/2)
        grid_img = ndimage.shift(grid_img, shift)
        grid_img = grid_img[diff:new_dim-diff, diff:new_dim-diff]

        # sometimes grid_img emerges with 152 pixels..
        prezoom_rows = grid_img.shape[0]
        if prezoom_rows != 151:
            grid_img = ndimage.zoom(grid_img, 151.0/float(grid_img.shape[0]))

        grid_out = OccupancyGrid()
        grid_out.info.map_load_time = self.clock.clock
        grid_out.info.resolution = (grid_res * old_dim / (new_dim - diff)) * prezoom_rows / grid_img.shape[0]
        # shape[0]=rows=height (y/lateral), shape[1]=cols=width (x/longitudinal).
        # Assigning shape[0] to width and shape[1] to height was the x/y flip bug:
        # downstream reshape(height, width) would silently transpose non-square grids.
        grid_out.info.width  = grid_img.shape[1]   # cols → x extent (width)
        grid_out.info.height = grid_img.shape[0]   # rows → y extent (height)
        grid_out.info.origin.position.x = grid_img.shape[1] * grid_out.info.resolution * 2 / 3 * -1
        grid_out.info.origin.position.y = grid_img.shape[0] * grid_out.info.resolution * 1 / 2 * -1
        grid_out.header.stamp = self.clock.clock
        grid_out.header.frame_id = 'base_link'
        grid_out.data = grid_img.astype(np.int8).flatten().tolist()

        return grid_out

    def getWeightedArrayFromNumpy(self, msg: OccupancyGrid, scale: float) -> np.ndarray:
        height, width = msg.shape

        height = int(height)
        width = int(width)

        # np.asarray converts values safely for masked arrays and plain ndarrays.
        # Do NOT use msg.data here — on a numpy array that is the raw byte buffer,
        # which reinterprets memory and causes a reshape ValueError.
        arr = np.asarray(msg, dtype=np.float16).reshape(height, width)

        arr *= scale

        return arr

    def getWeightedArrayFromOccupancyGrid(self, msg: OccupancyGrid, scale: float) -> np.ndarray:
        arr = np.asarray(msg.data, dtype=np.float16).reshape(msg.info.height, msg.info.width)

        arr *= scale

        return arr

    def resizeOccupancyGrid(self, original: np.ndarray) -> np.ndarray:
        # Remove every 6th row
        rows_to_delete = np.arange(0, original.shape[0], 6)
        downsampled = np.delete(original, rows_to_delete, axis=0)

        # Remove every 6th column
        cols_to_delete = np.arange(0, original.shape[1], 6)
        downsampled = np.delete(downsampled, cols_to_delete, axis=1)

        # trim 3 columns from the left (behind the car)
        downsampled = downsampled[:, 3:]

        # Now make sure downsampled has the correct shape for the background
        background = np.zeros((151, 151))
        h, w = downsampled.shape
        background[22:22+h, 0:w] = downsampled

        return background

    def createCostMap(self):
        steering_cost = np.zeros((151, 151))
        speed_cost = np.zeros((151, 151))

        grids = [('occupancy',        self.current_occupancy_grid,  CURRENT_OCCUPANCY_SCALE),
                 ('future_occupancy', self.future_occupancy_grid,   FUTURE_OCCUPANCY_SCALE),
                 ('drivable',         self.drivable_grid,           DRIVABLE_GRID_SCALE),
                 ('route_dist',       self.route_dist_grid,         ROUTE_DISTANCE_GRID_SCALE),
                 ('junction',         self.junction_grid,           JUNCTION_GRID_SCALE),
                 ('lane_control',     self.lane_control_grid,       LANE_CONTROL_SCALE),  # ← new
                ]

        try:

            for grid_name, grid, scale in grids:
                if grid is None or len(grid.data) == 0:
                    print("GRID NOT FOUND")
                    continue

                # Pad or trim data array to match declared dimensions
                data_dim = int(grid.info.height * grid.info.width)
                if len(grid.data) < data_dim:
                    for i in range(data_dim - len(grid.data)):
                        grid.data.append(0)
                elif len(grid.data) > data_dim:
                    grid.data = grid.data[:data_dim]

                stale = self.checkForStaleness(grid)
                if stale > 0:
                    ff_grid = self.fastforward(grid)
                    weighted_grid_arr = self.getWeightedArrayFromOccupancyGrid(ff_grid, scale)
                else:
                    ff_grid = occupancygrid_to_numpy(grid)
                    # np.asarray(arr, ...) converts values — safe for masked arrays and
                    # plain ndarrays alike.  Do NOT use arr.data which is a raw byte
                    # buffer and reinterprets memory, causing a reshape ValueError when
                    # the source dtype is wider than float16 (e.g. float64 from
                    # resizeOccupancyGrid's np.zeros output).
                    # NOTE: occupancy grids now publish at 300×300 (from StaticOccupancyNode
                    # fix); resizeOccupancyGrid expected 128×128 input and will crash with
                    # 300×300 — do not call it here.
                    weighted_grid_arr = np.asarray(ff_grid, dtype=np.float16) * scale

                # Normalise every layer to 151×151 before accumulation.
                # drivable / route_dist / junction grids arrive at 300×300;
                # zoom them down so np.maximum doesn't raise a broadcast error.
                if weighted_grid_arr.shape != (151, 151):
                    factor = 151.0 / weighted_grid_arr.shape[0]
                    weighted_grid_arr = ndimage.zoom(
                        weighted_grid_arr.astype(np.float32), factor
                    ).astype(np.float16)
                    # zoom output may be 151 or 152 due to float rounding — clip
                    weighted_grid_arr = weighted_grid_arr[:151, :151]

                if grid_name == 'drivable':
                    steering_cost = np.maximum(steering_cost, weighted_grid_arr)
                elif grid_name == 'junction':
                    speed_cost = np.maximum(speed_cost, weighted_grid_arr)
                else:
                    # occupancy, future_occupancy, route_dist, lane_control → steering
                    steering_cost = np.maximum(steering_cost, weighted_grid_arr)

            # Cap this to 100
            steering_cost = np.clip(steering_cost, 0, 100)
            speed_cost = np.clip(speed_cost, 0, 100)

            # Open the config file
            try:
                with open(self.file_path, 'r') as file:
                    data = yaml.safe_load(file)
            except FileNotFoundError:
                print("Error: config.yaml not found.")
            except yaml.YAMLError as e:
                print(f"Error parsing YAML file: {e}")

            # Publish steering cost — resized to config dimensions via cv2
            resolution = data['occupancy_grids']['resolution']
            grid_cols = int(data['occupancy_grids']['width']  / resolution)  # 300
            grid_rows = int(data['occupancy_grids']['length'] / resolution)  # 300

            steering_cost_msg = OccupancyGrid()
            steering_cost_msg.info.map_load_time = self.clock.clock
            steering_cost_msg.info.resolution = resolution
            # origin is the lower-left corner of the map in base_link:
            # x = forward (longitudinal), y = left (latitudinal)
            steering_cost_msg.info.origin.position.x = -1 * data['occupancy_grids']['vehicle_longitudinal_location']
            steering_cost_msg.info.origin.position.y = -1 * data['occupancy_grids']['vehicle_latitudinal_location']
            steering_cost_msg.header.stamp = self.clock.clock
            steering_cost_msg.header.frame_id = 'base_link'

            resized_grid = cv2.resize(steering_cost.astype(np.float32), (grid_cols, grid_rows), interpolation=cv2.INTER_NEAREST)
            steering_cost_msg.data = resized_grid.astype(np.int8).flatten().tolist()
            steering_cost_msg.info.width  = grid_cols
            steering_cost_msg.info.height = grid_rows

            self.steering_cost_pub.publish(steering_cost_msg)

            speed_cost_msg = OccupancyGrid()
            speed_cost_msg.info.map_load_time = self.clock.clock
            speed_cost_msg.info.resolution = resolution
            speed_cost_msg.info.origin.position.x = -1 * data['occupancy_grids']['vehicle_longitudinal_location']
            speed_cost_msg.info.origin.position.y = -1 * data['occupancy_grids']['vehicle_latitudinal_location']
            speed_cost_msg.header.stamp = self.clock.clock
            speed_cost_msg.header.frame_id = 'base_link'

            resized_speed = cv2.resize(speed_cost.astype(np.float32), (grid_cols, grid_rows), interpolation=cv2.INTER_NEAREST)
            speed_cost_msg.data = resized_speed.astype(np.int8).flatten().tolist()
            speed_cost_msg.info.width  = grid_cols
            speed_cost_msg.info.height = grid_rows

            self.speed_cost_pub.publish(speed_cost_msg)
        except (Exception) as e:
            self.get_logger().warn('Error composing aggregate cost map - likely waiting for Transform Buffer.')
            self.get_logger().warn(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = GridSummationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
