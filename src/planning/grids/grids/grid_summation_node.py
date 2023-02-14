'''
Package: grids
   File: grid_summation_node.py
 Author: Will Heitman (w at heit dot mn)

Subscribes to cost maps, calculates their weighted sum, and
publishes the result as a finished cost map.
'''

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

import matplotlib.pyplot as plt

STALENESS_TOLERANCE = 0.2  # seconds. Grids older than this will be ignored.

CURRENT_OCCUPANCY_SCALE = 1.0
DRIVABLE_GRID_SCALE = 1.0
ROUTE_DISTANCE_GRID_SCALE = 1.0


class GridSummationNode(Node):

    def __init__(self):
        """Subscribe to the desired cost maps

        - Drivable surface  (~5 Hz)
        - Route distance
        - Current occupancy (~8 Hz)

        """
        super().__init__('grid_summation_node')

        self.current_occupancy_grid = OccupancyGrid()
        self.drivable_grid = OccupancyGrid()
        self.route_dist_grid = OccupancyGrid()
        self.status = DiagnosticStatus()

        # Subscriptions and publishers
        self.current_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccupancyCb, 1)

        self.drivable_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.drivableGridCb, 1)

        self.route_dist_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/route_distance', self.routeDistGridCb, 1)

        self.combined_map_pub = self.create_publisher(
            OccupancyGrid, '/grid/cost', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/status/grid_summation', 1)

        self.combine_timer = self.create_timer(0.2, self.createCostMap)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    def currentOccupancyCb(self, msg: OccupancyGrid):
        self.current_occupancy_grid = msg

    def drivableGridCb(self, msg: OccupancyGrid):
        self.drivable_grid = msg

    def routeDistGridCb(self, msg: OccupancyGrid):
        self.route_dist_grid = msg

    def checkForStaleness(self, grid: OccupancyGrid, status: DiagnosticStatus):
        stamp = self.current_occupancy_grid.header.stamp
        stamp_in_seconds = stamp.sec + stamp.nanosec*1e-9
        current_time_in_seconds = self.clock.clock.sec + self.clock.clock.nanosec*1e-9
        stale = current_time_in_seconds - stamp_in_seconds > STALENESS_TOLERANCE
        if stale:
            if status.level == DiagnosticStatus.OK:
                status.level = DiagnosticStatus.WARN
                status.message = "Current occupancy was stale."
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "More than one layer was stale!"

    def getWeightedArray(self, msg: OccupancyGrid, scale: float) -> np.ndarray:
        """Converts the OccupancyGrid message into a numpy array, then multiplies it by scale

        Args:
            msg (OccupancyGrid)
            scale (float)

        Returns:
            np.ndarray: Weighted ndarray
        """
        arr = np.asarray(msg.data, dtype=np.float16).reshape(
            msg.info.height, msg.info.width)

        arr *= scale

        return arr

    def resizeOccupancyGrid(self, original: np.ndarray) -> np.ndarray:
        # This is a temporary measure until our current occ. grid's params
        # match the rest of our stack. Basically, due to the difference in cell size,
        # 1 out of 6 cells in the array needs to be deleted.
        downsampled = np.delete(original, np.arange(0, 128, 6), axis=0)
        downsampled = np.delete(downsampled, np.arange(
            0, 128, 6), axis=1)  # 106x106 result

        # trim the bottom columns (behind the car)
        downsampled = downsampled[:, 3:]

        background = np.zeros((151, 151))

        background[22:128, 0:103] = downsampled
        return background  # Correct scale

    def createCostMap(self):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'grid_summation'

        result = np.zeros((151, 151))

        # Calculate the weighted cost map layers

        # 1. Current occupancy
        stale = self.checkForStaleness(self.current_occupancy_grid, status)
        empty = len(self.current_occupancy_grid.data) == 0

        if not stale and not empty:
            msg = self.current_occupancy_grid
            weighted_current_occ_arr = self.getWeightedArray(
                msg, CURRENT_OCCUPANCY_SCALE)
            weighted_current_occ_arr = self.resizeOccupancyGrid(
                weighted_current_occ_arr)
            result += weighted_current_occ_arr

        # 2. Drivable area
        stale = self.checkForStaleness(self.drivable_grid, status)
        empty = len(self.drivable_grid.data) == 0

        if not stale and not empty:
            msg = self.drivable_grid
            weighted_drivable_arr = self.getWeightedArray(
                msg, DRIVABLE_GRID_SCALE)
            result += weighted_drivable_arr

        # 3. Route distance
        stale = self.checkForStaleness(self.route_dist_grid, status)
        empty = len(self.route_dist_grid.data) == 0

        if not stale and not empty:
            msg = self.route_dist_grid
            weighted_route_dist_arr = self.getWeightedArray(
                msg, ROUTE_DISTANCE_GRID_SCALE)
            result += weighted_route_dist_arr

        # Cap this to 100
        result = np.clip(result, 0, 100)

        # Publish as an OccupancyGrid
        result_msg = OccupancyGrid()

        result_msg.data = result.astype(np.int8).flatten().tolist()
        result_msg.info = self.drivable_grid.info
        result_msg.header = self.drivable_grid.header

        self.combined_map_pub.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)

    grid_summation_node = GridSummationNode()

    rclpy.spin(grid_summation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    grid_summation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
