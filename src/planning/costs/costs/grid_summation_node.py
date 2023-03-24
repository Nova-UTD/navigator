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
from carla_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from nova_msgs.msg import Egma
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from skimage.morphology import erosion

import matplotlib.pyplot as plt

STALENESS_TOLERANCE = 0.2  # seconds. Grids older than this will be ignored.

CURRENT_OCCUPANCY_SCALE = 1.0
FUTURE_OCCUPANCY_SCALE = 3.0
DRIVABLE_GRID_SCALE = 1.0
ROUTE_DISTANCE_GRID_SCALE = 1.0
JUNCTION_GRID_SCALE = 1.0


class GridSummationNode(Node):

    def __init__(self):
        """Subscribe to the desired cost maps

        - Drivable surface  (~5 Hz)
        - Route distance
        - Current occupancy (~8 Hz)

        """
        super().__init__('grid_summation_node')

        self.current_occupancy_grid = OccupancyGrid()
        self.future_occupancy_grid = OccupancyGrid()
        self.drivable_grid = OccupancyGrid()
        self.junction_grid = OccupancyGrid()
        self.route_dist_grid = OccupancyGrid()
        self.status = DiagnosticStatus()

        # Subscriptions and publishers
        self.current_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccupancyCb, 1)

        self.future_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/predictions_combined', self.futureOccupancyCb, 1)

        self.drivable_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.drivableGridCb, 1)

        self.junction_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/stateful_junction', self.junctionGridCb, 1)

        self.route_dist_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/route_distance', self.routeDistGridCb, 1)

        self.junction_occupancy_pub = self.create_publisher(
            OccupancyGrid, '/grid/junction_occupancy', 1)

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedometerCb, 1)

        self.steering_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/cost', 1)

        self.speed_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/speed_cost', 1)

        self.combined_egma_pub = self.create_publisher(
            Egma, '/egma/cost', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_status', 1)

        self.combine_timer = self.create_timer(0.2, self.createCostMap)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

        self.speed = 0.0
        self.ego_has_stopped = False
        self.last_stop_time = time.time()

    def speedometerCb(self, msg: CarlaSpeedometer):
        self.speed = msg.speed

        # seconds. If the ego has stopped less than ten seconds ago, we still say that the ego has stopped
        STOP_COOLDOWN = 5.0

        if time.time() - self.last_stop_time < STOP_COOLDOWN:
            self.ego_has_stopped = True
        elif self.speed < 0.1:
            self.last_stop_time = time.time()
            self.ego_has_stopped = True
        else:
            self.ego_has_stopped = False
            print("Ego has NOT stopped")

    def clockCb(self, msg: Clock):
        self.clock = msg

    def currentOccupancyCb(self, msg: OccupancyGrid):
        self.current_occupancy_grid = msg

    def futureOccupancyCb(self, msg: OccupancyGrid):
        self.future_occupancy_grid = msg

    def drivableGridCb(self, msg: OccupancyGrid):
        self.drivable_grid = msg

    def junctionGridCb(self, msg: OccupancyGrid):
        self.junction_grid = msg

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

        steering_cost = np.zeros((151, 151))
        speed_cost = np.zeros((151, 151))

        # Calculate the weighted cost map layers

        # 1. Current occupancy
        stale = self.checkForStaleness(self.current_occupancy_grid, status)
        empty = len(self.current_occupancy_grid.data) == 0

        weighted_current_occ_arr = None

        if not stale and not empty:
            msg = self.current_occupancy_grid
            weighted_current_occ_arr = self.getWeightedArray(
                msg, CURRENT_OCCUPANCY_SCALE)
            weighted_current_occ_arr = self.resizeOccupancyGrid(
                weighted_current_occ_arr)
            steering_cost += weighted_current_occ_arr
            speed_cost += weighted_current_occ_arr

        # 2. Future occupancy
        stale = self.checkForStaleness(self.future_occupancy_grid, status)
        empty = len(self.future_occupancy_grid.data) == 0

        if not stale and not empty:
            msg = self.future_occupancy_grid
            weighted_future_occ_arr = self.getWeightedArray(
                msg, FUTURE_OCCUPANCY_SCALE)
            weighted_future_occ_arr = self.resizeOccupancyGrid(
                weighted_future_occ_arr)
            steering_cost += weighted_future_occ_arr
            speed_cost += weighted_future_occ_arr

        # 3. Drivable area
        stale = self.checkForStaleness(self.drivable_grid, status)
        empty = len(self.drivable_grid.data) == 0

        if not stale and not empty:
            msg = self.drivable_grid
            weighted_drivable_arr = self.getWeightedArray(
                msg, DRIVABLE_GRID_SCALE)
            steering_cost += weighted_drivable_arr
            # speed_cost += weighted_drivable_arr

        # 4. Route distance
        stale = self.checkForStaleness(self.route_dist_grid, status)
        empty = len(self.route_dist_grid.data) == 0

        if not stale and not empty:
            msg = self.route_dist_grid
            weighted_route_dist_arr = self.getWeightedArray(
                msg, ROUTE_DISTANCE_GRID_SCALE)
            steering_cost += weighted_route_dist_arr

        # 5. Junctions
        stale = self.checkForStaleness(self.junction_grid, status)
        empty = len(self.junction_grid.data) == 0

        if not stale and not empty and weighted_current_occ_arr is not None:
            msg = self.junction_grid
            weighted_junction_arr = self.getWeightedArray(
                msg, JUNCTION_GRID_SCALE)

            speed_cost += weighted_junction_arr

        # Cap this to 100
        steering_cost = np.clip(steering_cost, 0, 100)
        speed_cost = np.clip(speed_cost, 0, 100)

        # plt.show()

        # Publish as an OccupancyGrid
        result_msg = OccupancyGrid()

        result_msg.data = steering_cost.astype(np.int8).flatten().tolist()
        result_msg.info = self.drivable_grid.info
        result_msg.header = self.drivable_grid.header

        self.steering_cost_pub.publish(result_msg)

        result_msg.data = speed_cost.astype(np.int8).flatten().tolist()
        self.speed_cost_pub.publish(result_msg)

        egma_msg = Egma()
        egma_msg.header = result_msg.header

        current_stamp = result_msg.header.stamp
        t = current_stamp.sec + current_stamp.nanosec * 1e-9
        for i in range(15):
            frame = result_msg

            t += 0.1  # dt = 0.1 seconds
            next_stamp = current_stamp
            next_stamp.sec = int(t)
            next_stamp.nanosec = int(t * 1e9 % 1e9)

            result_msg.header.stamp = next_stamp
            result_msg.header.frame_id = 'base_link'

            egma_msg.egma.append(frame)

        self.combined_egma_pub.publish(egma_msg)


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
