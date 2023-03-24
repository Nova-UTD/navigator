'''
Package: grids
   File: junction_manager.py
 Author: Will Heitman (w at heit dot mn)

Given a junction grid (where each cell is either within an intersection or not),
plus the current ego speed and occupancy grid, publish a map that either
includes the junction cost or not. This basically switches a junction's
cost on or off, allowing or blocking entrance into junctions based on perception data.

Subscriptions:
- Junction map (OccupancyGrid)
- Speed (CarlaSpeedometer)
- Current occupancy (OccupancyGrid)

Publisher:
- "Stafeful" junction map (OccupancyGrid)
    - This is either an empty OccupancyGrid (zeros)
      or the original junction map.
- Diagnostic status (DiagnosticStatus)

'''

from array import array as Array
import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from carla_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock

import matplotlib.pyplot as plt

from skimage.morphology import binary_erosion, square


class JunctionManager(Node):

    def __init__(self):
        super().__init__('junction_manager')

        self.current_occupancy_grid = None
        self.junction_grid = None
        self.status = DiagnosticStatus()  # TODO: Add status
        self.time_sec = 0.0
        self.speed = 0.0
        self.ego_has_stopped = False
        self.last_stop_time = 0.0
        self.last_in_junction_time = 0.0

        # Subscriptions and publishers
        self.current_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccupancyCb, 1)

        self.junction_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/junction', self.junctionGridCb, 1)

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedometerCb, 1)

        self.stateful_grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/stateful_junction', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

    def speedometerCb(self, msg: CarlaSpeedometer):
        self.speed = msg.speed

    def clockCb(self, msg: Clock):
        self.time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

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

    def currentOccupancyCb(self, msg: OccupancyGrid):
        occupancy_grid = np.asarray(msg.data,
                                    dtype=np.int8).reshape(msg.info.height, msg.info.width)

        occupancy_grid = self.resizeOccupancyGrid(occupancy_grid)

        # This creates a binary array, where each cell is simply "is occupied" or "is not occupied"
        self.current_occupancy_grid = occupancy_grid > 80.0

    def junctionIsOccupied(self, occupancy, junction) -> bool:
        # Erode the occupancy grid slightly.
        # This "denoises" it, preventing tiny objects from throwing
        # false positives.
        if occupancy is None:
            self.get_logger().warn("Occpancy grid unavailable. Skipping.")
            return

        eroded_occupancy = binary_erosion(occupancy)

        # square(5) creates a 5-cell wide erosion region.
        # Smaller values erode less.
        eroded_junction = binary_erosion(junction, square(1))

        is_occupied = np.sum(eroded_junction*eroded_occupancy) > 0.0

        # if is_occupied:
        #     plt.imshow(eroded_junction*eroded_occupancy, origin='lower')
        #     plt.scatter(50, 75)
        #     plt.show()

        return is_occupied

    def egoCanEnter(self, speed: float, occupancy: np.ndarray, junction: np.ndarray) -> bool:
        """Can the ego enter the nearest junction?

        Yes iff:
        - the ego has stopped (speed less than 0.5 m/s in past 3s) and the junction is not occupied,
        - OR the ego is significantly in the junction (beyond the edges)

        If the latter is true yet the junction IS occupied, we rely on object handling beyond
        the scope of this node.

        Args:
            speed (float): Ego speed, in m/s
            occupancy (np.ndarray): Current occupancy grid
            junction (np.ndarray): Junction grid
        """
        STOP_SPEED_THRESHOLD = 0.5  # m/s
        STOP_STALENESS_THRESHOLD = 1.5  # seconds
        EGO_CELL = (75, 50)

        can_enter = False
        junction_is_occupied = False
        in_junction = False

        if speed < STOP_SPEED_THRESHOLD:
            self.last_stop_time = self.time_sec

        has_stopped_recently = self.time_sec - \
            self.last_stop_time < STOP_STALENESS_THRESHOLD

        if has_stopped_recently:
            junction_is_occupied = self.junctionIsOccupied(occupancy, junction)
            if not junction_is_occupied:
                can_enter = True

        if not can_enter:
            eroded_junction = binary_erosion(junction, square(2))

            in_junction = eroded_junction[EGO_CELL] == True

            # plt.imshow(eroded_junction)
            # plt.show()

            if in_junction:
                self.last_in_junction_time = self.time_sec

            was_in_junction_recently = self.time_sec - \
                self.last_in_junction_time < STOP_STALENESS_THRESHOLD
            if was_in_junction_recently:
                can_enter = True

        # if in_junction:
        #     print("Already inside junction")
        # elif not has_stopped_recently:
        #     print("Can't enter. Hasn't stopped recently.")
        # elif junction_is_occupied:
        #     print("Can't enter. Junction occupied.")

        return can_enter

    def junctionGridCb(self, msg: OccupancyGrid):

        # The "> 0 " makes this an efficient binary grid
        junction_grid = np.asarray(msg.data,
                                   dtype=np.int8).reshape(msg.info.height, msg.info.width) > 0

        stateful_junction_grid = np.zeros((msg.info.height, msg.info.width))

        # If the ego cannot enter, set the stateful grid to include the junction cost
        if not self.egoCanEnter(self.speed, self.current_occupancy_grid, junction_grid):
            stateful_junction_grid = junction_grid.astype(np.int8)*100

        stateful_msg = OccupancyGrid()
        stateful_msg.data = Array(
            'b', stateful_junction_grid.ravel().astype(np.int8))
        stateful_msg.info = msg.info
        stateful_msg.header = msg.header
        self.stateful_grid_pub.publish(stateful_msg)

    def routeDistGridCb(self, msg: OccupancyGrid):
        self.route_dist_grid = msg

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


def main(args=None):
    rclpy.init(args=args)

    junction_manager = JunctionManager()

    rclpy.spin(junction_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    junction_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
