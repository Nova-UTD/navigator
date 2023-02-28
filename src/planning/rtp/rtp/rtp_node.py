'''
Package:   rtp
Filename:  rtp_node.py
Author:    Will Heitman (w at heit.mn)

Recursive tree planner for path generation

For D iterations, generate N branches, where each branch is a path segment
in the map frame. Evaluate the complete cost of the path, from the previous iterations
to the current branch segment, using the latest cost map.

When the recursion depth is hit, append the complete path to an array, along with
its cost.

If at any point in the recursion the cost exceeds some limit, return immediately.

At the end of recursion, select the path in the array with the least cost.

Publish this least-cost path.

Subscribes to:
✅ /grid/cost (OccupancyGrid)
✅ /odometry/gnss_processed (Odometry)

Publishes:
✅ /planning/path (Path) in map frame

Minimum update rate: 2 Hz, ideally 5 Hz
'''

from matplotlib import pyplot as plt
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import ros2_numpy as rnp
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker

from matplotlib.patches import Rectangle

N_BRANCHES: int = 7
STEP_LEN: float = 3.0  # meters
DEPTH: int = 3

# These are vdehicle constants for the GEM e6.
# The sim vehicle (Tesla Model 3) has similar constants.
WHEEL_BASE: float = 3.5  # meters
MAX_TURN_ANGLE = 0.47


class CostedPath:
    points: np.ndarray
    cost: int

    def __init__(self) -> None:
        self.points = np.asarray([])
        self.cost = 0


class RecursiveTreePlanner(Node):
    def __init__(self):
        super().__init__('rrt_node')

        cost_map_sub = self.create_subscription(
            OccupancyGrid, '/grid/cost', self.costMapCb, 1)

        odom_sub = self.create_subscription(
            Odometry, '/odometry/gnss_processed', self.odomCb, 1)

        path_pub = self.create_publisher(
            Path, '/planning/path', 1)

        self.ego_pose = None  # [x, y, heading]

        # Clock subscription
        # self.clock_sub = self.create_subscription(
        #     Clock, '/clock', self.clockCb, 10)
        # self._cached_clock_ = Clock()

    def getPaths(self, grid: np.ndarray, pose: np.ndarray, path_so_far: CostedPath, end_vec, depth: int) -> list:
        if depth < 1:
            return

        for steering_value in np.linspace(-1.0, 1.0, N_BRANCHES):
            new_path = np.copy(path_so_far)
            angle = steering_value * MAX_TURN_ANGLE

            # Add a path based on the steering angle
            radius = WHEEL_BASE/np.tan(angle)

            t = 0.0
            pts = []

            while t < STEP_LEN:
                x = radius*np.cos(t)
                y = radius*np.sin(t)
                pts.append([x, y])
                t += 1.0  # resolution in meters

            plt.plot(pts)
            plt.show()

            self.getPaths(grid, pose, new_path, depth=depth-1)

    def costMapCb(self, msg: OccupancyGrid):
        if self.ego_pose is None:
            print("Ego pose is None")
            return
        grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

        origin_path = CostedPath()
        hdg = self.ego_pose[2]
        heading_vector = np.asarray([np.cos(hdg), np.sin(hdg)])
        paths = self.getPaths(grid, self.ego_pose, origin_path,
                              end_vec=heading_vector, depth=DEPTH)
        return

    def odomCb(self, msg: Odometry):

        pos = msg.pose.pose.position
        heading = np.arcsin(msg.pose.pose.orientation.z) * 2

        self.ego_pose = [pos.x, pos.y, heading]

        print(f"Got odom: {self.ego_pose}")
        return


def main(args=None):
    rclpy.init(args=args)

    rtp = RecursiveTreePlanner()

    rclpy.spin(rtp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rtp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
