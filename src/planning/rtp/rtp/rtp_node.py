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
from dataclasses import dataclass
import random
import time
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
from std_msgs.msg import String


from matplotlib.patches import Rectangle

N_BRANCHES: int = 9
STEP_LEN: float = 3.0  # meters
DEPTH: int = 2

# These are vdehicle constants for the GEM e6.
# The sim vehicle (Tesla Model 3) has similar constants.
WHEEL_BASE: float = 3.5  # meters
MAX_TURN_ANGLE = 0.42  # radians
COST_CUTOFF = 90


@dataclass
class CostedPath:
    poses = []  # pose = [x,y,heading]
    cost = 0.0

    def append(self, segment):
        self.poses += segment.poses
        self.cost += segment.cost

    def copy(self):
        new_path = CostedPath()
        new_path.poses = self.poses.copy()
        new_path.cost = self.cost
        return new_path


class RecursiveTreePlanner(Node):
    def __init__(self):
        super().__init__('rrt_node')

        cost_map_sub = self.create_subscription(
            OccupancyGrid, '/grid/cost', self.costMapCb, 1)

        odom_sub = self.create_subscription(
            Odometry, '/odometry/gnss_processed', self.odomCb, 1)

        clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)
        self.clock = Clock()

        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 1)

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedCb, 1)

        self.path_pub = self.create_publisher(
            Path, '/planning/path', 1)
        
        self.airbag_sub = self.create_subscription(String, '/planning/current_airbag', self.airbagCb, 1)

        self.ego_pose = None  # [x, y, heading]

        self.speed = 0.0
        self.airbag = 'RED'

        # Clock subscription
        # self.clock_sub = self.create_subscription(
        #     Clock, '/clock', self.clockCb, 10)
        # self._cached_clock_ = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    def airbagCb(self, msg: String):
        self.airbag = msg.data

    def speedCb(self, msg: CarlaSpeedometer):
        self.speed = msg.speed

    def getSegment(self, inital_pose: np.ndarray, steering_angle, segment_length: float, res: float, costmap) -> CostedPath:
        end_pose = np.copy(inital_pose)
        segment_poses = []
        current_length = 0.0
        total_cost = 0
        while current_length < segment_length:
            end_heading = end_pose[2]

            x, y = (res * np.cos(end_heading) + end_pose[0],
                    res * np.sin(end_heading) + end_pose[1])

            # Check cost at (x,y). Row-major means y is first!
            if costmap.shape[0] < 1:
                self.get_logger().warn("Costmap was empty")
                return
            cost = costmap[int(y), int(x)]
            if cost > COST_CUTOFF:
                return

            total_cost += cost

            segment_poses.append([
                x, y, end_heading + steering_angle
            ])
            end_pose = segment_poses[-1]
            current_length += res

        path = CostedPath()
        path.poses = segment_poses
        path.cost = total_cost

        return path

    def generatePaths(self, depth: int, path: CostedPath, steering_angle, segment_length, res, num_branches, results: list, result_costs, costmap):
        # Current pose at this step is the latest pose in the current path
        pose = path.poses[-1]
        segment = self.getSegment(
            pose, steering_angle, segment_length, res, costmap)

        if segment is None:  # getSegment hit an obstacle
            return

        new_path: CostedPath = path.copy()

        new_path.append(segment)

        if depth == 0:
            results.append(new_path)

            if random.randint(0, 20) == 0:  # Only plot 5% of results
                poses_np = np.asarray(new_path.poses)
                # plt.plot(poses_np[:, 0], poses_np[:, 1])
            return

        for angle in np.linspace(-MAX_TURN_ANGLE, MAX_TURN_ANGLE, num_branches):
            self.generatePaths(depth-1, new_path, angle, segment_length,
                               res, num_branches, results, result_costs, costmap)

    def startGeneration(self, costmap: np.ndarray, depth=7, segment_length=9.0, branches=7):
        # plt.figure(figsize=(8, 6), dpi=160)
        # plt.axes().set_aspect('equal')
        # print(f"Generating path with depth {depth}, seg len {segment_length}")
        origin = (50., 75.)

        path = CostedPath()
        path.poses = [[origin[0], origin[1], 0.0]]
        path.cost = 0
        results = []
        costs = []

        res = 1.0
        for angle in np.linspace(-MAX_TURN_ANGLE, MAX_TURN_ANGLE, branches):
            self.generatePaths(depth-1, path, angle, segment_length,
                               res, branches, results, costs, costmap)

        # results = np.asarray(results)
        # print(results)
        # plt.imshow(costmap, origin='lower')
        # plt.xlim((25,75))
        # plt.ylim((60,90))
        return results

    def costMapCb(self, msg: OccupancyGrid):
        start = time.time()
        costmap = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        results = self.startGeneration(
            costmap, depth=4, segment_length=12, branches=9)

        min_cost = 100000
        best_path: CostedPath = None

        for result in results:
            if result.cost < min_cost:
                min_cost = result.cost
                best_path = result

        result_msg = Path()
        result_msg.header.frame_id = "base_link"
        result_msg.header.stamp = self.clock.clock

        if best_path is None:
            self.get_logger().error("No valid path was found!")
            return

        for pose in best_path.poses:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "base_link"
            pose_msg.header.stamp = self.clock.clock
            pose_msg.pose.position.x = pose[0] * 0.4 - 20
            pose_msg.pose.position.y = pose[1] * 0.4 - 30
            # TODO: Add heading (pose[2]?)
            result_msg.poses.append(pose_msg)

        command = CarlaEgoVehicleControl()
        command.steer = best_path.poses[2][2] * -2.5  # First steering value

        if command.steer > 1.0:
            command.steer = 1.0
        elif command.steer < -1.0:
            command.steer = -1.0

        command.header.stamp = self.clock.clock

        DESIRED_SPEED = 7 - command.steer * 5  # m/s, ~10mph

        if self.airbag == 'RED':
            DESIRED_SPEED = 0.0
            print("RED")
        elif self.airbag == 'AMBER':
            DESIRED_SPEED = 1.0
            print("AMBER")

        elif self.airbag == 'YELLOW':
            DESIRED_SPEED = 4.0
            print("Yellow")


        if self.speed < DESIRED_SPEED:
            command.throttle = 0.4
            command.brake = 0.0
            print(f"Driving, steer {command.steer}")
        else:
            command.throttle = 0.0
            command.brake = 1.0
            print(f"Braking, steer {command.steer}")

        self.command_pub.publish(command)

        self.path_pub.publish(result_msg)
        # print(best_path.poses)

        # print(f"Done in {time.time() - start}!")

    def odomCb(self, msg: Odometry):

        pos = msg.pose.pose.position
        heading = np.arcsin(msg.pose.pose.orientation.z) * 2

        self.ego_pose = [pos.x, pos.y, heading]
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
