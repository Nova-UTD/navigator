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
✅ /gnss/odometry_processed (Odometry)

Publishes:
✅ /planning/path (Path) in map frame

Minimum update rate: 2 Hz, ideally 5 Hz
'''

from matplotlib import pyplot as plt
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from navigator_msgs.msg import Mode
import numpy as np
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from dataclasses import dataclass
import random
import time
from geometry_msgs.msg import PoseStamped, Quaternion, Point
#from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
from navigator_msgs.msg import VehicleControl, VehicleSpeed
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Imu

from visualization_msgs.msg import Marker

from skimage.draw import line


from matplotlib.patches import Rectangle

N_BRANCHES: int = 15
STEP_LEN: float = 12.0  # meters
DEPTH: int = 1

# These are vehicle constants for the GEM e6.
# The sim vehicle (Tesla Model 3) has similar constants.
WHEEL_BASE: float = 3.5  # meters
MAX_TURN_ANGLE = 0.2  # radians
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
        super().__init__('rtp_node')

        self.speed_costmap = np.zeros((151, 151))
        self.origin = (44., 75.)

        # steering_cost_map_sub = self.create_subscription(
        #     OccupancyGrid, '/grid/route_distance', self.costMapCb, 1)
        steering_cost_map_sub = self.create_subscription(
            OccupancyGrid, '/grid/steering_cost', self.costMapCb, 1)

        speed_cost_map_sub = self.create_subscription(
            OccupancyGrid, '/grid/speed_cost', self.speedCostMapCb, 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        odom_sub = self.create_subscription(
            Odometry, '/gnss/odometry', self.odomCb, 1)

        current_mode_sub = self.create_subscription(
            Mode, '/guardian/mode', self.currentModeCb, 1)
        
        self.target_speed_pub = self.create_publisher(VehicleSpeed, '/planning/target_speed', 1)

        clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)
        self.clock = Clock().clock

        self.command_pub = self.create_publisher(
            VehicleControl, '/vehicle/control', 1)

        self.speed_sub = self.create_subscription(
            VehicleSpeed, '/speed', self.speedCb, 1)

        self.path_pub = self.create_publisher(
            Path, '/planning/path', 1)

        self.barrier_marker_pub = self.create_publisher(
            Marker, '/planning/barrier_marker', 1)

        self.ego_pose = None  # [x, y, heading]

        self.speed = 0.0

        self.previous_steer = 0.0

        self.current_mode = Mode.AUTO

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.mode

    def speedCostMapCb(self, msg: OccupancyGrid):
        if msg.info.height == 0:
            self.speed_costmap = np.asarray(msg.data, dtype=np.int8).reshape(
                int(np.sqrt(len(msg.data))), -1)

        else:
            self.speed_costmap = np.asarray(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def speedCb(self, msg: VehicleSpeed):
        self.speed = msg.speed

    def getBarrierIndex(self, path: CostedPath, map: np.ndarray, width_meters=1.8) -> int:
        """Given a path, perform a basic collision check and return the pose index
        for the first pose that fails the check (such as the pose that hits a vehicle,
        goes offroad, enters an intersection, etc).

        Args:
            path (CostedPath): Path to be checked
            map (np.ndarray): Cost map to use for the check

        Returns:
            int: The index of the first pose that fails the check, where the "barrier" lies
        """
        GRID_RES = 0.4  # meters per cell

        # This is the number of cells to extend to either side of the path
        # It's the full width (in cells) divided by two, rounded up
        REACH_CELLS = np.ceil(width_meters / GRID_RES / 3)  # = 3

        for i, pose in enumerate(path.poses):
            theta = pose[2] + np.pi/2

            ptA = np.asarray([np.cos(theta) * REACH_CELLS + pose[0],
                              np.sin(theta) * REACH_CELLS + pose[1]]).astype(np.int8)
            ptB = np.asarray([np.cos(theta) * REACH_CELLS * -1 + pose[0],
                              np.sin(theta) * REACH_CELLS * -1 + pose[1]]).astype(np.int8)

            # https://scikit-image.org/docs/stable/api/skimage.draw.html#skimage.draw.line
            rr, cc = line(ptA[1], ptA[0], ptB[1], ptB[0])

            max_cost = np.max(map[rr, cc])

            if max_cost > 90:
                # map[rr, cc] = 50
                # plt.xlim(40, 110)
                # plt.ylim(50, 100)
                # plt.imshow(map)
                # plt.show()
                return i

        return len(path.poses) - 1

    # def getSegment(self, inital_pose: np.ndarray, steering_angle, segment_length: float, res: float, costmap) -> CostedPath:
    #     end_pose = np.copy(inital_pose)
    #     segment_poses = []
    #     current_length = 0.0
    #     total_cost = 0

    #     x = end_pose[0]
    #     y = end_pose[1]

    #     idx = 0

    #     dx = (res * np.cos(steering_angle + inital_pose[2]))
    #     dy = (res * np.sin(steering_angle + inital_pose[2]))

    #     while current_length < segment_length:
    #         end_heading = end_pose[2]

    #         # x, y = (res * np.cos(end_heading) + end_pose[0],
    #         #         res * np.sin(end_heading) + end_pose[1])

    #         # x = 
    #         x = x + dx
    #         y = y + dy

    #         # Check cost at (x,y). Row-major means y is first!
    #         if costmap.shape[0] < 1:
    #             self.get_logger().warn("Costmap was empty")
    #             return
    #         cost = costmap[int(y), int(x)]
    #         if cost > COST_CUTOFF:
    #             return

    #         total_cost += cost

    #         segment_poses.append([
    #             x, y, steering_angle
    #         ])
    #         end_pose = segment_poses[-1]
    #         current_length += res

    #         idx += 1

    #     path = CostedPath()
    #     path.poses = segment_poses
    #     path.cost = total_cost

    #     return path

    def getSegment(self, inital_pose: np.ndarray, steering_angle, segment_length: float, res: float, costmap) -> CostedPath:
        end_pose = np.copy(inital_pose)
        segment_poses = []
        current_length = 0.0
        total_cost = 0

        idx = 0

        dx = (segment_length * np.cos(steering_angle)) / res
        dy = (segment_length * np.sin(steering_angle)) / res

        while current_length < segment_length:
            end_heading = end_pose[2]

            x, y = (res * np.cos(end_heading) + end_pose[0],
                    res * np.sin(end_heading) + end_pose[1])

            # x = 

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

            idx += 1

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
            return

        #for angle in np.linspace(-MAX_TURN_ANGLE, MAX_TURN_ANGLE, num_branches):
        for angle in MAX_TURN_ANGLE*np.power(np.linspace(-1, 1, num_branches),3):
            adjusted_angle = angle
            # if angle > 0:
            #     adjusted_angle = np.power(angle, 1.5)
            # else:
            #     adjusted_angle = np.power(abs(angle), 1.5) * -1
            self.generatePaths(depth-1, new_path, adjusted_angle, segment_length,
                               res, num_branches, results, result_costs, costmap)

    def startGeneration(self, costmap: np.ndarray, depth=7, segment_length=9.0, branches=7):
        # plt.figure(figsize=(8, 6), dpi=160)
        # plt.axes().set_aspect('equal')
        # print(f"Generating path with depth {depth}, seg len {segment_length}")

        path = CostedPath()
        path.poses = [[self.origin[0], self.origin[1], 0.0]]
        path.cost = 0
        results = []
        costs = []

        res = 1.0

        # The below loop creates the ROOT of our recursive tree
        # As a special case for the ROOT only, we multiply the number of branches
        # generated in this step by a constant, allowings us to boost the resolution
        # of the path when we're close to the front of the car.
        # The closer we are to the front of the car, the more important a smooth path is!

        for angle in np.linspace(-MAX_TURN_ANGLE, MAX_TURN_ANGLE, branches):
            self.generatePaths(depth-1, path, angle, segment_length,
                               res, branches, results, costs, costmap)

        # results = np.asarray(results)
        # print(results)
        # plt.imshow(costmap, origin='lower')
        # plt.xlim((25,75))
        # plt.ylim((60,90))
        return results
    
    def publishLookaheadMarker(self, radius, pose):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.clock
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.CYLINDER

        marker.action = Marker.ADD

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.2

        color = ColorRGBA()
        color.a = 0.3
        color.g = 1.0
        color.b = 1.0
        marker.color = color

        self.barrier_marker_pub.publish(marker)

        marker.id = 1
        marker.type = Marker.ARROW

        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 0.8
        marker.scale.z = 0.3

        color = ColorRGBA()
        color.a = 0.7
        color.g = 1.0
        color.b = 0.6
        marker.color = color

        pt_a = Point()
        marker.points.append(pt_a)

        pt_b = Point()
        pt_b.x = pose[0]
        pt_b.y = pose[1]
        marker.points.append(pt_b)

        self.barrier_marker_pub.publish(marker)
        # print("Lookahead published!")

    def publishBarrierMarker(self, pose):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.clock
        marker.ns = 'barrier'
        marker.id = 0
        marker.type = Marker.CUBE

        marker.action = Marker.ADD
        marker.pose.position.x = pose[0] * 0.4 - 20
        marker.pose.position.y = pose[1] * 0.4 - 30
        marker.pose.position.z = 1.0

        marker.pose.orientation.z = np.sin(pose[2]/2)
        marker.pose.orientation.w = np.cos(pose[2]/2)
        marker.scale.x = 0.2
        marker.scale.y = 2.0
        marker.scale.z = 0.2

        color = ColorRGBA()
        color.a = 0.8
        color.r = 1.0
        marker.color = color

        self.barrier_marker_pub.publish(marker)
        # print("Barrier published!")

    def removeBarrierMarker(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.clock
        marker.ns = 'barrier'
        marker.id = 0
        marker.type = Marker.CUBE

        marker.action = Marker.DELETE

        self.barrier_marker_pub.publish(marker)

    def initStatusMsg(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = self.get_name()

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)

        status.values.append(stamp)

        return status

    # def findPath(self,costmap, cost_threshold=20):
        

    # def costMapCb2(self, msg: OccupancyGrid):
    #     if msg.info.height == 0 or msg.info.width == 0:
    #         self.get_logger().warning("Incoming cost map dimensions were zero.")
    #         return
        
    #     costmap = np.asarray(msg.data, dtype=np.int8).reshape(
    #         msg.info.height, msg.info.width)
        
    #     lookahead_distance = 6.0 #min(max(self.speed * 3.0, 2.0), 20.) # meters



    def costMapCb(self, msg: OccupancyGrid):
        start = time.time()

        ALPHA = 0.5

        status = self.initStatusMsg()

        if msg.info.height == 0 or msg.info.width == 0:
            self.get_logger().warning("Incoming cost map dimensions were zero.")
            return

        costmap = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        results = self.startGeneration(
            costmap, depth=DEPTH, segment_length=STEP_LEN + 2. * self.speed, branches=N_BRANCHES)

        min_cost = 100000
        best_path: CostedPath = None

        barrier_idxs = []

        for result in results:

            # ADD BARRIER PROXIMITY COST
            # (only if car is stopped)
            if (self.speed < 2.0):
                barrier_idx = self.getBarrierIndex(
                    result, self.speed_costmap)
                MAX_IDX = 36
                result.cost += (MAX_IDX-barrier_idx)*50

            barrier_idxs.append(result.cost)
            if result.cost < min_cost:
                min_cost = result.cost
                best_path = result

        # plt.hist(barrier_idxs)
        # plt.show()

        result_msg = Path()
        result_msg.header.frame_id = "base_link"
        result_msg.header.stamp = self.clock

        if best_path is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "Could not find viable path. Likely too far off course."
            self.get_logger().error("Could not find viable path")
            self.status_pub.publish(status)
            path = CostedPath()
            path.poses = [[self.origin[0], self.origin[1], 0.0]]
            path.cost = 0
            best_path = path

        # convert poses from cost map grid to distances in meters in base_link
        poses_np = np.asarray(best_path.poses)
        poses_np_bl = np.copy(poses_np)
        poses_np_bl[:,0] -= (self.origin[0] + 5)
        poses_np_bl[:,1] -= self.origin[1]
        poses_np_bl[:,0:2] *= 0.4

        max_curvature = np.max(np.abs(poses_np[:,2]))
        # print(max_curvature)

        lookahead_distance = 6.0 #min(max(self.speed * 3.0, 2.0), 20.) # meters

        # print(lookahead_distance)
            
        barrier_idx = self.getBarrierIndex(best_path, self.speed_costmap)
        if barrier_idx == len(best_path.poses) - 1:
            # No barrier in front of us! Set the distance to be large.
            self.removeBarrierMarker()
            distance_from_barrier = 40.0
        else:
            barrier_pose = best_path.poses[barrier_idx]

            distance_from_barrier = np.linalg.norm(
                [barrier_pose[0] * 0.4 - (self.origin[0] * 0.4), barrier_pose[1] * 0.4 - 30])

            self.publishBarrierMarker(barrier_pose)

        lookahead_pose = None
        for pose in poses_np_bl[:]:
            if np.sqrt(pose[0]**2 + pose[1]**2) > lookahead_distance:
                lookahead_pose = pose
                break

        if lookahead_pose is None:
            lookahead_pose = poses_np_bl[-1]


        # print(f"Lookahead pose is ({lookahead_pose[0]}, {lookahead_pose[1]})")
        self.publishLookaheadMarker(lookahead_distance, lookahead_pose)

        for pose in best_path.poses:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "base_link"
            pose_msg.header.stamp = self.clock
            pose_msg.pose.position.x = pose[0] * 0.4 - 20
            pose_msg.pose.position.y = pose[1] * 0.4 - 30
            # TODO: Add heading (pose[2]?)
            result_msg.poses.append(pose_msg)
        if len(result_msg.poses) == 0:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "base_link"
            pose_msg.header.stamp = self.clock
            pose_msg.pose.position.x = 50 * 0.4 - 20
            pose_msg.pose.position.y = 75 * 0.4 - 30
            # TODO: Add heading (pose[2]?)
            result_msg.poses.append(pose_msg)
            self.get_logger().info('Published singleposition')
        command = VehicleControl()
        if len(best_path.poses) < 4:
            return
        #command.steer = ((best_path.poses[4][2] + best_path.poses[5][2] + best_path.poses[6][2])/3.0) * -2.7  # First steering value
        
        # x = np.sum((np.asarray(best_path.poses)[1:11,2] * np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1])))/10.0
        # command.steer = (x * -1.5)**3 - 1.6*x
        
        #target_steer = np.arctan2(lookahead_pose[1], lookahead_pose[0]) * -1 / 0.46

        target_steer = np.arctan2(-lookahead_pose[1], lookahead_pose[0])
        
        # target_steer should be within the feasible turning of the vehicle, normalize between -1,1
        target_steer = max( min(target_steer,MAX_TURN_ANGLE), -MAX_TURN_ANGLE ) / MAX_TURN_ANGLE

        DT = 0.2 # s
        ALPHA = 1.0 # min(10 - 1 * self.speed, 10.0)

        command.steer = self.previous_steer + ALPHA * (target_steer - self.previous_steer) * DT

        command.steer = target_steer

        # if command.steer > 1.0:
        #     command.steer = 1.0
        # elif command.steer < -1.0:
        #     command.steer = -1.0

        

        command.header.stamp = self.clock

        

        MAX_SPEED = 0.5 #5.0

        distance_from_barrier -= 7

        if distance_from_barrier <= 10.0 and distance_from_barrier >= -5:
            MAX_SPEED = distance_from_barrier / 3
            if distance_from_barrier <= 3.0:
                MAX_SPEED = -1.
            print(f"MAX {MAX_SPEED}, BAR: {distance_from_barrier}")

        MAX_SPEED = min(MAX_SPEED, 5.0-max_curvature * 2)

        target_speed = 1.5 # MAX_SPEED # m/s, ~10mph

        target_speed_msg = VehicleSpeed()
        target_speed_msg.speed = target_speed
        self.target_speed_pub.publish(target_speed_msg)

        pid_error = target_speed - self.speed
        # self.get_logger().info(f"Steer/Pose: {command.steer}/{((best_path.poses[4][2] + best_path.poses[5][2] + best_path.poses[6][2])/3.0)}")
        # print(f"Speed: {self.speed} / {target_speed}")
        if pid_error > 0:
            command.throttle = min(pid_error *0.5, 0.6)
            command.brake = 0.0
        #else:
        elif pid_error <= -1:
            command.brake = pid_error *0.6 *-1.0
            command.throttle = 0.0
            
        # print(f"Brake: {command.brake}")
        # if pid_error > 3.0:
        #     command.throttle = 0.5
        #     command.brake = 0.0
        # elif pid_error > 0.5:
        #     command.throttle = 0.4
        #     command.brake = 0.0
        # elif pid_error > -1.0:
        #     # Coast if speeding by ~2 mph
        #     command.throttle = 0.0
        #     command.brake = 0.0
        # elif pid_error > -2.0:
        #     # Brake slightly if speeding by ~5 mph
        #     command.throttle = 0.0
        #     command.brake = 0.3
        # else:
        #     status.level = DiagnosticStatus.WARN
        #     status.message = f"{int(abs(pid_error))} m/s over limit"
        #     command.throttle = 0.0
        #     command.brake = 0.8

        # if self.speed > MAX_SPEED + 0.5:
        #     command.brake = 0.4
        # MAX_SPEED = 1.0
        # if self.speed > MAX_SPEED:
        #     command.throttle = 0.0
        # else:
        #     command.throttle = 0.4

        if self.current_mode == Mode.AUTO:
            # print("AUTO")
            self.command_pub.publish(command)

        self.previous_steer = command.steer

        self.path_pub.publish(result_msg)

        self.status_pub.publish(status)
        self.last_status_time = time.time()

        # print(f"Done in {time.time() - start}!")

    def goForward(self, msg: OccupancyGrid):
        command = VehicleControl()
        command.header.stamp = self.clock
        command.steer = 0.0
        
        target_speed = 1.5 # MAX_SPEED # m/s, ~10mph

        target_speed_msg = VehicleSpeed()
        target_speed_msg.speed = target_speed
        self.target_speed_pub.publish(target_speed_msg)

        pid_error = target_speed - self.speed
        if pid_error > 0:
            command.throttle = min(pid_error *0.5, 0.6)
            command.brake = 0.0
        elif pid_error <= -1:
            command.brake = pid_error *0.6 *-1.0
            command.throttle = 0.0
        else:
            command.throttle = 0.0
            command.brake = 0.0

        self.command_pub.publish(command)

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
