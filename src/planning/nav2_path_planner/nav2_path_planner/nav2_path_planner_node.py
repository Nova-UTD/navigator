'''
Package:   nav2_path_planner
Filename:  nav2_path_planner_node.py
Author:    Justin Ruths

Plans a path using Nav2, specifically the SMAC Hybrid Path Planner

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

from nav2_simple_commander.robot_navigator import BasicNavigator
from ros2_numpy.occupancy_grid import occupancygrid_to_numpy, numpy_to_occupancy_grid
from tf_transformations import quaternion_from_euler

from visualization_msgs.msg import Marker

from skimage.draw import line

from PIL import Image

from matplotlib.patches import Rectangle

NAV2_MAP_FRAME = 'nav2_map'
NAV2_ODOM_FRAME = 'nav2_odom'
NAV2_BASELINK_FRAME = 'nav2_base_link'
NAV2_MAP_PGM = "/navigator/src/planning/nav2_path_planner/nav2_path_planner/nav2_map.pgm"
NAV2_MAP_PARAMS = '/navigator/src/planning/nav2_path_planner/nav2_path_planner/nav2_map.param.yaml'

class Nav2PathPlanner(Node):
    def __init__(self):
        super().__init__('nav2_path_planner_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_broadcast_time = None

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)
        self.clock = Clock()

        self.navigator = BasicNavigator(namespace='nav2')
        
        # Nav2 wants an initial pose even though we won't ever use it.
        self.zero_pose = PoseStamped()
        self.zero_pose.header.frame_id = NAV2_BASELINK_FRAME
        self.zero_pose.header.stamp = self.clock.clock
        self.zero_pose.pose.position.x = 0.0
        self.zero_pose.pose.position.y = 0.0
        self.zero_pose.pose.orientation.z = 0.0
        self.zero_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.zero_pose)

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

        self.timer = self.create_timer(0.05, self.broadcast_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.path_timer = self.create_timer(0.1, self.generate_path, callback_group=MutuallyExclusiveCallbackGroup())

        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/planning/look_ahead_marker', 1)

        ######## TODO: Just here until controller is in place:
        self.command_pub = self.create_publisher(
            VehicleControl, '/vehicle/control', 1)
        self.speed = 0.0
        self.speed_sub = self.create_subscription(
            VehicleSpeed, '/speed', self.speedCb, 1)
        ########

    def speedCb(self, msg: VehicleSpeed):
        self.speed = msg.speed

    # Goal is to establish map->nav2_map->nav2_odom->nav2_base_link such that nav2_map is located at the same spot as Navigator's base_link
    def broadcast_timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time(), rclpy.time.Duration(seconds=5.0) )
        except(LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning("Have not received map->base_link yet, cannot connect Nav2 TF to base_link.")
            self.get_logger().warning(str(e))
            return

        # Use the same transform as navigator's map->base_link as the transform map->Nav2's map
        t.header.frame_id = 'map'
        t.child_frame_id = NAV2_MAP_FRAME

        self.tf_broadcaster.sendTransform(t)
        self.last_broadcast_time = t.header.stamp

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
        if self.last_broadcast_time is None:
            self.get_logger().warning("Have not boadcast Nav2 transforms yet...")
            return
        if np.sqrt(self.path_goal.pose.position.x**2 + self.path_goal.pose.position.y**2) < 0.5:
            self.get_logger().info("Vehicle has reached the goal!")
            return

        try: # may fail if transforms have not buffered
            goal_pose = PoseStamped()
            # the goal pose is in Navigator's base_link, so shift it over to nav2_base_link
            goal_pose.header.frame_id = NAV2_BASELINK_FRAME
            goal_pose.header.stamp = self.path_goal.header.stamp
            goal_pose.pose = self.path_goal.pose

            # costmap for planning
            cmap = np.asarray(self.costmap.data, dtype=np.float16).reshape(
                        self.costmap.info.height, self.costmap.info.width)
            
            # rescale up to 255 for grayscale image
            img = Image.fromarray(np.uint8( 255.0 - np.flipud(cmap) * 255.0/100.0) , 'L')
            img.save(NAV2_MAP_PGM)
            # tell Nav2 to update the map to the one we just saved
            self.navigator.changeMap(NAV2_MAP_PARAMS)

            # Pass to Nav2
            path = self.navigator.getPath(self.zero_pose, goal_pose, planner_id='GridBased', use_start=False)
            if path is None:
                self.get_logger().warning("!!! Nav2 returned a path as None !!!")
                return
            self.get_logger().debug("Path Returned with %i elements" % len(path.poses))

            # smoothing is automatically done by Nav2, no need to do it again (leave commented out)
            # smoothed_path = self.navigator.smoothPath(path)

            # transform from nav2_map to nav2_base_link - but the transform is zero translation and zero rotation, so nothing to do (leave commented out)
            # path_base_link = self.path_transform(path,NAV2_BASELINK_FRAME)  # Don't need this since there is no transform to do
            path_base_link = path
            # now go back to Navigator to its base_link, simply by relabeling
            # also sometimes the path is generated with an offset bias in the first path pose...
            # so we shift the whole path over in that case
            biasx = -1.0*path_base_link.poses[0].pose.position.x
            biasy = -1.0*path_base_link.poses[0].pose.position.y
            path_base_link.header.frame_id = 'base_link'
            for i in range(len(path_base_link.poses)):
                path_base_link.poses[i].header.frame_id = 'base_link'
                path_base_link.poses[i].pose.position.x += biasx
                path_base_link.poses[i].pose.position.y += biasy

            self.path_pub.publish(path_base_link)

            # TODO: Revist this status stuff once 
            # status.level = DiagnosticStatus.ERROR
            # status.message = "Could not find viable path. Likely too far off course."
            # self.get_logger().error("Could not find viable path")
            # self.status_pub.publish(status)

            # self.status_pub.publish(status)
            # self.last_status_time = time.time()

            ############################ TODO: Just for testing until controller is in place:
            ############################ remove publish_marker function below at the same time
            lookahead_distance = 4
            lookahead_pose = None
            for p in path_base_link.poses:
                if np.sqrt(p.pose.position.x**2 + p.pose.position.y**2) > lookahead_distance:
                    lookahead_pose = p
                    break

            if lookahead_pose is None:
                lookahead_pose = path_base_link.poses[-1]

            self.publish_marker(lookahead_pose,(0.0,0.2,1.0),self.lookahead_marker_pub)

            command = VehicleControl()
            command.header.stamp = self.clock.clock
            command.header.frame_id = 'base_link'
            target_steer = np.arctan2(-lookahead_pose.pose.position.y, lookahead_pose.pose.position.x)
            command.steer = target_steer
            target_speed = 1.0 # MAX_SPEED # m/s, ~10mph
            pid_error = target_speed - self.speed
            command.throttle = 0.0
            command.brake = 0.0
            # self.get_logger().info(f"Steer/Pose: {command.steer}/{((best_path.poses[4][2] + best_path.poses[5][2] + best_path.poses[6][2])/3.0)}")
            # print(f"Speed: {self.speed} / {target_speed}")
            if pid_error > 0:
                command.throttle = min(pid_error *0.5, 0.3)
                command.brake = 0.0
            #else:
            elif pid_error <= -1:
                command.brake = pid_error *0.5 *-1.0
                command.throttle = 0.0
            
            self.command_pub.publish(command)
            ##############################

        except(Exception) as e:
            self.get_logger().info('Issue generating path...')
            self.get_logger().info(str(e))
            self.get_logger().info('line number: %s' % str(e.__traceback__.tb_lineno))
            return

    def get_transform(self, source_frame, target_frame):
        ''' Lookup latest transform between source_frame and target_frame from the buffer '''
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), rclpy.time.Duration(seconds=5.0) )

        except(LookupException, ConnectivityException, ExtrapolationException) as e:
            raise Exception('Cannot find transformation from %s to %s' % (source_frame,target_frame))
            #self.get_logger().warning('Cannot find transformation from %s to %s' % (source_frame,target_frame))
        return trans     # Type: TransformStamped

    def pose_transform(self, pose, target_frame):
        ''' pose: will be transformed to target_frame '''
        trans = self.get_transform( pose.header.frame_id, target_frame )

        pose2 = PoseStamped()
        pose2.header.frame_id = target_frame
        pose2.header.stamp = pose.header.stamp
        pose2.pose = do_transform_pose(pose.pose, trans)
        
        return pose2

    def path_transform(self, path, target_frame):
        ''' path: will be transformed to target_frame '''
        trans = self.get_transform( path.header.frame_id, target_frame )

        path2 = Path()
        path2.header.frame_id = target_frame
        path2.header.stamp = path.header.stamp

        for pose in path.poses:
            pose2 = PoseStamped()
            pose2.header.frame_id = target_frame
            pose2.header.stamp = pose.header.stamp
            pose2.pose = do_transform_pose(pose.pose, trans)
            path2.poses.append( pose2 )
        return path2
    
    # TODO: Don't need this after we remove the simple controller here
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
    node = Nav2PathPlanner()
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
