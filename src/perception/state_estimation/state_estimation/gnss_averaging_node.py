'''
Package: state_estimation
   File: gnss_averaging_node.py
 Author: Will Heitman (w at heit dot mn)

Simple state machine that combined GNSS averaging with dead reckoning for CARLA.

Uses a two-state machine:

1. Stationary state: Node collects GNSS estimates into an array while speed <0.1 m/s
2. Moving state: Node dead reckons to update current_pose.

On 1->2 transition: GNSS estimates in array are average to form updated current_pose,
                    only if car was in (1) for >5 seconds.

Subscribes to: 
- Raw GNSS (Odometry)
- Speedometer

Publishes:
- Corrected pose (Odometry)
- Diagnostic status
'''

import math
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster

# Message definitions
from navigator_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


class GnssAveragingNode(Node):

    def __init__(self):
        super().__init__('gnss_averaging_node')

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 1)

        self.imu_sub = self.create_subscription(
            Imu, '/carla/hero/imu', self.imu_cb, 1)

        self.speedometer_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedometer_cb, 1)

        self.raw_gnss_sub = self.create_subscription(
            Odometry, '/gnss/odometry_raw', self.raw_gnss_cb, 10
        )

        self.true_pose_sub = self.create_subscription(
            PoseStamped, '/true_pose', self.true_pose_cb, 1)

        self.smoothed_gnss_sub = self.create_subscription(
            Odometry, '/gnss/odometry_processed', self.smoothed_gnss_cb, 10
        )

        self.result_pub = self.create_publisher(
            Odometry, '/odometry/processed', 1)

        self.diagnostic_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.diagnostic_pub_timer = self.create_timer(
            0.25, self.publish_diagnostics)

        self.speed = 0.0  # rad/s
        self.clock: Time = None
        self.last_update_time = None
        self.heading_rate = 0.0  # rad/s
        self.is_stationary = True
        self.cached_gnss_poses = []
        self.current_pose = None  # [x, y, heading]
        self.yaw = 0.0

        # This variable describes whether or not our pose was recently refreshed by average GNSS
        # If the car has not recently been stationary for a significant period of time,
        # this will be false, and our result will likely be inaccurate.
        self.averaging_is_fresh = False
        self.last_refresh_time = -1.0

        # Diagnostic state
        self.diag_state = 'OK'

    def smoothed_gnss_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation

        if q.z < 0:
            self.yaw = abs(2*np.arccos(q.w) - 2 * np.pi)
        else:
            self.yaw = 2 * np.arccos(q.w)

        if self.yaw > np.pi:
            self.yaw -= 2 * np.pi

    def true_pose_cb(self, msg: PoseStamped):
        if self.current_pose is None:
            return
        dx = abs(msg.pose.position.x - self.current_pose[0])
        dy = abs(msg.pose.position.y*-1 - self.current_pose[1])
        error = math.sqrt(dx**2 + dy**2)
        print(f"Error: {error}")

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'localization'
        status.level = DiagnosticStatus.OK

        if self.clock is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "Clock not yet received."
            self.diagnostic_pub.publish(status)
            return

        current_time = self.clock.sec + self.clock.nanosec*1e-9
        dt = current_time - self.last_update_time
        if dt > 0.3:  # seconds
            status.level = DiagnosticStatus.ERROR
            status.message = f"Localization last updated {dt} seconds ago."

        last_refresh_gap = current_time - self.last_refresh_time
        if last_refresh_gap > 15.0 and not self.is_stationary:
            # It's been 15 seconds since our car was last stationary
            # and our pose was reset from averaged GNSS data
            status.level = DiagnosticStatus.WARN
            status.message = f"Localization last refreshed {last_refresh_gap} seconds ago. Result likely inaccurate."

        elif last_refresh_gap > 30.0 and not self.is_stationary:
            # It's been 30 seconds since our car was last stationary
            # and our pose was reset from averaged GNSS data. We need to stop to refresh.
            status.level = DiagnosticStatus.ERROR
            status.message = f"Localization last refreshed {last_refresh_gap} seconds ago. Result likely highly inaccurate."

        else:
            status.level = DiagnosticStatus.OK
            status.message = "Localization operating normally."

        self.diagnostic_pub.publish(status)

    def raw_gnss_cb(self, msg: Odometry):
        if self.clock is None:
            self.get_logger().debug('Clock not yet received. Skipping update.')
            return

        if self.current_pose is None:
            self.current_pose = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                2*math.asin(msg.pose.pose.orientation.z)
            ])
            self.get_logger().info("Initializing to first GNSS pose")

        current_time = self.clock.sec + self.clock.nanosec*1e-9

        if self.speed < 0.1:
            self.is_stationary = True

            gnss_pose_arr = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self.yaw
            ])

            if gnss_pose_arr[2] > math.pi:
                gnss_pose_arr[2] -= 2*math.pi

            self.cached_gnss_poses.append(gnss_pose_arr)

            if len(self.cached_gnss_poses) % 10 == 0:
                print(f"Cache now has {len(self.cached_gnss_poses)} poses")

            if len(self.cached_gnss_poses) > 10:
                cached_poses = np.array(self.cached_gnss_poses)
                self.current_pose = np.mean(cached_poses, axis=0)

            # if len(self.cached_gnss_poses) > 50:
            #     self.cached_gnss_poses = self.cached_gnss_poses[:-50]

        elif self.is_stationary:
            # We're over the stationary speed, so update our state
            self.is_stationary = False

            # Calculate average pose
            cached_poses = np.array(self.cached_gnss_poses)
            average_pose = np.mean(cached_poses, axis=0)

            if len(cached_poses) < 2:
                return

            average_pose[2] %= 2*math.pi  # Wrap to [0, 2*pi]

            self.get_logger().info(
                f"Pose was corrected by {average_pose-self.current_pose}")
            # plt.scatter(cached_poses[:, 0], cached_poses[:, 1], c='blue')
            # plt.scatter(average_pose[0], average_pose[1], c='green')
            # plt.show()

            self.current_pose = average_pose

            self.last_refresh_time = current_time

            self.cached_gnss_poses.clear()

        else:
            # Dead reckon
            dt = current_time - self.last_update_time
            # print("Dead reckoning with dt = {:.2f}".format(dt))

            # Update heading by integrating heading rate
            self.current_pose[2] += self.heading_rate * dt * 1.2
            self.current_pose[2] %= 2 * np.pi
            # self.current_pose[2] = 2*math.asin(msg.pose.pose.orientation.z)

            # move in the (noisy) commanded direction
            dist = (self.speed * dt)
            self.current_pose[0] += np.cos(self.current_pose[2]) * dist
            self.current_pose[1] += np.sin(self.current_pose[2]) * dist

        # Get current time in seconds
        self.last_update_time = current_time

        result_msg = Odometry()
        result_msg.header = msg.header
        result_msg.child_frame_id = 'hero'
        result_msg.pose.pose.position.x = self.current_pose[0]
        result_msg.pose.pose.position.y = self.current_pose[1]
        result_msg.pose.pose.position.z = 0.0  # TODO: Don't assume z=0
        result_msg.pose.pose.orientation.w = math.cos(self.current_pose[2]/2)
        result_msg.pose.pose.orientation.z = math.sin(self.current_pose[2]/2)
        result_msg.twist.twist.linear.x = self.speed
        result_msg.twist.twist.angular.z = self.heading_rate

        self.result_pub.publish(result_msg)

        # Publish our map->base_link tf
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'hero'
        transl = Vector3()
        transl.x = self.current_pose[0]
        transl.y = self.current_pose[1]
        transl.z = 0.0  # TODO: Stop assuming flat surface
        t.transform.translation = transl
        t.transform.rotation = result_msg.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t)

    def clock_cb(self, msg: Clock):
        if self.last_update_time is None:
            self.last_update_time = msg.clock.sec + msg.clock.nanosec*1e-9

        self.clock = msg.clock

    def imu_cb(self, msg: Imu):
        self.heading_rate = msg.angular_velocity.z

    def speedometer_cb(self, msg: CarlaSpeedometer):
        self.speed = msg.speed


def main(args=None):
    rclpy.init(args=args)

    gnss_averaging_node = GnssAveragingNode()

    rclpy.spin(gnss_averaging_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gnss_averaging_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
