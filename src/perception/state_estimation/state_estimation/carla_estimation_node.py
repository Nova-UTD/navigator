'''
Package: map_management
   File: gnss_estimation_node.py
 Author: Will Heitman (w at heit dot mn)

Very simple node to convert raw GNSS odometry into a map->base_link transform.
'''

import math
import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node

from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster


class CarlaEstimationNode(Node):

    def __init__(self):
        super().__init__('gnss_estimation_node')

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/gnss', self.odom_cb, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/carla/hero/imu', self.imu_cb, 10
        )

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speedometer_cb, 10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odometry/processed', 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cached_imu = Imu()
        self.cached_speed = CarlaSpeedometer()

        # Make a queue of previous poses for our weighted moving average
        # where '10' is the size of our history
        self.history_size = 5
        self.previous_x_vals = np.zeros((self.history_size))
        self.previous_y_vals = np.zeros((self.history_size))
        self.previous_z_vals = np.zeros((self.history_size))
        self.wma_pose = Pose()

    def imu_cb(self, msg: Imu):
        # BUG: IMU orientation quaternion is whack.
        # x & y are (-.48,0.0) for north, (0.0,-.48) for east,
        # (.48,0.0) for south, and (0.0,.48) for west.

        # Fix orientation quat manually, assuming flat ground
        # This means that roll and pitch (quat x & y) will be zero
        heading_x = msg.orientation.x / -.48
        heading_y = msg.orientation.y / .48

        self.cached_imu = msg

    def speedometer_cb(self, msg: CarlaSpeedometer):
        self.cached_speed = msg

    def _update_odom_weighted_moving_average_(self, current_pos: Point):
        # Calculate noisy yaw from the change in position
        # old_pose = self.previous_poses[self.history_size-1]

        # Calculate the weighted moving average (WMA) for odometry
        # 1. Discard oldest reading and add newest to 'queue'

        self.previous_x_vals = np.roll(self.previous_x_vals, -1)
        self.previous_y_vals = np.roll(self.previous_y_vals, -1)
        self.previous_z_vals = np.roll(self.previous_z_vals, -1)
        self.previous_x_vals[self.history_size-1] = current_pos.x
        self.previous_y_vals[self.history_size-1] = current_pos.y
        self.previous_z_vals[self.history_size-1] = current_pos.z

        # 2. Generate weight array: [1,2,3,...,n]
        weights = np.arange(0, self.history_size)

        # 3. Use a numpy functions to handle the rest :->)
        wma_x = np.average(self.previous_x_vals, axis=0, weights=weights)
        wma_y = np.average(self.previous_y_vals, axis=0, weights=weights)
        wma_z = np.average(self.previous_z_vals, axis=0, weights=weights)

        wma_pose = Pose()
        wma_pose.position.x = wma_x
        wma_pose.position.y = wma_y
        wma_pose.position.z = wma_z
        wma_yaw = 0.0 # TODO: Calculate yaw

        # q = cos(theta) + sin(theta)(xi + yj + zk)
        # Set x, y = 0 s.t. theta = yaw
        wma_pose.orientation.w = math.cos(wma_yaw)
        wma_pose.orientation.x = 0.0
        wma_pose.orientation.y = 0.0
        wma_pose.orientation.z = math.sin(wma_yaw)

        self.wma_pose = wma_pose

    def _quat_to_yaw_(self, q: Quaternion):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return yaw

    def odom_cb(self, msg: Odometry):

        current_pos: Point = msg.pose.pose.position

        # 4. Form an odom message to store our result
        odom_msg = Odometry()

        # Copy the header from the GNSS odom message
        odom_msg.header = msg.header
        odom_msg.child_frame_id = msg.child_frame_id

        self._update_odom_weighted_moving_average_(current_pos)

        odom_msg.pose.pose = self.wma_pose

        # Publish our odometry message, converted from GNSS
        self.odom_pub.publish(odom_msg)

        self.get_logger().info("{}".format(str(self.previous_y_vals)))
        self.get_logger().info(f"CURRENT Y: {current_pos.y}")

        # Publish our map->base_link tf
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'hero'
        transl = Vector3()
        transl.x = self.wma_pose.position.x
        transl.y = self.wma_pose.position.y
        transl.z = self.wma_pose.position.z
        t.transform.translation = transl
        t.transform.rotation = self.wma_pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    carla_estimation_node = CarlaEstimationNode()

    rclpy.spin(carla_estimation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    carla_estimation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
