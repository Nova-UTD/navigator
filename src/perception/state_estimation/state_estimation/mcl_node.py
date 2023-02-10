'''
Package: state_estimation
   File: mcl_node.py
 Author: Will Heitman (w at heit dot mn)

A ROS2 wrapper around MCL (see mcl.py)

Subscribes to:
- LiDAR (sensor_msgs/PointCloud2)
- Speedometer (carla_msgs/CarlaSpeedometer)
- GNSS odom (nav_msgs/Odometry)

Reads:
- .pcd map file from disk

Publishes:
- MCL result (geometry_msgs/PoseWithCovarianceStamped)
    - Minimum frequency: 2 Hz
'''

import math
import numpy as np
import rclpy
import ros2_numpy as rnp
import struct
import time

# Message definitions
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nova_msgs.srv import GetLandmarks
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster

from .mcl import MCL

import matplotlib.pyplot as plt


class MCLNode(Node):

    def __init__(self):
        super().__init__('mcl_node')

        self.previous_result = None
        self.clock = Clock()
        self.filter = None
        self.gnss_pose = None
        self.last_update_time = time.time()
        self.old_gnss_pose = None
        self.grid: np.array = None
        self.imu = None
        self.speed: float = 0.0  # m/s

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/lidar/semantic', self.cloud_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/odometry/gnss_processed', self.gnss_cb, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/carla/hero/imu', self.imu_cb, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.map_cb, 10)

        self.speed_sub = self.create_subscription(
            CarlaSpeedometer, '/carla/hero/speedometer', self.speed_cb, 1)

        self.particle_cloud_pub = self.create_publisher(
            PointCloud2, '/mcl/particles', 10)
        # landmark_request = GetLandmarks.Request()
        # self.get_logger().info("Sending request")
        # self.landmarks: GetLandmarks.Response = self.landmark_client.call(
        #     landmark_request)
        # self.get_logger().info(
        #     f"Received {len(self.landmarks.speed_limit_signs)} speed limit signs")

        self.landmark_client = self.create_client(
            GetLandmarks, 'get_landmarks')

        # landmark_request = GetLandmarks.Request()
        # self.get_logger().info("Sending request")
        # self.landmarks: GetLandmarks.Response = self.landmark_client.call(
        #     landmark_request)
        # self.get_logger().info(
        #     f"Received {len(self.landmarks.speed_limit_signs)} speed limit signs")

        self.tf_broadcaster = TransformBroadcaster(self)

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def imu_cb(self, msg: Imu):
        self.imu = msg

    def speed_cb(self, msg: CarlaSpeedometer):
        self.speed = msg.speed

    def getMotionDelta(self, current_gnss_pose, old_gnss_pose, speed: float, dt):

        # Start by calculating heading change
        delta = np.zeros((3))
        delta[2] = current_gnss_pose[2] - old_gnss_pose[2]
        # Wrap heading to [0, 2*pi]
        delta[2] %= 2*np.pi

        # Now calculate displacement via speedometer and dt
        displacement = speed * dt  # result in meters
        delta[0] = displacement * np.cos(current_gnss_pose[2])
        delta[1] = displacement * np.sin(current_gnss_pose[2])

        return delta

    def publish_particle_cloud(self):
        particles = self.filter.particles
        N = len(particles)
        cloud_array = np.zeros(N, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('i', np.float32)
        ])
        cloud_array['x'] = particles[:, 0]
        cloud_array['y'] = particles[:, 1]
        cloud_array['z'] = particles[:, 2]
        cloud_array['i'] = self.filter.weights

        msg: PointCloud2 = rnp.msgify(PointCloud2, cloud_array)
        msg.header.stamp = self.clock.clock
        msg.header.frame_id = 'map'
        self.particle_cloud_pub.publish(msg)

    def cloud_cb(self, msg: PointCloud2):
        """Update our filter with the latest observations and publish the result

        Args:
            msg (PointCloud2): Our latest observations from e.g. road segmentation
        """

        # Wait until filter is created
        if self.filter is None or self.imu is None:
            return

        if self.previous_result is None:
            self.previous_result = np.zeros((3))

        # Change in pose since last filter update
        dt = time.time() - self.last_update_time
        heading_rate = self.imu.angular_velocity.z

        # The filter accepts clouds as a (N,2) array. Format accordingly.
        cloud_formatted = rnp.numpify(msg)

        ROAD_ID = 4286595200
        TRAFFIC_LIGHT_ID = 4294617630
        POLE_ID = 4288256409

        cloud_formatted = cloud_formatted[np.logical_or(cloud_formatted['rgb'] == POLE_ID,
                                                        np.logical_or(cloud_formatted['rgb'] == ROAD_ID,
                                                                      cloud_formatted['rgb'] == TRAFFIC_LIGHT_ID))]

        cloud = np.vstack(
            (cloud_formatted['x'], cloud_formatted['y'], cloud_formatted['rgb'])).T

        # Filter to road points only

        # step() is the critical function that feeds data into the filter
        # and returns a pose and covariance.

        clock_seconds = self.clock.clock.sec + self.clock.clock.nanosec * 1e-9

        result_pose, pose_variance = self.filter.step(
            [heading_rate, self.speed], clock_seconds, cloud, self.gnss_pose, self.grid)

        self.last_update_time = time.time()
        self.publish_particle_cloud()

        # self.get_logger().info(f"Diff: {str(self.gnss_pose-result_pose)}")

        # Turn our filter result into a transform
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.header.stamp = self.clock.clock
        t.child_frame_id = 'hero'
        t.transform.translation.x = result_pose[0]
        t.transform.translation.y = result_pose[1]
        t.transform.rotation.w = math.cos(result_pose[2] / 2)
        t.transform.rotation.z = math.sin(result_pose[2] / 2)

        # Broadcast our transform
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info("BROADCASTING")

        # Cache our gnss_pose to calculate the delta later
        self.previous_result = result_pose

    def gnss_cb(self, msg: Odometry):
        pose_msg = msg.pose.pose
        yaw = 2*math.asin(pose_msg.orientation.z)
        self.old_gnss_pose = self.gnss_pose
        self.gnss_pose = np.array([
            pose_msg.position.x,
            pose_msg.position.y,
            yaw
        ])

    def map_cb(self, msg: OccupancyGrid):
        data = np.rot90(np.array(msg.data).reshape(151, 151), k=1, axes=(1, 0))
        # plt.imshow(data, origin='lower')
        # print("Showing!")
        # plt.show()

        if self.gnss_pose is None:
            return  # Wait for initial guess from GNSS

        self.grid = np.asarray(msg.data,
                               dtype=np.int8).reshape(msg.info.height, msg.info.width)

        if self.filter is not None:
            return

        origin = msg.info.origin.position
        res = msg.info.resolution

        clock_seconds = self.clock.clock.sec + self.clock.clock.nanosec * 1e-9
        self.filter = MCL(clock_seconds, res, initial_pose=self.gnss_pose,
                          map_origin=np.array([origin.x, origin.y]), N=50)

        self.get_logger().info("MCL filter created")


def main(args=None):
    rclpy.init(args=args)

    mcl_node = MCLNode()

    rclpy.spin(mcl_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mcl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
