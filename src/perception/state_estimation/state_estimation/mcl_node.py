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
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster

from .mcl import MCL


class MCLNode(Node):

    def __init__(self):
        super().__init__('mcl_node')

        self.clock = Clock()
        self.filter = None
        self.gnss_pose = None
        self.old_gnss_pose = None
        self.grid: np.array = None

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/lidar/semantic', self.cloud_cb, 10)

        self.gnss_sub = self.create_subscription(
            Odometry, '/odometry/gnss_processed', self.gnss_cb, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.map_cb, 10)

        self.particle_cloud_pub = self.create_publisher(
            PointCloud2, '/mcl/particles', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def get_motion_delta(self, old_pose, current_pose):
        if old_pose is None:
            return np.zeros(3)

        delta = current_pose-old_pose
        # delta[2] *= -1  # Why? I don't know

        # Wrap heading to [0, 2*pi]
        delta[2] %= 2*np.pi
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
        if self.filter is None:
            return

        # Change in pose since last filter update
        delta = self.get_motion_delta(self.old_gnss_pose, self.gnss_pose)

        # The filter accepts clouds as a (N,2) array. Format accordingly.
        cloud_formatted = rnp.numpify(msg)
        cloud_formatted = cloud_formatted[cloud_formatted['c'] == 0]
        cloud = np.vstack((cloud_formatted['x'], cloud_formatted['y'])).T

        # Filter to road points only

        # step() is the critical function that feeds data into the filter
        # and returns a pose and covariance.

        print(cloud)
        result_pose, pose_variance = self.filter.step(
            delta, cloud, self.gnss_pose)
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
        self.old_gnss_pose = self.gnss_pose

    def gnss_cb(self, msg: Odometry):
        pose_msg = msg.pose.pose
        yaw = 2*math.asin(pose_msg.orientation.z)
        self.gnss_pose = np.array([
            pose_msg.position.x,
            pose_msg.position.y,
            yaw
        ])

    def map_cb(self, msg: OccupancyGrid):
        if self.gnss_pose is None:
            return  # Wait for initial guess from GNSS

        self.grid = np.asarray(msg.data,
                               dtype=np.int8).reshape(msg.info.height, msg.info.width)

        origin = msg.info.origin.position
        res = msg.info.resolution
        self.filter = MCL(self.grid, res, initial_pose=self.gnss_pose,
                          map_origin=np.array([origin.x, origin.y]))

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
