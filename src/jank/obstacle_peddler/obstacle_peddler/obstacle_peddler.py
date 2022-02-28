import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
import math
from os.path import exists

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2

class ObstaclePeddler(Node):
    def __init__(self):
        super().__init__('obstacle_peddler')

        self.MAX_HEIGHT = 1.0 # meters
        self.MIN_HEIGHT = 0.5 # meters
        self.MIN_X = 5.0 # meters FROM REAR AXLE
        self.TARGET_DISTANCE = 5.0 # meters FROM FRONT BUMPER
        self.VEHICLE_LENGTH = 4.3 # meters
        self.BANG_ON_VALUE = 1.0 # Some established value coordinated with throttle interface
        self.STEERING_K = 1.0 # Fix this

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_front/velodyne_points',
            self.front_lidar_cb,
            10
        )

        self.obstacle_roi_pub = self.create_publisher(PointCloud2, '/obstacles/roi', 10)
        self.throttle_cmd_pub = self.create_publisher(Float32, '/command/throttle', 10)
        self.steering_cmd_pub = self.create_publisher(Float32, '/command/steering', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_to_bl_tf = TransformStamped()

    def getLidarToBlTransform(self, lidar_frame: str):
        try:
            self.lidar_to_bl_tf = self.tf_buffer.lookup_transform(
                'base_link',
                lidar_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {dest_frame} to {lidar_frame}: {ex}')
            return

    def filterPoints(self, pts):
        # Remove nan values
        pts = pts[np.logical_not(np.isnan(pts['x']))]

        translation = self.lidar_to_bl_tf.transform.translation
        # pts['x'] += translation.x
        # pts['y'] += translation.y
        # pts['z'] += translation.z
        # self.get_logger().info("{}".format(translation.z))

        # Rotate all points to base_link
        # TODO: Remove or rework; rotations are computationally costly!
        quat = [
            self.lidar_to_bl_tf.transform.rotation.x,
            self.lidar_to_bl_tf.transform.rotation.y,
            self.lidar_to_bl_tf.transform.rotation.z,
            self.lidar_to_bl_tf.transform.rotation.w
        ]
        r = R.from_quat(quat)
        # xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        
        # pts = r.apply(xyz)

        xyz = np.array([pts['x'],pts['y'],pts['z']])
        
        xyz = np.transpose(xyz)
        xyz = r.apply(xyz)
        xyz = np.transpose(xyz)
        xyz[0] += translation.x
        xyz[1] += translation.y
        xyz[2] += translation.z
        pts['x'] = xyz[0]
        pts['y'] = xyz[1]
        pts['z'] = xyz[2]

        # Crop to front distance
        pts = pts[pts['x'] >= self.MIN_X]

        # Crop to height
        pts = pts[
            np.logical_and(
                pts['z'] <= self.MAX_HEIGHT,
                pts['z'] >= self.MIN_HEIGHT
            )
        ]

        # Crop to width:
            # +/-1.5 meters if y < 6 OR
            # +/-1.0 meters otherwise
        pts = pts[
            np.logical_or(
                np.abs(pts['y']) < 1.0,
                np.logical_and(
                    pts['x'] <= 6.0,
                    abs(pts['y']) < 1.5
                )
            )
        ]

        # Only keep shiny points
        pts = pts[
            pts['intensity'] >= 200.0
        ]

        return pts

    def formPointCloud2(self, nparray, frame_id: str):
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, nparray)
        filtered_msg.header.frame_id = frame_id
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg

    def front_lidar_cb(self, msg: PointCloud2):
        if not self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
            self.get_logger().warn("Could not find tf from lidar_front to base_link")
            return
        else: 
            self.getLidarToBlTransform('lidar_front')
            pts = rnp.numpify(msg)
            pts = self.filterPoints(pts)

            if pts.shape[0] < 1:
                self.get_logger().error("No points to lock onto! Waiting.")
                return
            if pts.shape[0] < 10:
                self.get_logger().warn("Low obstacle quality: {} points found".format(pts.shape[0]))

            roi_msg = self.formPointCloud2(pts, 'base_link')
            
            self.obstacle_roi_pub.publish(roi_msg)

            closest_x = np.min(pts['x'])
            dist_from_front = closest_x - self.VEHICLE_LENGTH
            median_y = np.median(pts['y'])
            
            self.get_logger().info(
                "{}".format(np.max(pts['intensity']))
            )

            steering_cmd = Float32()
            steering_cmd.data = median_y * self.STEERING_K
            self.steering_cmd_pub.publish(steering_cmd)

            throttle_cmd = Float32()
            if dist_from_front < self.TARGET_DISTANCE:
                throttle_cmd.data = 0.0
            else:
                throttle_cmd.data = self.BANG_ON_VALUE
            

            # FINAL CLAMP for safety :)
            if throttle_cmd.data > 1.0:
                throttle_cmd.data = 1.0
                self.get_logger().error("Throttle too big! Clamping.")
            elif throttle_cmd.data < 0.0
                throttle_cmd.data = 0.0
            self.throttle_cmd_pub.publish(throttle_cmd)

            self.get_logger().info(
                "Str: {} | Thr: {} | Dst: {}".format(steering_cmd.data, throttle_cmd.data, dist_from_front)
            )

def main(args=None):
    rclpy.init(args=args)

    obstacle_peddler = ObstaclePeddler()

    rclpy.spin(obstacle_peddler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_peddler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()