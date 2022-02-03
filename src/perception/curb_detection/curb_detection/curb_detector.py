import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import PointCloud2


class CurbDetector(Node):

    def __init__(self):
        super().__init__('curb_detector')
        self.lower_ring_pub_front = self.create_publisher(
            PointCloud2,
            'lidar/road_front',
            10
        )

        self.lower_ring_pub_rear = self.create_publisher(
            PointCloud2,
            'lidar/road_rear',
            10
        )

        self.front_lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_front/velodyne_points',
            self.front_lidar_cb,
            10)

        self.rear_lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_rear/velodyne_points',
            self.rear_lidar_cb,
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def transform_pts(self, source_frame, dest_frame):
        tf: TransformStamped
        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(
                dest_frame,
                source_frame,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {dest_frame} to {source_frame}: {ex}')
            return
        self.get_logger().info(tf.header.frame_id)

    def front_lidar_cb(self, msg: PointCloud2):
        self.lower_ring_pub_front.publish(self.process_lidar(msg, 'lidar_front'))

    def rear_lidar_cb(self, msg: PointCloud2):
        self.lower_ring_pub_rear.publish(self.process_lidar(msg, 'lidar_rear'))
        
        
    def process_lidar(self, msg: PointCloud2, lidar_frame: str):
        # Transform to base_link
        if not self.tf_buffer.can_transform('base_link', lidar_frame, self.get_clock().now()):
            return
        t_zero = rclpy.time.Time(seconds=0, nanoseconds=0)
        lidar_to_bl_tf: TransformStamped = self.tf_buffer.lookup_transform('base_link', lidar_frame, t_zero)
        self.get_logger().info(str(lidar_to_bl_tf.transform.translation.x))

        npcloud = rnp.numpify(msg)
        # self.get_logger().info(str(npcloud.shape))
        # Choose only points in ring 4 and below
        mask = npcloud[:]['ring']<=4
        npcloud = npcloud[mask]
        translation = lidar_to_bl_tf.transform.translation
        npcloud['x'] += translation.x
        npcloud['y'] += translation.y
        npcloud['z'] += translation.z

        # Rotate each point
        quat = [
            lidar_to_bl_tf.transform.rotation.x,
            lidar_to_bl_tf.transform.rotation.y,
            lidar_to_bl_tf.transform.rotation.z,
            lidar_to_bl_tf.transform.rotation.w
        ]
        r = Rotation.from_quat(quat)
        xyz = np.transpose(np.array([npcloud['x'], npcloud['y'], npcloud['z']]))
        
        pts = r.apply(xyz)
        self.get_logger().info(str(xyz[:,0]))
        npcloud['x'] = pts[:,0]
        npcloud['y'] = pts[:,1]
        npcloud['z'] = pts[:,2]

        # At this point our data is now in the base_link frame.
        # Let's remove all points with z > 0.5 (meter)
        mask = npcloud[:]['z'] <= 0.5
        npcloud = npcloud[mask]
        
        # Construct message from numpy array and publish
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, npcloud)
        filtered_msg.header.frame_id = 'base_link'
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        return filtered_msg
        

    # def calibrate(self, points):
    #     # Find angle of point to LEFT or RIGHT (roll)
    #         # Points along Y axis will have X value close to 0, ring=3
    #     # Take abs value of all points' x values. Find min.
        
    #     ring_points = np.array([[]])
    #     for point in points:
    #         if point[3] == 4:
    #             ring_points = np.append(ring_points, [point], axis=1)
    #     abs_points = np.abs(ring_points)
    #     res = np.argmin(abs_points[0])
    #     self.get_logger().info("Min is: %s" % str(ring_points))

    #     # Then find angle of point IN FRONT (pitch)
    #         # Y value = 0, ring = 3



def main(args=None):
    rclpy.init(args=args)

    curb_detector = CurbDetector()

    rclpy.spin(curb_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    curb_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()