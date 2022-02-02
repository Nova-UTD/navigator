import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        npcloud = rnp.numpify(msg)
        lower_ring = 0
        upper_ring = 3
        x = np.transpose(npcloud['x'][:,lower_ring:upper_ring+1])
        y = np.transpose(npcloud['y'][:,lower_ring:upper_ring+1])
        z = np.transpose(npcloud['z'][:,lower_ring:upper_ring+1])
        i = np.transpose(npcloud['intensity'][:,lower_ring:upper_ring+1])
        pts = x
        np.append(pts, y, axis=1)
        np.append(pts, z, axis=1)
        # np.append(pts, i, axis=1)
        # r = npcloud['ring']
        # self.get_logger().info(str(npcloud.shape))
        mask = npcloud[:]['ring']<=4
        npcloud = npcloud[mask]
        # self.get_logger().info(str(npcloud))
        np.savetxt('foo1.csv', npcloud, fmt='%4f %4f %4f %4f %2u')
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, npcloud)
        filtered_msg.header.frame_id = 'lidar_front'
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        self.lower_ring_pub_front.publish(filtered_msg)

        self.transform_pts('lidar_front', 'base_link')

    def rear_lidar_cb(self, msg: PointCloud2):
        npcloud = rnp.numpify(msg)
        lower_ring = 0
        upper_ring = 3
        x = np.transpose(npcloud['x'][:,lower_ring:upper_ring+1])
        y = np.transpose(npcloud['y'][:,lower_ring:upper_ring+1])
        z = np.transpose(npcloud['z'][:,lower_ring:upper_ring+1])
        i = np.transpose(npcloud['intensity'][:,lower_ring:upper_ring+1])
        pts = x
        np.append(pts, y, axis=1)
        np.append(pts, z, axis=1)
        # np.append(pts, i, axis=1)
        # r = npcloud['ring']
        # self.get_logger().info(str(npcloud.shape))
        mask = npcloud[:]['ring']<=4
        npcloud = npcloud[mask]
        # self.get_logger().info(str(npcloud))
        np.savetxt('foo1.csv', npcloud, fmt='%4f %4f %4f %4f %2u')
        filtered_msg: PointCloud2 = rnp.msgify(PointCloud2, npcloud)
        filtered_msg.header.frame_id = 'lidar_rear'
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        self.lower_ring_pub_rear.publish(filtered_msg)


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