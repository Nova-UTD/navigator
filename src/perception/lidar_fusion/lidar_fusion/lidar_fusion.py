import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
import message_filters
import ros2_numpy as rnp

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.spatial.transform import Rotation as R


class LidarFusionNode(Node):

    def __init__(self, rotation_rate=10):
        super().__init__('lidar_fusion_node')

        self.rotation_rate = rotation_rate
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        lidar_front_sub = message_filters.Subscriber(
            self, PointCloud2, '/lidar_front')
        lidar_rear_sub = message_filters.Subscriber(
            self, PointCloud2, '/lidar_rear')
        self.fusion_sync = message_filters.ApproximateTimeSynchronizer(
            fs=[lidar_front_sub, lidar_rear_sub],
            queue_size=1,
            slop=1/rotation_rate
        )
        self.fusion_sync.registerCallback(self.fuse_lidar)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_fused_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic="/lidar_fused",
            qos_profile=10
        )

    # point cloud fields: float32 x , float32 y, float32 z, float32 intensity, uint16 ring
    # incoming shape from rnp: (~, 16), where 16 is the ring
    def tranform_point_cloud(self, pcd: np.ndarray, pcd_tf: TransformStamped):
        pcd = pcd.flatten()

        rotation = R.from_quat([
            pcd_tf.transform.rotation.x,
            pcd_tf.transform.rotation.y,
            pcd_tf.transform.rotation.z,
            pcd_tf.transform.rotation.w,
        ])

        xyz = np.array([pcd['x'], pcd['y'], pcd['z']]).transpose()
        xyz = rotation.apply(xyz).transpose()

        pcd['x'] = xyz[0] + pcd_tf.transform.translation.x
        pcd['y'] = xyz[1] + pcd_tf.transform.translation.y
        pcd['z'] = xyz[2] + pcd_tf.transform.translation.z

        return pcd

    def fuse_lidar(self, lidar_front_msg: PointCloud2, lidar_rear_msg: PointCloud2):
        try:
            now = rclpy.time.Time()
            lidar_front_target_tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                lidar_front_msg.header.frame_id,
                now,
                timeout=Duration(seconds=1.0)
            )

            lidar_rear_target_tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                lidar_rear_msg.header.frame_id,
                now,
                timeout=Duration(seconds=1.0)
            )

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform lidar frames to base-link: {ex}')
            return

        lidar_front_pcd = rnp.numpify(lidar_front_msg)
        lidar_front_pcd = self.tranform_point_cloud(
            lidar_front_pcd, lidar_front_target_tf)

        lidar_rear_pcd = rnp.numpify(lidar_rear_msg)
        lidar_rear_pcd = self.tranform_point_cloud(
            lidar_rear_pcd, lidar_rear_target_tf)

        fused_pcd = np.concatenate([lidar_front_pcd, lidar_rear_pcd])

        # Basic filtering
        fused_pcd = fused_pcd[np.logical_not(
            np.logical_and(
                np.logical_and(fused_pcd['x'] < 1, fused_pcd['x'] > -5),
                np.logical_and(fused_pcd['y'] < 1, fused_pcd['y'] > -1)
            )
        )]

        # fused_pcd = fused_pcd[fused_pcd['z'] > 0.3]

        lidar_fused_msg: PointCloud2 = rnp.msgify(PointCloud2, fused_pcd)
        lidar_fused_msg.header.frame_id = self.target_frame
        lidar_fused_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_fused_pub.publish(lidar_fused_msg)


def main(args=None):
    rclpy.init(args=args)

    lidar_fusion_node = LidarFusionNode(rotation_rate=10)

    rclpy.spin(lidar_fusion_node)

    lidar_fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
