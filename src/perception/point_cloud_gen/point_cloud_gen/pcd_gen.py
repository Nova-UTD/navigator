import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import ros2_numpy as rnp
import numpy as np


IMAGE_SIZE_X = 1280
IMAGE_SIZE_Y = 720
FOCAL_X = 531.5750
FOCAL_Y = 531.3700
CENTER_X = 643.3700
CENTER_Y = 356.9830

DEPTH_MAX = 20.0
RANGE_MAX = 60.0
SEARCH_RADIUS = 0.75

INTRINSIC_MATRIX = np.array([
    [FOCAL_X, 0, CENTER_X, 0],
    [0, FOCAL_Y, CENTER_Y, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

EXTRINSIC_MATRIX = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 1],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

PROJECTION_MATRIX = np.matmul(INTRINSIC_MATRIX, EXTRINSIC_MATRIX)
INV_PROJ = np.linalg.inv(PROJECTION_MATRIX)


class PointCloudGenNode(Node):

    def __init__(self):
        super().__init__('pcd_gen_node')

        self.depth_sub = self.create_subscription(
            Image,
            '/depth_image',
            self.depth_cb,
            10
        )

        self.pcd_pub = self.create_publisher(
            PointCloud2,
            '/depth_point_cloud',
            10
        )

    def depth_cb(self, depth_msg: Image):
        depth_image = rnp.numpify(depth_msg)

        # project into 3-D space
        xx, yy = np.meshgrid(np.arange(0, IMAGE_SIZE_X),
                             np.arange(0, IMAGE_SIZE_Y))
        pixel_array = np.stack([xx, yy, np.ones_like(
            xx), 1/(depth_image+1e-3)]).reshape(4, -1)
        points_array = np.matmul(INV_PROJ, pixel_array) * depth_image.flatten()
        pcd = np.transpose(points_array[:-1, :], (1, 0)).astype(np.float32)

        # filter down to like 10,000 points using random uniform or whatever else is appropriate
        num_points = pcd.shape[0]
        num_select_points = 100_000
        if num_select_points < num_points:
            random_indices = np.random.choice(
                num_points, num_select_points, replace=False)
            pcd = pcd[random_indices]

        # turn into pointcloud msg
        pcd.dtype = [('x', np.float32), ('y', np.float32),
                     ('z', np.float32)]  # specify fields
        pcd_msg = rnp.msgify(PointCloud2, pcd)
        pcd_msg.header.frame_id = 'base_link'
        pcd_msg.header.stamp = self.get_clock().now().to_msg()
        self.pcd_pub.publish(pcd_msg)


def main(args=None):
    rclpy.init(args=args)

    pcd_gen_node = PointCloudGenNode()

    rclpy.spin(pcd_gen_node)

    pcd_gen_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
