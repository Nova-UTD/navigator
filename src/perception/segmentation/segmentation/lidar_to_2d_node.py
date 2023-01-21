'''
Package: segmentation
   File: image_segmentation_node.py
 Author: Will Heitman (w at heit dot mn)

Node to semantically segment a 2D image using a model.
'''


import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import sys
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32

name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":  (np.uint8,  1),
    "bayer_bggr8":  (np.uint8,  1),
    "bayer_gbrg8":  (np.uint8,  1),
    "bayer_grbg8":  (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}

img_shape = (512, 1024)


class LidarTo2dNode(Node):

    def __init__(self):
        super().__init__('lidar_to_2d_node')
        self.get_logger().info("Ready to project!")

        lidar_sub = self.create_subscription(
            PointCloud2, "/lidar/filtered", self.lidar_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_camera_frame = 'hero/rgb_left'

    def getTransformMatrix(self, source: str, dest: str) -> np.array:
        t: TransformStamped
        try:
            t = self.tf_buffer.lookup_transform(
                dest, source, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frame: {ex}')
            return

        q_msg = t.transform.rotation
        rotation = R.from_quat([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
        rotation_matrix = rotation.as_matrix()
        translation_matrix = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])

        self.R0 = rotation_matrix  # TODO: Figure out what this is

        L2C = np.vstack((rotation_matrix, translation_matrix)).T
        # print(L2C)
        return L2C

    def numpy_to_image(self, arr, encoding):
        if not encoding in name_to_dtypes:
            raise TypeError('Unrecognized encoding {}'.format(encoding))

        im = Image(encoding=encoding)

        # extract width, height, and channels
        dtype_class, exp_channels = name_to_dtypes[encoding]
        dtype = np.dtype(dtype_class)
        if len(arr.shape) == 2:
            im.height, im.width, channels = arr.shape + (1,)
        elif len(arr.shape) == 3:
            im.height, im.width, channels = arr.shape
        else:
            raise TypeError("Array must be two or three dimensional")

        # check type and channels
        if exp_channels != channels:
            raise TypeError("Array has {} channels, {} requires {}".format(
                channels, encoding, exp_channels
            ))
        if dtype_class != arr.dtype.type:
            raise TypeError("Array is {}, {} requires {}".format(
                arr.dtype.type, encoding, dtype_class
            ))

        # make the array contiguous in memory, as mostly required by the format
        contig = np.ascontiguousarray(arr)
        im.data = contig.tostring()
        im.step = contig.strides[0]
        im.is_bigendian = (
            arr.dtype.byteorder == '>' or
            arr.dtype.byteorder == '=' and sys.byteorder == 'big'
        )

        return im

    # From Github https://github.com/balcilar/DenseDepthMap
    def dense_map(self, lidar_pts, image_width, image_height, grid):
        ng = 2 * grid + 1

        mX = np.zeros((image_height, image_width)) + np.float("inf")
        mY = np.zeros((image_height, image_width)) + np.float("inf")
        mD = np.zeros((image_height, image_width))
        mX[np.int32(lidar_pts[1]), np.int32(lidar_pts[0])
           ] = lidar_pts[0] - np.round(lidar_pts[0])
        mY[np.int32(lidar_pts[1]), np.int32(lidar_pts[0])
           ] = lidar_pts[1] - np.round(lidar_pts[1])
        mD[np.int32(lidar_pts[1]), np.int32(lidar_pts[0])] = lidar_pts[2]

        KmX = np.zeros((ng, ng, image_height - ng, image_width - ng))
        KmY = np.zeros((ng, ng, image_height - ng, image_width - ng))
        KmD = np.zeros((ng, ng, image_height - ng, image_width - ng))

        for i in range(ng):
            for j in range(ng):
                KmX[i, j] = mX[i: (image_height - ng + i),
                               j: (image_width - ng + j)] - grid - 1 + i
                KmY[i, j] = mY[i: (image_height - ng + i),
                               j: (image_width - ng + j)] - grid - 1 + i
                KmD[i, j] = mD[i: (image_height - ng + i),
                               j: (image_width - ng + j)]
        S = np.zeros_like(KmD[0, 0])
        Y = np.zeros_like(KmD[0, 0])

        for i in range(ng):
            for j in range(ng):
                s = 1/np.sqrt(KmX[i, j] * KmX[i, j] + KmY[i, j] * KmY[i, j])
                Y = Y + s * KmD[i, j]
                S = S + s

        S[S == 0] = 1
        out = np.zeros((image_height, image_width))
        out[grid + 1: -grid, grid + 1: -grid] = Y/S
        return out

    # From LiDAR coordinate system to Camera Coordinate system
    def lidar2cam(self, pts_3d_lidar):
        n = pts_3d_lidar.shape[0]
        pts_3d_hom = np.hstack((pts_3d_lidar, np.ones((n, 1))))
        pts_3d_cam_ref = np.dot(pts_3d_hom, np.transpose(self.L2C))
        pts_3d_cam_rec = np.transpose(
            np.dot(self.R0, np.transpose(pts_3d_cam_ref)))
        return pts_3d_cam_rec

    # From Camera Coordinate system to Image frame
    def rect2Img(self, rect_pts, img_width, img_height):
        n = rect_pts.shape[0]
        points_hom = np.hstack((rect_pts, np.ones((n, 1))))
        points_2d = np.dot(points_hom, np.transpose(self.P))  # nx3
        points_2d[:, 0] /= points_2d[:, 2]
        points_2d[:, 1] /= points_2d[:, 2]

        mask = (points_2d[:, 0] >= 0) & (points_2d[:, 0] <= img_width) & (
            points_2d[:, 1] >= 0) & (points_2d[:, 1] <= img_height)
        mask = mask & (rect_pts[:, 2] > 2)
        return points_2d[mask, 0:2], mask

    def lidar_cb(self, msg: PointCloud2):
        lidar_array = rnp.numpify(msg)

        # Strip away the dtypes
        lidar_array_raw = np.vstack(
            (lidar_array['x'], lidar_array['y'], lidar_array['z'])).T

        # print(lidar_array_raw)

        self.L2C = self.getTransformMatrix('base_link', self.left_camera_frame)
        if self.L2C is None:
            return

        lidar_rect = self.lidar2cam(lidar_array_raw)

        lidar_on_image = self.rect2Img(lidar_rect, img_shape[1], img_shape[0])
        print(lidar_on_image)


def main(args=None):
    rclpy.init(args=args)

    lidar_projection_node = LidarTo2dNode()

    rclpy.spin(lidar_projection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_projection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
