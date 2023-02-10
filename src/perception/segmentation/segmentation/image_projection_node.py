'''
Package: segmentation
   File: image_projection_node.py
 Author: Will Heitman (w at heit dot mn)

Node to project 2D semantic segmentation results onto a 3D point cloud
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32

import image_geometry
import matplotlib.pyplot as plt

import struct

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


class ImageProjectioNode(Node):

    def __init__(self):
        super().__init__('image_projection_node')
        self.get_logger().info("Ready to project!")

        lidar_sub = self.create_subscription(
            PointCloud2, "/lidar/fused", self.lidarCb, 1)

        self.semantic_lidar_pub = self.create_publisher(
            PointCloud2, "/lidar/semantic", 10)

        left_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_left/camera_info', self.leftCameraInfoCb, 1)

        left_camera_image_sub = self.create_subscription(
            Image, '/semantic/left', self.leftCameraImageCb, 1)

        right_camera_image_sub = self.create_subscription(
            Image, '/semantic/right', self.rightCameraImageCb, 1)

        right_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_right/camera_info', self.rightCameraInfoCb, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_camera_frame = 'hero/rgb_left'
        self.right_camera_frame = 'hero/rgb_right'
        self.left_cam_model: image_geometry.PinholeCameraModel = None
        self.right_cam_model: image_geometry.PinholeCameraModel = None

        self.left_semantic_image = None
        self.right_semantic_image = None

    def imageToNumpy(self, msg: Image) -> np.array:
        """Converts Image message to numpy array

        Args:
            msg (Image)

        Raises:
            TypeError if image encoding not recognized

        Returns:
            np.array
        """
        if not msg.encoding in name_to_dtypes:
            raise TypeError('Unrecognized encoding {}'.format(msg.encoding))

        dtype_class, channels = name_to_dtypes[msg.encoding]
        dtype = np.dtype(dtype_class)
        dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
        shape = (msg.height, msg.width, channels)

        data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
        data.strides = (
            msg.step,
            dtype.itemsize * channels,
            dtype.itemsize
        )

        if channels == 1:
            data = data[..., 0]
        return data

    def leftCameraInfoCb(self, msg: CameraInfo) -> None:
        """Sets camera info for left camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        if self.left_cam_model is not None:
            return  # Already set, skip
        self.left_cam_model = image_geometry.PinholeCameraModel()
        self.left_cam_model.fromCameraInfo(msg)

    def rightCameraInfoCb(self, msg: CameraInfo):
        """Sets camera info for right camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        if self.right_cam_model is not None:
            return  # Already set, skip
        self.right_cam_model = image_geometry.PinholeCameraModel()
        self.right_cam_model.fromCameraInfo(msg)

    def leftCameraImageCb(self, msg: Image):
        self.left_semantic_image = self.imageToNumpy(msg)

    def rightCameraImageCb(self, msg: Image):
        self.right_semantic_image = self.imageToNumpy(msg)

    def tagPoints(self, camera_frame: str, pts: np.array, semantic_img: np.array) -> np.array:
        """Given LiDAR point cloud and semantic segmentation image, paint the image
            onto the point cloud, assigning a class to each point.

        Args:
            frame (str)
            pts (np.array): LiDAR points, where each row is a point [x,y,z]
            semantic_img (np.array): n x m semantic segmentation image

        Returns:
            np.array: Classified LiDAR points, where each row is a point [x,y,z,class]
        """
        t = TransformStamped()

        try:
            t = self.tf_buffer.lookup_transform(
                camera_frame, 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frame: {ex}')
            return

        # Rotate to camera frame
        q = t.transform.rotation
        tf_rotation: R = R.from_quat([q.x, q.y, q.z, q.w])
        lidar_array_tfed = tf_rotation.apply(pts)

        # Translate to camera frame
        lidar_array_tfed += [t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z]

        classified_pts = []
        rgb = []

        for idx, pt in enumerate(lidar_array_tfed):
            uv = self.right_cam_model.project3dToPixel(pt)
            pixel_coords = np.array(uv).astype(int)

            if pixel_coords[0] > 1023 or pixel_coords[0] < 0:
                continue
            elif pixel_coords[1] > 511 or pixel_coords[1] < 0:
                continue

            pixel_class = semantic_img[pixel_coords[1], pixel_coords[0]]
            # print(pixel_class)
            r = int(pixel_class[0])
            g = int(pixel_class[1])
            b = int(pixel_class[2])
            a = 255
            rgb.append(struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0])
            # classified_pt = np.append(pts[idx], pixel_class)
            classified_pts.append(pts[idx])

        classified_pts = np.array(classified_pts)  # Convert to np array
        rgb = np.array(rgb).astype(np.uint32)

        if len(classified_pts) < 1:
            return

        xyzc = np.zeros(classified_pts.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])

        xyzc['x'] = classified_pts[:, 0]
        xyzc['y'] = classified_pts[:, 1]
        xyzc['z'] = classified_pts[:, 2]
        xyzc['rgb'] = rgb

        # self.get_logger().info(str(xyzc[rgb == 4286595200]))

        return xyzc

    def lidarCb(self, msg: PointCloud2):
        """When LiDAR data is received, trim it, call tagPoints for each camera, and publish the result.

        Args:
            msg (PointCloud2): _description_

        Returns:
            _type_: _description_
        """
        if self.left_cam_model is None or self.right_cam_model is None:
            return  # Camera models not yet available.

        if self.left_semantic_image is None or self.right_semantic_image is None:
            return  # Semantic results not yet available.

        # Convert point cloud message to numpy array
        lidar_array = rnp.numpify(msg)

        # Strip away the dtypes
        lidar_array_raw = np.vstack(
            (lidar_array['x'], lidar_array['y'], lidar_array['z'])).T

        # Downsampe to 25%
        lidar_array_raw = lidar_array_raw[::4]

        # Trim distance
        MAX_DISTANCE = 40.0  # meters
        dist = np.linalg.norm(lidar_array_raw, axis=1)
        lidar_array_raw = lidar_array_raw[dist < MAX_DISTANCE]
        # Only points in front of us
        lidar_array_raw = lidar_array_raw[lidar_array_raw[:, 0] > 0.0]

        start = time.time()

        left_classified_pts = self.tagPoints(self.left_camera_frame, lidar_array_raw,
                                             self.left_semantic_image)

        left_classified_pts = left_classified_pts[left_classified_pts['y'] > 0]

        right_classified_pts = self.tagPoints(self.right_camera_frame, lidar_array_raw,
                                              self.right_semantic_image)

        right_classified_pts = right_classified_pts[right_classified_pts['y'] < 0]

        classified_pts = np.concatenate(
            (left_classified_pts, right_classified_pts), axis=0)

        # Convert our combined classified points back to a PointCloud2 message
        result_msg: PointCloud2 = rnp.msgify(PointCloud2, classified_pts)
        result_msg.header = msg.header

        self.semantic_lidar_pub.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)

    lidar_projection_node = ImageProjectioNode()

    rclpy.spin(lidar_projection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_projection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
