'''
Package: segmentation
   File: image_segmentation_node.py
 Author: Will Heitman (w at heit dot mn)

Node to semantically segment a 2D image using a model.
'''


import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, Duration, QoSDurabilityPolicy
# import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
# from scipy.spatial.transform import Rotation as R
import sys
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32

import matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge

# OpenMMSegmentation
from mmseg.apis import inference_segmentor, init_segmentor
import mmcv

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


class ImageSegmentationNode(Node):

    def __init__(self):
        super().__init__('image_segmentation_node')
        config_file = '/mmsegmentation/configs/pspnet/pspnet_r18-d8_512x1024_80k_cityscapes.py'
        checkpoint_file = '/navigator/data/perception/pspnet_r18-d8_512x1024_80k_cityscapes_20201225_021458-09ffa746.pth'

        self.model = init_segmentor(
            config_file, checkpoint_file, device='cuda:0')  # Change this to '1,' 2,' etc to change GPU used

        image_qos_policy = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0, nanoseconds=2e8)
        )

        self.left_rgb_sub = self.create_subscription(
            Image, "/carla/hero/rgb_left/image", self.rgbLeftCb, image_qos_policy)

        self.left_result_pub = self.create_publisher(
            Image, '/semantic/left', 1)

        self.right_rgb_sub = self.create_subscription(
            Image, "/carla/hero/rgb_right/image", self.rgbRightCb, image_qos_policy)
        self.right_result_pub = self.create_publisher(
            Image, '/semantic/right', 1)

        self.idx = 0

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 1)
        self.clock = Clock()
        # model.show_result(img, result, out_file='result.jpg', opacity=1.0)
        self.bridge = CvBridge()

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def imageToNumpy(self, msg) -> np.ndarray:
        """Converts Image message to numpy array

        Args:
            msg (Image): ROS Image message

        Raises:
            TypeError: If image encoding is not recognized

        Returns:
            np.ndarray: Image as np array
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

    def numpyToImage(self, arr: np.ndarray, encoding: str) -> Image:
        """Converts np array to Image message

        Args:
            arr (np.ndarray): _description_
            encoding (str): _description_

        Raises:
            TypeError: Unrecognized image encoding requested
            TypeError: Input array shape is invalid
            TypeError: Image channels did not match requested encoding
            TypeError: Array dtype was invalid

        Returns:
            Image: ROS Image message
        """
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
        # contig = np.ascontiguousarray(arr)
        im.data = arr.tostring()
        im.step = arr.strides[0]
        im.is_bigendian = (
            arr.dtype.byteorder == '>' or
            arr.dtype.byteorder == '=' and sys.byteorder == 'big'
        )

        return im

    def convertToColor(self, mono_result: np.ndarray) -> np.array:
        """Converts a one-channel segmentation result to a colored image using the CityScapes coloring scheme.

        Args:
            mono_result (np.array): Segmentation result (2d)

        Returns:
            np.array: Colored result (3D array, accounting for RGB channel)
        """
        result_rgb = np.zeros(
            (mono_result.shape[0], mono_result.shape[1], 3), dtype=np.uint8)
        result_rgb[:, :][mono_result == 0] = [128, 64, 128]  # Road
        result_rgb[:, :][mono_result == 1] = [244, 35, 232]  # Sidewalk
        result_rgb[:, :][np.logical_or(np.logical_or(
            mono_result == 2, mono_result == 3), mono_result == 4)] = [70, 70, 70]  # Building, wall, fence
        result_rgb[:, :][mono_result == 3] = [100, 40, 40]  # Fence
        result_rgb[:, :][mono_result == 5] = [153, 153, 153]  # Pole
        result_rgb[:, :][mono_result == 6] = [250, 170, 30]  # Traffic light
        result_rgb[:, :][mono_result == 7] = [220, 220, 0]  # Traffic light
        result_rgb[:, :][mono_result == 8] = [107, 142, 35]  # Vegetation
        result_rgb[:, :][mono_result == 9] = [145, 170, 100]  # Terrain
        result_rgb[:, :][mono_result == 10] = [70, 130, 180]  # Sky
        result_rgb[:, :][mono_result == 11] = [220, 20, 60]  # Person
        result_rgb[:, :][np.logical_or(np.logical_or(
            np.logical_or(mono_result == 12, mono_result == 13),
            np.logical_or(mono_result == 14, mono_result == 15)),
            mono_result == 16)] = [0, 0, 142]  # Car, rider, truck, bus, train, motorcycle
        result_rgb[:, :][np.logical_or(mono_result == 17, mono_result == 18)] = [
            119, 11, 32]  # Building, wall, fence
        return result_rgb

    def rgbLeftCb(self, msg: Image):
        img_array = self.bridge.imgmsg_to_cv2(
            msg, 'rgb8')[:, :, :3]  # Cut out alpha

        # Actually performs the inference
        result = inference_segmentor(self.model, img_array)[0]

        result_rgb = self.convertToColor(result)

        result_msg_rgb = self.bridge.cv2_to_imgmsg(result_rgb, encoding='rgb8')
        result_msg_rgb.header = msg.header

        self.left_result_pub.publish(result_msg_rgb)

    def rgbRightCb(self, msg: Image):
        img_array = self.bridge.imgmsg_to_cv2(
            msg, 'rgb8')[:, :, :3]  # Cut out alpha

        # Actually performs the inference
        result = inference_segmentor(self.model, img_array)[0]

        result_rgb = self.convertToColor(result)

        result_msg_rgb = self.bridge.cv2_to_imgmsg(result_rgb, encoding='rgb8')
        result_msg_rgb.header = msg.header

        self.right_result_pub.publish(result_msg_rgb)


def main(args=None):
    rclpy.init(args=args)

    lidar_processor = ImageSegmentationNode()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
