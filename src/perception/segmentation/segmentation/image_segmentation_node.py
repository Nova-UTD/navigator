'''
Package: segmentation
   File: image_segmentation_node.py
 Author: Will Heitman (w at heit dot mn)

Node to semantically segment a 2D image using a model.
'''


import rclpy
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
            config_file, checkpoint_file, device='cuda:0')

        self.left_rgb_sub = self.create_subscription(
            Image, "/carla/hero/rgb_left/image", self.rgb_left_cb, 10)
        self.left_result_pub = self.create_publisher(
            Image, '/semantic/left_mono', 10)

        self.right_rgb_sub = self.create_subscription(
            Image, "/carla/hero/rgb_right/image", self.rgb_right_cb, 10)
        self.right_result_pub = self.create_publisher(
            Image, '/semantic/right_mono', 10)

        self.idx = 0
        # model.show_result(img, result, out_file='result.jpg', opacity=1.0)

    def image_to_numpy(self, msg):
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

    def rgb_left_cb(self, msg: Image):
        img_array = mmcv.imread("/navigator/demo.png")

        img_array = self.image_to_numpy(msg)[:, :, :3]  # Cut out alpha
        result = inference_segmentor(self.model, img_array)[0]

        result_msg = self.numpy_to_image(result.astype(np.uint8), 'mono8')
        result_msg.header = msg.header

        self.left_result_pub.publish(result_msg)

    def rgb_right_cb(self, msg: Image):
        img_array = mmcv.imread("/navigator/demo.png")

        img_array = self.image_to_numpy(msg)[:, :, :3]  # Cut out alpha
        result = inference_segmentor(self.model, img_array)[0]

        result_msg = self.numpy_to_image(result.astype(np.uint8), 'mono8')
        result_msg.header = msg.header

        self.right_result_pub.publish(result_msg)


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
