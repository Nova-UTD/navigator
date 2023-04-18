'''
Package: driveable_area
   File: driveable_area_node.py
 Author: Jesse Musa

Loads a pretrained model from the BDD100K model zoo.
'''
import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import math
import matplotlib.pyplot as plt


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot

# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line

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

def numpy_to_image(arr, encoding):
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


class DriveableAreaNode(Node):

    def __init__(self):
        super().__init__('driveable_area_node')

        camera_sub = self.create_subscription(
            Image, '/cameras/camera0', self.cameraCb, 1)

        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.makeSegmentation)

        self.result_pub = self.create_publisher(Image, "/cameras/camera0/drivable", 1)

        self.current_image: Image = None

        self.cv_bridge = CvBridge()

        config_file = '/home/nova/navigator/data/perception/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.py'
        checkpoint_file = '/home/nova/navigator/data/perception/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.pth'

        # checkpoint file download link: https://dl.cv.ethz.ch/bdd100k/drivable/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.pth

        # build the model from a config file and a checkpoint file
        self.model = init_segmentor(
            config_file, checkpoint_file, device='cuda:0')

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def cameraCb(self, image):
        self.current_image = image

    def makeSegmentation(self):
        if self.current_image == None:
            return


        image = self.cv_bridge.imgmsg_to_cv2(self.current_image)

        original_size = (image.shape[1], image.shape[0])

        # model input size
        target_size = (1024, 512)

        # scale needed to rescale input image to the cropable size of the
        max_scale_match = max(
            target_size[0] / original_size[0], target_size[1] / original_size[1])

        # size of the scaled image
        middle_size = (
            int(original_size[0] * max_scale_match), int(original_size[1] * max_scale_match))

        # scales the image
        resized_image = cv2.resize(
            image, middle_size, interpolation=cv2.INTER_AREA)

        x_pixel_drop_count = middle_size[0] - target_size[0]

        # if the int(pixel_drop_count /2) isn't a whole number there would be a extra pixel
        # to handle this round one up, the other down
        xstart = math.ceil(x_pixel_drop_count/2)
        xend = middle_size[0] - math.floor(x_pixel_drop_count/2)

        # Crops the image
        cropped_image = resized_image[middle_size[1] -
                                      target_size[1]:, xstart: xend, :3]

        # inference_segmentor
        start = time.time()
        segmented_image = inference_segmentor(self.model, cropped_image)

        resize_nparr = np.ones((middle_size[1], middle_size[0])) * 2
        resize_nparr[middle_size[1] - target_size[1]
            :, xstart: xend] = segmented_image[0]

        output_image = cv2.resize(
            resize_nparr, original_size, interpolation=cv2.INTER_AREA)
        output_arr = np.asarray(output_image).astype(np.uint8)*100
        print(output_arr)
        
        result_msg = numpy_to_image(output_arr, 'mono8')
        # result_msg = self.cv_bridge.cv2_to_imgmsg(output_image, 'bgra8')
        # result_msg.header.stamp = self.clock
        self.result_pub.publish(result_msg)

        self.get_logger().info(f"{time.time() - start}")


def main(args=None):
    rclpy.init(args=args)

    tracking_node = DriveableAreaNode()

    rclpy.spin(tracking_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
