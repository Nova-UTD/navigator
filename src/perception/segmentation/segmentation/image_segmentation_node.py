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
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

import matplotlib.pyplot as plt

# OpenMMSegmentation
import torch
import torchvision
import mmseg
from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
from mmseg.core.evaluation import get_palette
print(mmseg.__version__)

print(torch.__version__, torch.cuda.is_available())
# from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot


class ImageSegmentationNode(Node):

    def __init__(self):
        super().__init__('image_segmentation_node')


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
