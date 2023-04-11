'''
Package: sensor_processing
   File: lidar_processing_node.py
 Author: Will Heitman (w at heit dot mn)

Node to filter and process raw LiDAR pointclouds

This node specifically deals with quirks with the
CARLA simulator. Namely, synching issues mean that
raw LiDAR streams "flicker" from left-sided PCDs
to right-sided ones. This node first merges
these left- and right-sided PCDs into a complete
cloud before cutting out points near the car.
'''
# https://1drv.ms/v/s!An_6l1bJUIotjlN-qg-uqKdIORgC?e=s5RDv6

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
import time
import copy


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32


import matplotlib.pyplot as plt

# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line

class BoundingBoxes:
    
    def __init__(self):
        
        self.boxes = []
    
    def clear(self):
        self.boxes = []
        
class BoundingBox:
    
    def __init__(self):
        
        self.x = None
        self.y = None
        self.w = None
        self.h = None
        self.type = None
        
    def setBox(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    
        


class PredNetNode(Node):

    def __init__(self):
        super().__init__('prednet_inference_node')
        
        # Subcribes to masses
        self.boxes_sub = self.create_subscription(
            BoundingBoxes, '/camera/object', self.update_detection, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.update_tracking)
        
        # Instantiates publisher
        self.tracking_pub = self.create_publisher(
           BoundingBoxes, '/grid/predictions', 10)
        
        # Size of the grids
        self.sizeX = None
        self.sizeY = None
        
        """
        # GPU that the prednet model will use
        self.device = 'cuda:0'
        # Sets the device to the rigth GPU
        torch.cuda.set_device(torch.device(self.device))
        torch.cuda.empty_cache()
        """
        
        
        
        
        
        



def main(args=None):
    rclpy.init(args=args)

    prednet_node = PredNetNode()

    rclpy.spin(prednet_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    prednet_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
