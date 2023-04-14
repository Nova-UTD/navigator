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
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from mmseg.models import build_segmentor
from mmcv.runner import (
    get_dist_info,
    init_dist,
    load_checkpoint,
    wrap_fp16_model,
)

# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line
        
    
class DriveableAreaNode(Node):

    def __init__(self):
        super().__init__('driveable_area_node')
        
        
        self.get_logger().info("Starting")
        
        camera_sub = self.create_subscription(Image, '/cameras/stitched', self.update_camera,1)
        
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.makimport matplotlib.pyplot as plt

        
        self.br = CvBridge()
        """
        # GPU that the prednet model will use
        self.device = 'cuda:0'
        # Sets the device to the rigth GPU
        torch.cuda.set_device(torch.device(self.device))
        torch.cuda.empty_cache()
        """      

    def clock_cb(self, msg):
        self.clock = msg.clock

    def update_camera(self, image):
        
        self.current_image = image
    
    def make_segmentation(self):
                
        #for i in range(len(self.camera_handlers)):
        if True:
            #current_cam = self.camera_handlers[i]
            
            #if current_cam.current_image == None:
                #continue
            if self.current_image == None:
                return
            image = self.br.imgmsg_to_cv2(self.current_image)
            
            # Display image
            
            #image2D = self.current_image.data.reshape((self.current_image.width, self.current_image.height))
            
            IMAGES_FN = '/navigator/src/perception/driveable_area/cameraView/'
            
            cv2.imwrite(IMAGES_FN + "cameraCenter.jpeg", image)
            
        return    
    


        
        
        
        



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
