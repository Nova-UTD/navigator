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
import math
import matplotlib.pyplot as plt


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

import mmcv
from mmcv.parallel import MMDataParallel
from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot


# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line
        
    
class DriveableAreaNode(Node):

    def __init__(self):
        super().__init__('driveable_area_node')
                
        camera_sub = self.create_subscription(Image, '/cameras/stitched', self.update_camera,1)
        
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.make_segmentation)

        self.current_image = None
        
        self.br = CvBridge()
        
        self.done = False
            
        config_file = '/navigator/data/perception/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.py'
        checkpoint_file = '/navigator/data/perception/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.pth'
        
        # checkpoint file download link: https://dl.cv.ethz.ch/bdd100k/drivable/models/ccnet_r50-d8_512x1024_80k_drivable_bdd100k.pth

        # build the model from a config file and a checkpoint file
        self.model = init_segmentor(config_file, checkpoint_file, device='cuda:0')
          

    def clock_cb(self, msg):
        self.clock = msg.clock

    def update_camera(self, image):
        
        self.current_image = image
    
    def make_segmentation(self):
        if self.current_image == None:
            return
        
        start = time.time()
        
        image = self.br.imgmsg_to_cv2(self.current_image)
        
        original_size = (image.shape[1], image.shape[0])
        
        # model input size
        target_size = (1024, 512)
        
        # scale needed to rescale input image to the cropable size of the 
        max_scale_match = max(target_size[0]/ original_size[0], target_size[1]/ original_size[1])
        
        # size of the scaled image
        middle_size = (int(original_size[0] * max_scale_match), int(original_size[1] * max_scale_match))
        
        # scales the image
        resized_image = cv2.resize(image, middle_size, interpolation = cv2.INTER_AREA)
        
        
        x_pixel_drop_count = middle_size[0] - target_size[0]
            
        # if the int(pixel_drop_count /2) isn't a whole number there would be a extra pixel
            # to handle this round one up, the other down
        xstart = math.ceil(x_pixel_drop_count/2)
        xend = middle_size[0] - math.floor(x_pixel_drop_count/2)
                
        # Crops the image
        cropped_image = resized_image[middle_size[1] - target_size[1]:, xstart: xend, :3]
                
        # inference_segmentor
        segmented_image = inference_segmentor(self.model, cropped_image)
        
        
        resize_nparr = np.ones((middle_size[1], middle_size[0])) * 2
        resize_nparr[middle_size[1] - target_size[1]:, xstart: xend] = segmented_image[0]
        
        output_image = cv2.resize(resize_nparr, original_size, interpolation = cv2.INTER_AREA)
        
        
        self.get_logger().info(f"{time.time() - start}")
        
        """
        IMAGES_FN = '/navigator/src/perception/driveable_area/cameraView/'
        
        
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        h = ax1.matshow(output_image)
        plt.colorbar(h)
        # plt.show()[]
        fig1.savefig(IMAGES_FN + "drivearea.png")
        plt.close()
        """
        

        
        
        
        



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
