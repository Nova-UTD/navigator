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
import cv2


# Message definitions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from sensor_msgs import Image


import matplotlib.pyplot as plt

# Set the Environmental varibale  max_split_size_mb to working amount in order to mkae the model run
# export 'PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:unlimited' // in comand line

class BoundingBoxes:
    
    def __init__(self):
        
        self.boxes = []
    
    def add_box(self, box):
        self.boxes.append(box)
    
    def clear(self):
        self.boxes = []
        
class BoundingBox:
    
    def __init__(self, x=None,y=None,w=None,h=None, type=None):
        
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.type = type
        
    def setBox(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h


class CameraHandler:
    
    def __init__(self, index) -> None:
        self.index = index
        self.current_image = None
        
    def update_camera(self, image):
        
        self.current_image = image
        
    
class ObjectTrackingNode(Node):

    def __init__(self):
        super().__init__('object_tracking_node')
        
        
        self.get_logger().info("Starting")
        
        # Subcribes to bounding boxes detection
        self.boxes_sub = self.create_subscription(
            Float32, '/camera/object', self.update_detection, 10)
        
        # Get active UVC cameras
        _, working_ports, _ = self.list_ports()

        # Holds what updates the camera
        self.camera_handlers =[]
        # Holds the subscriptions
        self.camera_subscriptions = []
        # index for camera
        index = 0
        
        # For each working port, check if it's a ZED cam (not a laptop webcam etc)
        for port in working_ports:
            with open(f'/sys/class/video4linux/video{port}/name') as f:
                contents = f.readline()
                if contents.find('ZED') >= 0:
                    self.camera_handlers.append(CameraHandler(index))
                    
                    index += 1
                    
                    camera_sub = self.create_subscription(Image, f'/cameras/camera{index}', self.camera_handlers[index-1 ].update_camera,1)
                    self.camera_subscriptions.append(camera_sub)
        
        
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        # Subscribes to timer in order to make predictions at a constant rate
        self.timer = self.create_timer(0.1, self.update_tracking)
        
        # Instantiates publisher
        self.tracking_pub = self.create_publisher(
           Float32, '/grid/predictions', 10)
        
        # CUrrent Object boundaries
        self.current_boxes = BoundingBoxes()
        
        """
        # GPU that the prednet model will use
        self.device = 'cuda:0'
        # Sets the device to the rigth GPU
        torch.cuda.set_device(torch.device(self.device))
        torch.cuda.empty_cache()
        """      

    def update_tracking(self):
        
        self.get_logger().info("Trying to update trakcing")
        
        for i in range(len(self.camera_handlers)):
            current_cam = self.camera_handlers[i]
            
            if current_cam.current_image == None:
                continue
            
            image2D = current_cam.current_image.data.reshape((current_cam.current_image.width, current_cam.current_image.height))
            
            IMAGES_FN = '/navigator/src/perception/object_tracking/cameraView/'

            fig1 = plt.figure()
            ax1 = fig1.add_subplot(111)
            h = ax1.matshow(image2D)
            plt.colorbar(h)
            fig1.savefig(IMAGES_FN + f'camera{i}' + '.png')
            plt.close()
            
        return
    
    # Sets current boxes to the newly object detected boxes
    def update_detection(self, detectionBoxes):
        current_boxes = detectionBoxes
        
        
    
    
    def list_ports(self) -> tuple:
        """:

        Returns:
            tuple: available, working, and broken ports
        """        
        non_working_ports = []
        dev_port = 0
        working_ports = []
        available_ports = []
        while len(non_working_ports) < 10: # if there are more than 5 non working ports stop the testing. 
            camera = cv2.VideoCapture(dev_port)
            if not camera.isOpened():
                non_working_ports.append(dev_port)
                # print("Port %s is not working." %dev_port)
            else:
                is_reading, img = camera.read()
                w = camera.get(3)
                h = camera.get(4)
                if is_reading:
                    # print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                    working_ports.append(dev_port)
                else:
                    # print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                    available_ports.append(dev_port)
            dev_port +=1
        return available_ports,working_ports,non_working_ports  

        
        
        
        



def main(args=None):
    rclpy.init(args=args)

    tracking_node = ObjectTrackingNode()

    rclpy.spin(tracking_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
