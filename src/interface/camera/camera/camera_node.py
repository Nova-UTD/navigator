'''
Package:   camera
Filename:  camera_node.py
Author:    Will Heitman (w at heit.mn)

Connects to USB webcams and publishes their streams to ROS

Available webcams are listed using \

'''

import cv2
from cv_bridge import CvBridge
import numpy as np
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
import time

import rclpy
from rclpy.node import Node


class camera_node(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        # Get active UVC cameras
        _, working_ports, _ = self.list_ports()

        self.cameras = []
        self.camera_publishers = []

        # For each working port, check if it's a ZED cam (not a laptop webcam etc)
        for port in working_ports:
            with open(f'/sys/class/video4linux/video{port}/name') as f:
                contents = f.readline()
                if contents.find('ZED') >= 0:
                    camera = cv2.VideoCapture(port)
                    self.cameras.append(camera)
                    
                    camera_pub = self.create_publisher(Image, f'/camera/camera{len(self.camera_publishers)}', 1)
                    self.camera_publishers.append(camera_pub)

        # Call publishFrames very frequently.
        self.capture_timer = self.create_timer(0.01, self.publishFrames)

    def publishFrames(self):
        """Iterate through each camera and publish the latest frame
        """        
        start = time.time()

        for idx, camera in enumerate(self.cameras):
            if not camera.isOpened():
                self.get_logger().warn("Camera could not be opened")
            ret, frame = camera.read()

            # Crop to left frame only. ZEDs return both camera in stereo by default.
            original_width = frame.shape[1]
            frame = frame[:,0:int(original_width/2),:]

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            self.camera_publishers[idx].publish(msg)

    def list_ports(self) -> tuple:
        """
        Test the ports and returns a tuple with the available ports and the ones that are working.

        Taken from S.O.

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

    def releaseCameras(self):
        """Disconnect from all cameras on shutdown.
        """        
        for camera in self.cameras:
            camera.release()


def main(args=None):
    rclpy.init(args=args)
    node = camera_node()
    rclpy.spin(node)

    camera_node.releaseCameras()
    camera_node.destroy_node()
    rclpy.shutdown()
