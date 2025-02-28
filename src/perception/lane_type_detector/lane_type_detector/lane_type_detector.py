"""
Package: lane_type_detector
File: lane_type_detector.py
Author: Pranav Boyapati
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigator_msgs.msg import LaneDetections
from navigator_msgs.msg import AllLaneDetections
import cv2
from ultralytics import YOLO
import numpy
import base64


class LaneTypeDetector(Node):
    def __init__(self):
        super().__init__('lane_type_detector')

        #initialize the classification model
        model_path = '/navigator/data/perception/models/LaneDetector.pt'
        try:
            self.get_logger().info(f"Loading YOLO model from: {model_path}")
            self.model = YOLO(model_path)
            self.get_logger().info("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            raise e

        #create subscriptions
        self.camera_sub = self.create_subscription(Image, '/cameras/camera0', self.image_callback, 10)

        #create variables to store subscription info
        self.bridge = CvBridge()
        self.image = None

        #create publisher
        self.lane_detections_publisher = self.create_publisher(AllLaneDetections, '/lane_types/detections', 10)


    #callbacks for subscriptions
    def image_callback(self, msg : Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='8UC3')
        self.make_lane_detections()

    
    #main control function
    def make_lane_detections(self):
        #gets prediction
        #self.image = "image.jpeg"
        results = self.model(self.image)

        #create message and publish
        boxes = results[0].boxes.xyxy.tolist()      #top left corner of box to bottom right corner of box
        classes = results[0].boxes.cls.tolist()
        print(classes)
        confidences = results[0].boxes.conf.tolist()
            
        lane_detection_msg = AllLaneDetections()
        lane_detection_msg.header.stamp = self.get_clock().now().to_msg()
        lane_detection_msg.header.frame_id = "camera_frame"

        numTotalLanes = 1
        numLeftTurnLanes = 0
        numRightTurnLanes = 0

        numLanesLeftOfImageCenter = 0

        for box, cls, conf in zip(boxes, classes, confidences):
            if (int(cls) == 3):
                numTotalLanes += 1
                
                x1, y1, x2, y2 = box
                if (((x1 + x2) / 2) < (400 / 2)):
                    numLanesLeftOfImageCenter += 1
                
            if (int(cls) == 4):
                numLeftTurnLanes += 1
            if (int(cls) == 1):
                numRightTurnLanes += 1


            
            
        print(numTotalLanes)
        print(numLeftTurnLanes)
        print(numRightTurnLanes)
        print(numLanesLeftOfImageCenter)
        
        this_lane_detection = LaneDetections()
        
        this_lane_detection.totallanecount = numTotalLanes
        
        alllanes = []
        for i in range(numTotalLanes):
            alllanes.append("Straight")
        for j in range(numLeftTurnLanes):
            alllanes[j] = "Left"
        for k in range(numRightTurnLanes):
            alllanes[-1 * k] = "Right"
        this_lane_detection.alllanes = alllanes
	
        if (len(alllanes) != 0):
        	this_lane_detection.currentlane = alllanes[numLanesLeftOfImageCenter]

        this_lane_detection.numlanesleftofcurrent = numLanesLeftOfImageCenter
        
        lane_detection_msg.lane_detections.append(this_lane_detection)
        self.lane_detections_publisher.publish(lane_detection_msg)


def main(args=None):
    rclpy.init(args=args)
    lane_type_detections = LaneTypeDetector()
    rclpy.spin(lane_type_detections)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()
