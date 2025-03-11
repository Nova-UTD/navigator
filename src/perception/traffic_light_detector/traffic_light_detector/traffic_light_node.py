'''
Package: traffic_light_detector
   File: traffic_light_node.py
 Author: Vindhya Kaushall

Node to filter and process raw camera footage and classify traffic lights

This node outputs green for green, yellow for yellow, and red for red.

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from navigator_msgs.msg import TrafficLight, TrafficLightDetection
from PIL import Image as PILImage
import base64

class TrafficLightDetectionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        
        # Initialize YOLO model for traffic light detection
        self.model = YOLO('best_traffic_med_yolo_v8.pt')  # Correct path

        # Initialize CvBridge to convert ROS image messages to OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to the image topic (modify topic if necessary)
        self.image_subscriber = self.create_subscription(
            Image, '/cameras/camera0', self.image_callback, 10)
        
        # Publisher for the detection result
        self.traffic_light_publisher = self.create_publisher(TrafficLightDetection, '/traffic_lights/detections', 10)

    def image_callback(self, msg: Image):
       # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
       # Run inference on the image using YOLO
        results = self.model.predict(source=cv_image)
    
       # Create a TrafficLightDetection message to publish
        detection_msg = TrafficLightDetection()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = 'camera_frame'
    
       # Process results and append to the detection message
        """for r in results:
            for box in r.boxes:"""
            # Extract bounding box data
        
        boxes = results[0].boxes.xyxy.tolist()
        classes = results[0].boxes.cls.tolist()
        names = results[0].names
        confidences = results[0].boxes.conf.tolist()

# Iterate through the results
        """x1=0.0
        y1=0.0 
        x2=0.0
        y2=0.0"""

        for box, cls, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = box
            confidence = conf
            detected_class = cls
            name = names[int(cls)]
            
                # Create a TrafficLight message
            traffic_light = TrafficLight()
            traffic_light.x = float(x1)
            traffic_light.y = float(y1)
            traffic_light.width = float(x2 - x1)
            traffic_light.height = float(y2 - y1)
            traffic_light.label = int(cls)  # Example label (traffic light class)
            '''traffic_light.confidence = float(conf)''' #no to this
                
            detection_msg.traffic_lights.append(traffic_light)

       # Publish the detection result
        self.traffic_light_publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

