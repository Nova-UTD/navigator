import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class Pedestrian_Skeleton(Node):
    def __init__(self):
        super().__init__('Pedestrian_Skeleton_detecctor')
        
        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            'cameras/camera0',
            self.image_callback,
            10)
        
        # Publish the processed image
        self.publisher = self.create_publisher(Image, 'processed_image', 10)

        # Publish detection messages
        self.detection_publisher = self.create_publisher(String, 'detection_status', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load YOLOv8 Pose model
        model_path = 'src/perception/pedestrian_skeleton/pedestrian_skeleton/pedestrian_skeleton.pt'
        self.model = YOLO(model_path)

        # Confidence threshold
        self.CONFIDENCE_THRESHOLD = 0.7  # Adjust as needed

    def image_callback(self, msg):
        try:
            encoding = msg.encoding

            if encoding == '8UC3':
                # Convert ROS Image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                # Perform Pose Detection using YOLOv8
                results = self.model(cv_image)

                # Extract detected objects
                detections = results[0].boxes.data  # Tensor with [x1, y1, x2, y2, confidence, class]
                
                highest_confidence = 0.0
                person_detected = False

                for detection in detections:
                    x1, y1, x2, y2, confidence, cls = detection.tolist()
                    
                    if cls == 0 and confidence > self.CONFIDENCE_THRESHOLD:  # Class 0 is person
                        person_detected = True
                        highest_confidence = max(highest_confidence, confidence)

                # Publish a message if a person is detected
                if person_detected:
                    detection_msg = String()
                    detection_msg.data = f"Person detected with confidence: {highest_confidence:.2f}"
                    self.detection_publisher.publish(detection_msg)
                    self.get_logger().info(detection_msg.data)

                # Annotate the image
                annotated_img = results[0].plot()
                
                # Convert annotated image back to ROS Image
                processed_img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')

                # Publish the annotated image
                self.publisher.publish(processed_img_msg)

                # Display the image
                cv2.imshow("Annotated Image", annotated_img)
                cv2.waitKey(1)

            else:
                self.get_logger().error(f"Unsupported image encoding: {encoding}")

        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Pedestrian_Skeleton()
    
    # Spin the node
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
