import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from std_msgs.msg import String, Float32

class YOLOv8BrakeDetection(Node):
    def __init__(self):
        super().__init__('yolov8_brake_detection')

        # Declare the parameter for the camera topic name
        self.declare_parameter('image_topic', '/cameras/camera0')
        topic_name = self.get_parameter('image_topic').get_parameter_value().string_value

        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            topic_name,  # Use the topic name from the parameter
            self.image_callback,
            10
        )

        # Publisher for brake detection results (status as string)
        self.publisher = self.create_publisher(String, 'brake_detection_results', 10)
        # Publisher for confidence score as a float
        self.confidence_publisher = self.create_publisher(Float32, 'brake_detection_confidence', 10)

        # Initialize CvBridge for converting ROS Image to OpenCV image
        self.bridge = CvBridge()

        # Load the YOLOv8 model
        model_path = '/navigator/ros2_ws/src/yolov8_brake_detection/yolov8_brake_detection/best.pt'
        try:
            self.get_logger().info(f"Loading YOLO model from: {model_path}")
            self.model = YOLO(model_path)
            self.get_logger().info("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            raise e

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run inference with YOLOv8
            results = self.model(cv_image)

            # Process the results (detection boxes, classes, and confidence)
            for detection in results:
                class_name = detection['class']  # Update as per YOLO's output format
                confidence = detection['confidence']

                # Publish the brake status
                if class_name == 'car_BrakeOn':
                    self.publish_brake_status("Brake On")
                    print(f"Detected Brakes On")

                elif class_name == 'car_BrakeOff':
                    self.publish_brake_status("Brake Off")
                    print(f"Detected Brakes Off")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def publish_brake_status(self, status):
        try:
            # Publish brake status as a string
            message = String()
            message.data = status
            self.publisher.publish(message)
            self.get_logger().info(f"Published brake status: {status}")

        except Exception as e:
            self.get_logger().error(f"Failed to publish brake status: {e}")

    def publish_confidence(self, confidence):
        try:
            # Publish confidence score as a float
            confidence_message = Float32()
            confidence_message.data = confidence
            self.confidence_publisher.publish(confidence_message)
            self.get_logger().info(f"Published confidence: {confidence:.2f}")

        except Exception as e:
            self.get_logger().error(f"Failed to publish confidence score: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Initialize the YOLOv8BrakeDetection node
    node = YOLOv8BrakeDetection()

    # Spin the node to process incoming messages
    rclpy.spin(node)

    # Clean up and shut down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
