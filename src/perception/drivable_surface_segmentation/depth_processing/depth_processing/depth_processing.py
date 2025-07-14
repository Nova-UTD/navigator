import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np

class DepthProcessingNode(Node):
    def __init__(self):
        super().__init__('depth_processing_node')
        self.bridge = CvBridge()

        #subscribe to rosbag depth topics
        self.range_image_sub = self.create_subscription(Image, '/ouster/range_image', self.process_depth,qos_profile_sensor_data)

        self.publisher = self.create_publisher(Image, '/processed_depth', 10)

        self.get_logger().info("Depth Processing Node Started (rosbag)")

    def process_depth(self, msg):
        #ros2 image to opencv numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        print(f"Depth Image shape: {depth_image.shape}, Min: {np.min(depth_image)}, Max: {np.max(depth_image)}")
        
        #convert depth from millimeters to meters
        depth_image = depth_image.astype(np.float32) / 1000.0

        #clamp depth between 0 to 10
        depth_image = np.clip(depth_image, 0, 10)
        print(f"Normalized Depth Image: Min: {np.min(depth_image)}, Max: {np.max(depth_image)}")

        #convert cv2 to 32 bit single channel grayscale depth map as (ros image)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        depth_msg.header = msg.header  # Keep original timestamps
        self.publisher.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
