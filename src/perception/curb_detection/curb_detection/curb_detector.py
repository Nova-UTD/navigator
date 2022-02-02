import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from sensor_msgs.msg import PointCloud2


class CurbDetector(Node):

    def __init__(self):
        super().__init__('curb_detector')
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            10)

    def listener_callback(self, msg: PointCloud2):
        data = rnp.numpify(msg)
        data: np.ndarray
        self.get_logger().info('I heard: "%s"' % data)


def main(args=None):
    rclpy.init(args=args)

    curb_detector = CurbDetector()

    rclpy.spin(curb_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    curb_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()