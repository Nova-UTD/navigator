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
        npcloud = rnp.numpify(msg)
        # ring_mask = npcloud[:][4] < 3
        for row in npcloud:
            if row['ring'] <= 4:
                self.get_logger().info('I heard: "%s"' % row)
        # npcloud = npcloud[mask]
        # self.get_logger().info('I heard: "%s"' % npcloud)


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