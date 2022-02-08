import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from voltron_msgs.msg import CanFrame
import math


class MyNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/incoming_can_messages_can0',
            self.listener_callback,
            1)
        self.publisher_ = self.create_publisher(CanFrame, '/outgoing_can_messages_can0', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.publisher_
    
    def timer_callback(self):
        msg = CanFrame()
        msg.identifier = 0xFF0000
        msg.data = 0xF10098FFFFFFFFFF
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % (msg))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MyNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    self.file.close()


if __name__ == '__main__':
    main()