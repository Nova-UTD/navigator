import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float
from nav_msgs.msg import Odometry
from voltron_msgs.msg import CanFrame
import math

ENABLE_CLUTCH =           0x0F0A008000000000
ENABLE_CLUTCH_AND_MOTOR = 0x0F0A00C000000000

def int_position_to_masked(p):
    lo = p & 0xff
    hi = (p >> 8) & 0x3f
    return (lo << 40) | (hi << 32)

class MyNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/incoming_can_messages_can0',
            self.listener_callback,
            1)
        self.possubscription = self.create_subscription(
            Float,
            '/throttle/position',
            self.pos_callback,
            10)
        self.publisher_ = self.create_publisher(CanFrame, '/outgoing_can_messages_can0', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.publisher_
        self.possubscription
        self.desired_position = 0x
        self.valid = false
    
    def timer_callback(self):
        iden = 0x00FF0000
        msg = CanFrame()
        msg.identifier = iden
        msg.data = 0xF10098FFFFFFFFFF
        self.publisher_.publish(msg)
        if not valid:
            return
        msg = CanFrame()
        msg.identifier = iden
        msg.data = 
    
    def pos_callback(self, msg):
        if msg != self.desired_position:
            self.

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