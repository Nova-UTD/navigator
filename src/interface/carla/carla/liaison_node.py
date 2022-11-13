'''
Package:   carla
Filename:  liaison_node.py
Author:    Will Heitman (w@heit.mn)

This node handles our ROS-based communication with the
CARLA Leaderboard. This includes publishing to /carla/hero/status,
but more functionality will be added in the future.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LeaderboardLiaisonNode(Node):

    def __init__(self):
        # self.get_logger().info("woo")
        super().__init__('liaison_node')
        self.status_pub = self.create_publisher(Bool, '/carla/hero/status', 1)
        status_timer = self.create_timer(1.0, self.publish_hero_status)

    def publish_hero_status(self):
        """Publish a true Bool to the leaderboard status topic"""
        status = Bool()
        status.data = True # This means "good to go!"
        self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)

    liaison_node = LeaderboardLiaisonNode()

    rclpy.spin(liaison_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    liaison_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
