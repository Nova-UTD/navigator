import rclpy
from rclpy.node import Node
from my_interfaces.msg import ChaseMode

class ModeToggle(Node):
    def __init__(self):
        super().__init__('mode_toggle')
        self.pub = self.create_publisher(ChaseMode, '/chase_mode', 10)
        self.mode = True
        self.timer = self.create_timer(10.0, self.flip)
        self.publish_now()

    def publish_now(self):
        msg = ChaseMode(chase=self.mode)
        self.pub.publish(msg)
        self.get_logger().info(f"Mode: {'CHASE' if self.mode else 'RUN AWAY'}")

    def flip(self):
        self.mode = not self.mode
        self.publish_now()

def main():
    rclpy.init()
    node = ModeToggle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
