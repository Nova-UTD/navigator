import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

def norm_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class Turtle1Controller(Node):
    def __init__(self):
        super().__init__('turtle1_controller')
        self.goal = None
        self.pose = None
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.sub_goal = self.create_subscription(Point, '/mouse_goal', self.on_goal, 10)
        self.timer = self.create_timer(0.03, self.loop)
        self.k_lin = 2.0
        self.k_ang = 6.0
        self.stop_dist = 0.15

    def on_pose(self, msg): self.pose = msg
    def on_goal(self, msg): self.goal = (msg.x, msg.y)

    def loop(self):
        if self.pose is None or self.goal is None:
            self.pub.publish(Twist())
            return
        dx = self.goal[0] - self.pose.x
        dy = self.goal[1] - self.pose.y
        dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        ang_err = norm_angle(target_theta - self.pose.theta)
        cmd = Twist()
        if dist > self.stop_dist:
            cmd.linear.x = max(min(self.k_lin * dist, 2.0), -2.0)
            cmd.angular.z = max(min(self.k_ang * ang_err, 4.0), -4.0)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = Turtle1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
