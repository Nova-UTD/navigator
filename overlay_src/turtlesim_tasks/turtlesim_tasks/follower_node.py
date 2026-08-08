import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_interfaces.msg import ChaseMode

def norm_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.pose1 = None
        self.pose2 = None
        self.chase = True
        self.pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.sub1 = self.create_subscription(Pose, '/turtle1/pose', self.on_pose1, 10)
        self.sub2 = self.create_subscription(Pose, '/turtle2/pose', self.on_pose2, 10)
        self.subm = self.create_subscription(ChaseMode, '/chase_mode', self.on_mode, 10)
        self.timer = self.create_timer(0.03, self.loop)
        self.k_lin = 2.0
        self.k_ang = 6.0
        self.close_thresh = 0.5
        self.far_thresh   = 6.0

    def on_pose1(self, msg): self.pose1 = msg
    def on_pose2(self, msg): self.pose2 = msg
    def on_mode(self, msg):
        self.chase = bool(msg.chase)
        self.get_logger().info(f"Follower mode -> {'CHASE' if self.chase else 'RUN AWAY'}")

    def loop(self):
        cmd = Twist()
        if self.pose1 is None or self.pose2 is None:
            self.pub.publish(cmd); return
        dx = self.pose1.x - self.pose2.x
        dy = self.pose1.y - self.pose2.y
        dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        if self.chase:
            if dist > self.close_thresh:
                ang_err = norm_angle(target_theta - self.pose2.theta)
                cmd.linear.x  = max(min(self.k_lin * dist, 2.0), -2.0)
                cmd.angular.z = max(min(self.k_ang * ang_err, 4.0), -4.0)
        else:
            if dist < self.far_thresh:
                away_theta = norm_angle(target_theta + math.pi)
                ang_err = norm_angle(away_theta - self.pose2.theta)
                cmd.linear.x  = 2.0
                cmd.angular.z = max(min(self.k_ang * ang_err, 4.0), -4.0)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
