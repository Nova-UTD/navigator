import pygame
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, ColorRGBA, Float32
from geometry_msgs.msg import PoseStamped, Polygon, Point32, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from voltron_msgs.msg import CostedPaths, CostedPath, Zone, ZoneArray, Obstacle3DArray, Obstacle3D, Trajectory
import math
import numpy as np


class NovaUXNode(Node):

    def __init__(self):
        super().__init__('nova_ux_node')

        self.desired_speed_sub = self.create_subscription(
            Float32, '/control/target_speed', self.target_speed_cb, 10)
        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10
        )
        self.target_speed = 0.0  # m/s
        self.ground_speed = 0.0
        pygame.mixer.init(size=32)
        self.sound_timer = self.create_timer(.5, self.play_sinewave)

    def gnss_cb(self, msg: Odometry):
        linear_twist = msg.twist.twist.linear
        self.ground_speed = math.sqrt(linear_twist.x**2 +
                                 linear_twist.y**2 +
                                 linear_twist.z**2)
        print(f"{self.ground_speed} m/s")
        # self.play_sinewave()

    def play_sinewave(self):
        freq = (self.ground_speed - self.target_speed) * 100
        # freq = 220
        print(freq)
        buffer = np.sin(2 * np.pi * np.arange(44100) * freq / 44100).astype(np.float32)
        sound = pygame.mixer.Sound(buffer)
        sound.play(0)
        pygame.time.wait(int(sound.get_length() * 1000))

    def target_speed_cb(self, msg: Float32):
        print(f"Target speed is {msg.data}")
        self.target_speed = msg.data


def main(args=None):
    rclpy.init(args=args)

    viz_subscriber = NovaUXNode()

    rclpy.spin(viz_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
