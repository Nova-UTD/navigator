#!/usr/bin/env python3
"""Draw a crosshair at (u, v) on a camera image for pixel_to_world checks.

Usage:
  ros2 run frame_tf_client overlay 400 350
  ros2 run frame_tf_client overlay 400 350 rgb_left

Then in RViz: Add → Image, topic /pixel_query_overlay
Compare that marked spot to the red sphere from pixel_to_world (map view).

Camera name → image topic (this stack's remaps):
  rgb_front  → /cameras/camera0
  rgb_right  → /cameras/camera1
  rgb_back   → /cameras/camera2
  rgb_left   → /cameras/camera3
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# Matches carla_interface remaps in launch.carla_interface.py
CAMERA_IMAGE_TOPICS = {
    "rgb_front": "/cameras/camera0",
    "rgb_right": "/cameras/camera1",
    "rgb_back": "/cameras/camera2",
    "rgb_left": "/cameras/camera3",
}


class PixelOverlay(Node):
    def __init__(self, u: float, v: float, camera_name: str):
        super().__init__("pixel_query_overlay")
        self.u = int(round(u))
        self.v = int(round(v))
        self.camera_name = camera_name
        self.bridge = CvBridge()

        topic = CAMERA_IMAGE_TOPICS.get(camera_name)
        if topic is None:
            raise ValueError(
                f"Unknown camera '{camera_name}'. "
                f"Choose one of: {', '.join(CAMERA_IMAGE_TOPICS)}"
            )

        self.sub = self.create_subscription(
            Image, topic, self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, "/pixel_query_overlay", 10)
        self.get_logger().info(
            f"Overlay ({self.u},{self.v}) on {camera_name} [{topic}] → /pixel_query_overlay"
        )

    def _cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        u = min(max(self.u, 0), w - 1)
        v = min(max(self.v, 0), h - 1)

        # Crosshair only (label in a corner so it isn't mistaken for geometry)
        color = (0, 0, 255)  # red BGR
        cv2.drawMarker(
            img, (u, v), color,
            markerType=cv2.MARKER_CROSS, markerSize=28, thickness=2)
        cv2.circle(img, (u, v), 14, color, 2)

        label = f"{self.camera_name}  ({u},{v})"
        cv2.rectangle(img, (8, 8), (8 + 12 * len(label), 36), (0, 0, 0), -1)
        cv2.putText(
            img, label, (14, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print("Usage: ros2 run frame_tf_client overlay <u> <v> [camera_name]")
        print("Example: ros2 run frame_tf_client overlay 400 350")
        print("Example: ros2 run frame_tf_client overlay 400 350 rgb_left")
        print(f"Cameras: {', '.join(CAMERA_IMAGE_TOPICS)}")
        rclpy.shutdown()
        return

    camera = sys.argv[3] if len(sys.argv) > 3 else "rgb_front"
    try:
        node = PixelOverlay(float(sys.argv[1]), float(sys.argv[2]), camera)
    except ValueError as ex:
        print(ex)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
