'''
Package:   recording
Filename:  recorder.py
Author:    Will Heitman (w at heit.mn)

Subscribes to:
/grid/occupancy/current (nav_msgs/OccupancyGrid)
/cameras/stitched (sensor_msgs/Image)

Converts these to numpy arrays and saves them to a file
'''

from datetime import datetime

import numpy as np
import os
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

# Thanks to ros2_numpy!
name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":  (np.uint8,  1),
    "bayer_bggr8":  (np.uint8,  1),
    "bayer_gbrg8":  (np.uint8,  1),
    "bayer_grbg8":  (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}

# Larger = fewer writes, but they take longer.
PREFERRED_MEMORY_USAGE = 1e9  # bytes.


def imageToNumpy(msg: Image):
    # https://github.com/Box-Robotics/ros2_numpy/blob/foxy-devel/ros2_numpy/image.py
    if not msg.encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(msg.encoding))

    dtype_class, channels = name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    shape = (msg.height, msg.width, channels)

    data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
    data.strides = (
        msg.step,
        dtype.itemsize * channels,
        dtype.itemsize
    )

    if channels == 1:
        data = data[..., 0]
    return data


class recorder(Node):

    def __init__(self):
        super().__init__('recorder')

        self.current_time = 0.0
        self.occupancy_frames = []
        self.current_occ_msg = None

        self.camera_frames = []
        self.current_cam_msg = None

        self.odom_frames = []
        self.current_odom_msg = None

        self.stamps = []

        self.setUpDirectory()

        current_occ_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccCb, 10)

        cameraSub = self.create_subscription(
            Image, '/cameras/stitched', self.camCb, 10)

        odomSub = self.create_subscription(
            Odometry, '/gnss/odometry', self.odomCb, 10)

        clockSub = self.create_subscription(Clock, '/clock', self.clockCb, 1)

        appendTimer = self.create_timer(0.2, self.addToRecording)

    def setUpDirectory(self):
        now = datetime.now()
        date_string = now.strftime('%Y-%m-%d_%H-%M-%S')

        self.directory = f'/navigator/recordings/{date_string}'
        os.mkdir(self.directory)

    def addToRecording(self):

        if self.current_cam_msg is None:
            self.get_logger().warning("Cam not received")
            return
        if self.current_occ_msg is None:
            self.get_logger().warning("Occ not received")
            return

        if self.current_odom_msg is None:
            self.get_logger().warning("Odom not received")
            return

        # Convert occupancy to a numpy array
        occ_array = np.asarray(self.current_occ_msg.data, dtype=np.int8).reshape(
            self.current_occ_msg.info.height, self.current_occ_msg.info.width)
        self.occupancy_frames.append(occ_array)

        # Convert image to a numpy array
        image_array = imageToNumpy(self.current_cam_msg)
        self.camera_frames.append(image_array)

        # Add a timestamp
        self.stamps.append(self.current_time)

        # Is our data large enough to be written to disk?
        occ_mem_usage = self.occupancy_frames[0].nbytes * \
            len(self.occupancy_frames)

        image_mem_usage = self.camera_frames[0].nbytes * \
            len(self.camera_frames)

        total_mem_usage = occ_mem_usage + image_mem_usage

        print(f"{int(total_mem_usage/1e6)} MB used ")

        if total_mem_usage > PREFERRED_MEMORY_USAGE:
            self.writeToFile()

        return

    def camCb(self, msg: Image):
        self.current_cam_msg = msg

    def writeToFile(self):
        total_occupancy_frames = np.asarray(self.occupancy_frames)
        total_image_frames = np.asarray(self.camera_frames)
        stamps = np.asarray(self.stamps)

        now = datetime.now()
        self.get_logger().info("Writing to file!")
        date_string = now.strftime('%Y-%m-%d_%H-%M-%S')

        np.savez(
            f'{self.directory}/{date_string}',
            cam=total_image_frames,
            occ=total_occupancy_frames,
            time=stamps
        )

        self.camera_frames.clear()
        self.occupancy_frames.clear()
        self.stamps.clear()

    def camCb(self, msg: Image):
        self.current_cam_msg = msg
        # self.get_logger().info('Got cam!')
        return

    def clockCb(self, msg: Clock):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def currentOccCb(self, msg: OccupancyGrid):
        self.current_occ_msg = msg

        # self.get_logger().info('Got occ!')
        return

    def odomCb(self, msg: Odometry):
        self.odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = recorder()
    try:
        rclpy.spin(node)
    finally:
        node.writeToFile()
    recorder.destroy_node()
    rclpy.shutdown()
