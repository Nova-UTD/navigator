'''
Package:   recording
Filename:  recorder.py
Author:    Will Heitman (w at heit.mn)

Subscribes to:
/cameras/stitched (sensor_msgs/Image)
/gnss/odometry (nav_msgs/Odometry)
/grid/occupancy/current (nav_msgs/OccupancyGrid)

Converts these to numpy arrays and saves them to a .npz file

See https://numpy.org/doc/stable/reference/generated/numpy.savez.html#numpy.savez

Note that we do not use compression (np.savez_compressed). It just takes too long,
and our data likely can't be compressed very much.
'''

from datetime import datetime

import numpy as np
import os
import rclpy

# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
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
RAM_LIMIT = 1e8  # bytes.
FRAME_RATE = 5  # FPS. Messages will be recorded at this rate.


def imageToNumpy(msg: Image):
    """Turn an Image message into a Numpy array

    Args:
        msg (Image): Image message

    Raises:
        TypeError: Unsupported encoding

    Returns:
        np.ndarray: Converted np array
    """
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

        # Objects to store data in memory
        self.current_time = 0.0
        self.occupancy_frames = []
        self.current_occ_msg = None
        self.camera_frames = []
        self.current_cam_msg = None
        self.odom_frames = []
        self.current_odom_msg = None
        self.stamps = []
        self.total_disk_usage = 0  # Bytes

        self.recording_enabled = False

        self.setUpDirectory()

        current_occ_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccCb, 10)

        camera_sub = self.create_subscription(
            Image, '/cameras/camera0', self.camCb, 10)

        odom_sub = self.create_subscription(
            Odometry, '/gnss/odometry', self.odomCb, 10)

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)

        append_timer = self.create_timer((1.0/FRAME_RATE), self.addToRecording)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

    def getStatus(self):
        msg = DiagnosticStatus()
        msg.name = 'recording'
        msg.level = DiagnosticStatus.OK

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.current_time)
        msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = 'state'
        if self.recording_enabled:
            state_kv.value = 'recording'
        else:
            state_kv.value = 'idle'

        msg.values.append(state_kv)

        disk_usage_kv = KeyValue()
        disk_usage_kv.key = 'disk_usage'
        disk_usage_kv.value = f"{self.total_disk_usage}"
        msg.values.append(disk_usage_kv)

        return msg

    def setUpDirectory(self):
        """On start, creates a directory to hold our recordings.
        """
        now = datetime.now()
        date_string = now.strftime('%y-%m-%d_%H-%M-%S')

        self.directory = f'/navigator/recordings/{date_string}'
        os.mkdir(self.directory)

    def addToRecording(self):
        """
        Add the latest received data to RAM.
        Once RAM is full (RAM_LIMIT), calls writeToFile().
        """

        if not self.recording_enabled:
            status_msg = self.getStatus()
            self.status_pub.publish(status_msg)
            return

        if self.current_cam_msg is None:
            self.get_logger().warning("Cam not received")
            return
        if self.current_occ_msg is None:
            self.get_logger().warning("Occ not received")
            return

        if self.current_odom_msg is None:
            self.get_logger().warning("Odom not received")
            return

        # Process OccupancyGrid
        occ_array = np.asarray(self.current_occ_msg.data, dtype=np.int8).reshape(
            self.current_occ_msg.info.height, self.current_occ_msg.info.width)
        self.occupancy_frames.append(occ_array)

        # Process Image
        image_array = imageToNumpy(self.current_cam_msg)
        self.camera_frames.append(image_array)

        # Process Odometry
        self.current_odom_msg: Odometry
        pos = self.current_odom_msg.pose.pose.position
        quat = self.current_odom_msg.pose.pose.orientation
        yaw = 2*np.arccos(quat.w)
        vel = self.current_odom_msg.twist.twist.linear
        odom_array = np.asarray([pos.x, pos.y, yaw, vel.x, vel.y])
        self.odom_frames.append(odom_array)

        # Add a timestamp
        self.stamps.append(self.current_time)

        # Is our data large enough to be written to disk?
        total_mem_usage = self.getMemUsage()

        if total_mem_usage > RAM_LIMIT:
            self.writeToFile()

        # Publish status
        status_msg = self.getStatus()
        self.status_pub.publish(status_msg)

    def getMemUsage(self) -> int:
        occ_mem_usage = self.occupancy_frames[0].nbytes * \
            len(self.occupancy_frames)

        image_mem_usage = self.camera_frames[0].nbytes * \
            len(self.camera_frames)

        odom_mem_usage = self.odom_frames[0].nbytes * \
            len(self.odom_frames)

        total_mem_usage = occ_mem_usage + image_mem_usage + odom_mem_usage

        return total_mem_usage

    def writeToFile(self):
        total_occupancy_frames = np.asarray(self.occupancy_frames)
        total_odom_frames = np.asarray(self.odom_frames)
        total_image_frames = np.asarray(self.camera_frames)
        stamps = np.asarray(self.stamps)

        now = datetime.now()
        date_string = now.strftime('%y-%m-%d_%H-%M-%S')

        self.get_logger().info(
            f"Writing {int(self.getMemUsage() / 1e6)} MB to {self.directory}/{date_string}")

        self.total_disk_usage += self.getMemUsage()

        # https://numpy.org/doc/stable/reference/generated/numpy.savez
        np.savez(
            f'{self.directory}/{date_string}',
            cam=total_image_frames,
            occ=total_occupancy_frames,
            odom=total_odom_frames,
            time=stamps
        )

        # Free up RAM
        self.camera_frames.clear()
        self.occupancy_frames.clear()
        self.odom_frames.clear()
        self.stamps.clear()

    def camCb(self, msg: Image):
        """Caches the latest message, to be used by addToRecording()

        Args:
            msg (Image)
        """
        self.current_cam_msg = msg

    def clockCb(self, msg: Clock):
        """Caches the latest message, to be used by addToRecording()

        Args:
            msg (Clock)
        """
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def currentOccCb(self, msg: OccupancyGrid):
        """Caches the latest message, to be used by addToRecording()

        Args:
            msg (OccupancyGrid)
        """
        self.current_occ_msg = msg

    def odomCb(self, msg: Odometry):
        """Caches the latest message, to be used by addToRecording()

        Args:
            msg (Odometry)
        """
        self.current_odom_msg = msg

    def close(self):
        self.writeToFile()

        status_msg = DiagnosticStatus()
        status_msg.name = 'recording'
        status_msg.message = f"Playing stopped"
        status_msg.level = DiagnosticStatus.OK

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(0.0)
        status_msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = 'state'
        state_kv.value = f"idle"
        status_msg.values.append(state_kv)

        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = recorder()
    try:
        rclpy.spin(node)
    finally:
        # Write any remaining data to the file before closing.
        node.close()
    recorder.destroy_node()
    rclpy.shutdown()
