'''
Package:   recording
Filename:  player.py
Author:    Will Heitman (w at heit.mn)

Loads and plays all recordings from a given directory,
publishing the data as ROS messages.

Arguments (optional):

'from=hh-mm': Start recording at this file, continuing until either
              'to=' or end.
    Ex: 'from=17-36' starts at 5:36pm today
        'from=04-04_17-36' starts at April 4, 5:36pm

'to=': Same format as 'from'.

'src=': Source directory for recordings. Default: '/navigator/recordings'
'''

import os
import sys  # argv
from array import array as Array
from datetime import datetime
from time import sleep, strftime, strptime, time

import numpy as np
import rclpy
# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

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

FRAME_RATE = 5  # FPS. Messages will be replayed at this rate.
LOOP_FOREVER = True


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


def numpyToOccupancyGrid(arr):
    # https://github.com/Box-Robotics/ros2_numpy/blob/foxy-devel/ros2_numpy/occupancy_grid.py
    if not len(arr.shape) == 2:
        raise TypeError('Array must be 2D')
    if not arr.dtype == np.int8:
        raise TypeError('Array must be of int8s')

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        # We assume that the masked value are already -1, for speed
        arr = arr.data

    grid.data = Array('b', arr.ravel().astype(np.int8))
    grid.info = MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid


def numpyToImage(arr, encoding):
    if not encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(encoding))

    im = Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError("Array has {} channels, {} requires {}".format(
            channels, encoding, exp_channels
        ))
    if dtype_class != arr.dtype.type:
        raise TypeError("Array is {}, {} requires {}".format(
            arr.dtype.type, encoding, dtype_class
        ))

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big'
    )

    return im


def numpyToOdom(arr: np.ndarray, sec, nsec) -> Odometry:
    """

    Format: [pos.x, pos.y, yaw, vel.x, vel.y]

    Args:
        arr (np.ndarray): Input array

    Returns:
        Odometry: Converted message
    """
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nsec
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = arr[0]
    msg.pose.pose.position.y = arr[1]

    yaw = arr[2]
    msg.pose.pose.orientation.w = np.cos(yaw/2)  # Quaternion conversion
    msg.pose.pose.orientation.z = np.sin(yaw/2)

    msg.twist.twist.linear.x = arr[3]
    msg.twist.twist.linear.y = arr[4]

    return msg


class player(Node):

    def __init__(self):
        super().__init__('player')

        self.srcdir = '/replay'
        self.search_from = strptime('70-1-1_0-0-0', '%y-%m-%d_%H-%M-%S')
        self.search_to = strptime('30-1-1_0-0-0', '%y-%m-%d_%H-%M-%S')
        self.processArguments(sys.argv)


        self.records = self.getFilesInRange()
        self.get_logger().info(f"Playing {len(self.records)} records from {strftime('%m/%d/%Y',self.search_from)} to {strftime('%m/%d/%Y',self.search_to)}")
        # self.setUpDirectory()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_occ_grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/occupancy/current', 10)

        self.camera_pub = self.create_publisher(
            Image, '/cameras/stitched', 10)

        self.odom_pub = self.create_publisher(
            Odometry, '/gnss/odometry', 10)

        self.clock_pub = self.create_publisher(Clock, '/clock', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        # readTimer = self.create_timer((1.0/FRAME_RATE), self.playNextFrame)

        while True:
            self.run()
            if not LOOP_FOREVER:
                break

        self.get_logger().info("Recording complete.")

    def getStatus(self, len: int, idx: int, sec: int, nsec: int, record_name):
        msg = DiagnosticStatus()
        msg.name = 'recording'
        msg.message = f"Replaying record {record_name}, ({idx[0]}/{len})"
        msg.level = DiagnosticStatus.OK

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(sec+nsec*1e-9)
        msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = 'state'
        state_kv.value = f"replaying"
        msg.values.append(state_kv)

        progress = KeyValue()
        progress.key = 'progress'
        progress.value = f"{idx[0]}/{len}"
        msg.values.append(progress)

        return msg

    def run(self):
        """
        Iterate through all records, convert the data to ROS messages,
        and publish them.
        """
        for record in self.records:
            # Load the arrays from .npz
            npz: np.Npz = np.load(record)

            self.get_logger().info(f"Playing {record}")

            cam = npz['cam']
            occ = npz['occ']
            odom = npz['odom']
            times = npz['time']

            for idx, time in np.ndenumerate(times):

                time_sec = int(times[idx])
                time_nsec = int((times[idx]-time_sec) * 1e9)

                # Prepare camera message
                image_msg: Image = numpyToImage(cam[idx], 'bgr8')

                # Prepare occupancy grid message
                occ_msg: OccupancyGrid = numpyToOccupancyGrid(occ[idx])
                occ_msg.header.frame_id = 'base_link'
                occ_msg.header.stamp.sec = time_sec
                occ_msg.header.stamp.nanosec = time_nsec
                occ_msg.info.resolution = 1./3.  # Meters
                occ_msg.info.origin.position.x = -64.0 * (1. / 3.)
                occ_msg.info.origin.position.y = -64.0 * (1. / 3.)

                # Prepare odom message
                odom_msg = numpyToOdom(odom[idx], time_sec, time_nsec)

                # Use odom to broadcast map->base_link tf
                self.sendTf(odom_msg)

                self.camera_pub.publish(image_msg)
                self.current_occ_grid_pub.publish(occ_msg)
                self.odom_pub.publish(odom_msg)

                status_msg = self.getStatus(
                    len(times), idx, time_sec, time_nsec, record)
                self.status_pub.publish(status_msg)

                sleep(0.1)

            npz.close()

    def sendTf(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def getFilesInRange(self):
        """Sort subdirectories from oldest to newest.
        Only keep directories in desired range.
        For first and last subdirectory, only keep files in desired range.

        Add all files to a big ordered (sorted) list. Return it.

        Only works on recordings made in the 21st century ;)
        """
        files = []
        all_directories = os.listdir(self.srcdir)
        all_directories.sort()

        with os.scandir(self.srcdir) as entries:
            for entry in entries:
                if entry.is_file():
                    continue  # Found a file. Skip.
                try:
                    folder_time = strptime(
                        entry.name, '%y-%m-%d_%H-%M-%S')

                    if folder_time < self.search_from:
                        continue  # Recording too old
                    elif folder_time > self.search_to:
                        continue  # Recording too new

                    with os.scandir(entry.path) as records:
                        for record in records:
                            if not record.is_file():
                                continue
                            if record.name.split('.')[-1] != 'npz':
                                self.get_logger().warning(
                                    f'Found invalid record "{record.name}". Records should be in the .npz format.')
                                continue
                            files.append(record.path)
                except ValueError as e:
                    self.get_logger().warning(str(e))
                    continue

        files_in_range = []

        for file in files:
            # Check if the file is in time range
            try:
                file_time = strptime(
                    file.split('/')[-1].split('.')[0], '%y-%m-%d_%H-%M-%S')

                if file_time < self.search_from:
                    continue  # Recording too old
                elif file_time > self.search_to:
                    continue  # Recording too new

                files_in_range.append(file)
            except ValueError as e:
                self.get_logger().warning(str(e))
                continue

        files_in_range.sort()

        return files_in_range

    def processArguments(self, args: list):
        """See script header.

        Args:
            args (list)
        """
        for arg in args:
            arg: str
            if arg.find('from=') != -1:
                datestring = arg.split('=')[1]
                if len(datestring.split('-')) == 2:
                    year = datetime.now().year
                    self.search_from = strptime(f"{year}-{datestring}", '%Y-%m-%d')
                elif len(datestring.split('-')) == 3:
                    self.search_from = strptime(datestring, '%y-%m-%d')
                print(arg.split('=')[1])
            elif arg.find('src=') != -1:
                self.srcdir = arg.split('=')[1]

    def setUpDirectory(self):
        """On start, creates a directory to hold our recordings.
        """
        now = datetime.now()
        date_string = now.strftime('%Y-%m-%d_%H-%M-%S')

        self.directory = f'/navigator/recordings/{date_string}'
        os.mkdir(self.directory)

    def addToRecording(self):
        """
        Add the latest received data to RAM.
        Once RAM is full (RAM_LIMIT), calls writeToFile().
        """

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

    def getMemUsage(self) -> int:
        occ_mem_usage = self.occupancy_frames[0].nbytes * \
            len(self.occupancy_frames)

        image_mem_usage = self.camera_frames[0].nbytes * \
            len(self.camera_frames)

        odom_mem_usage = self.odom_frames[0].nbytes * \
            len(self.odom_frames)

        total_mem_usage = occ_mem_usage + image_mem_usage + odom_mem_usage

        return total_mem_usage

    def close(self):

        status_msg = DiagnosticStatus()
        status_msg.name = 'recording'
        status_msg.message = f"Playing stopped"
        status_msg.level = DiagnosticStatus.OK

        # stamp = KeyValue()
        # stamp.key = 'stamp'
        # stamp.value = str(0.0)
        # msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = 'state'
        state_kv.value = f"idle"
        status_msg.values.append(state_kv)

        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = player()
    try:
        rclpy.spin(node)
    finally:
        # Write any remaining data to the file before closing.
        node.close()
    player.destroy_node()
    rclpy.shutdown()
