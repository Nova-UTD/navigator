'''
Package: map_management
   File: gnss_processing_node.py
 Author: Will Heitman (w at heit dot mn)

Very simple node to convert raw GNSS odometry into a map->base_link transform.
'''

import math
import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from builtin_interfaces.msg import Time

from carla_msgs.msg import CarlaSpeedometer, CarlaWorldInfo
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix

from tf2_ros import TransformBroadcaster
from xml.etree import ElementTree as ET

from scipy.spatial.transform import Rotation as R


def toRadians(degrees: float):
    return degrees * math.pi / 180.0


def latToScale(lat: float) -> float:
    """Return UTM projection scale

    Args:
        lat (float): degrees latitude

    Returns:
        float: projection scale
    """
    return math.cos(toRadians(lat))


EARTH_RADIUS_EQUA = 6378137.0


class GnssProcessingNode(Node):

    def __init__(self):
        super().__init__('gnss_processing_node')

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/gnss_raw', self.raw_odom_cb, 10
        )
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/carla/hero/gnss', self.gnssCb, 10
        )

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/carla/hero/imu', self.imuCb, 10
        )

        self.world_info_sub = self.create_subscription(
            CarlaWorldInfo, '/carla/world_info', self.worldInfoCb, 10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odometry/gnss_processed', 10
        )

        self.raw_odom_pub = self.create_publisher(
            Odometry, '/odometry/gnss_raw', 10
        )

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1
        )

        self.status_timer = self.create_timer(1.0, self.updateStatus)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cached_imu = Imu()
        self.latest_timestamp = Time()
        self.clock = Clock()
        self.lat0 = None
        self.lon0 = None

        # Make a queue of previous poses for our weighted moving average
        # where '10' is the size of our history
        self.history_size = 5
        self.previous_x_vals = np.zeros((self.history_size))
        self.previous_y_vals = np.zeros((self.history_size))
        self.previous_z_vals = np.zeros((self.history_size))
        self.wma_pose = Pose()

    def _parse_header_(self, root: ET.Element) -> dict:
        header = root.find('header')

        # Find our geoReference tag, which has contents like this:
        # "<![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]>"
        geo_reference: str = header.find('geoReference').text
        geo_reference_parts = geo_reference.split('+')

        for part in geo_reference_parts:
            if part.startswith('lon_0'):
                lon0 = float(part[6:])
            elif part.startswith('lat_0'):
                lat0 = float(part[6:])

        return (lat0, lon0)

    def worldInfoCb(self, msg: CarlaWorldInfo):
        if self.lat0 is not None:
            return
        if msg.opendrive == "":
            return
        root = ET.fromstring(msg.opendrive)
        self.lat0, self.lon0 = self._parse_header_(root)
        self.get_logger().info("World info received.")

    def latlonToMercator(self, lat: float, lon: float, scale: float) -> tuple:
        """Convert geodetic coords (in degrees) to UTM coords (meters)

        Copied to match CARLA:
        [LatLonToMercator](https://github.com/carla-simulator/carla/blob/fe3cb6863a604f5c0cf8b692fe2b6300b45b5999/LibCarla/source/carla/geom/GeoLocation.cpp#L38)

        Args:
            lat (float): degrees
            lon (float): degrees
            scale (float): projection scale (see latToScale)

        Returns:
            tuple: (x,y) in UTM meters
        """
        x = scale * toRadians(lon) * EARTH_RADIUS_EQUA
        y = scale * EARTH_RADIUS_EQUA * \
            math.log(math.tan((90.0+lat) * math.pi/360.0))

        return (x, y)

    def gnssCb(self, msg: NavSatFix):
        if self.lat0 is None:
            return

        x0, y0 = self.latlonToMercator(
            self.lat0, self.lon0, latToScale(self.lat0))

        # TODO: Should scale here be from self.lat0?
        x, y = self.latlonToMercator(
            msg.latitude, msg.longitude, latToScale(msg.latitude))

        position_x = x - x0
        position_y = y - y0

        odom_msg = Odometry()

        # The odometry is for the current time-- right now
        odom_msg.header.stamp = self.clock.clock

        # The odometry is the car's location on the map,
        # so the child frame is "base_link"
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        pos = Point()

        pos.x = position_x
        pos.y = position_y
        pos.z = msg.altitude  # TODO: Make relative to georeference
        odom_msg.pose.pose.position = pos
        self.raw_odom_pub.publish(odom_msg)

    def imuCb(self, msg: Imu):
        self.cached_imu = msg
        msg.header.stamp

    def clockCb(self, msg: Clock):
        self.clock = msg

    def updateStatus(self):
        status = DiagnosticStatus()
        status.name = self.get_name()  # Get the node name

        # Check if message is stale
        current_time = self.clock.clock.sec + self.clock.clock.nanosec * 1e-9
        data_received_time = self.latest_timestamp.sec + \
            self.latest_timestamp.nanosec * 1e-9
        time_since_data_received = current_time - data_received_time > 1.0
        if time_since_data_received > 1.0:
            status.level = DiagnosticStatus.STALE
            status.message = f"No GNSS data received in {time_since_data_received} seconds"
        else:
            status.level = DiagnosticStatus.OK
            # No message necessary if OK.
        self.status_pub.publish(status)

    def _update_odom_weighted_moving_average_(self, current_pos: Point):
        # Calculate noisy yaw from the change in position
        # old_pose = self.previous_poses[self.history_size-1]

        # Calculate the weighted moving average (WMA) for odometry
        # 1. Discard oldest reading and add newest to 'queue'

        self.previous_x_vals = np.roll(self.previous_x_vals, -1)
        self.previous_y_vals = np.roll(self.previous_y_vals, -1)
        self.previous_z_vals = np.roll(self.previous_z_vals, -1)
        self.previous_x_vals[self.history_size-1] = current_pos.x
        self.previous_y_vals[self.history_size-1] = current_pos.y
        self.previous_z_vals[self.history_size-1] = current_pos.z

        # 2. Generate weight array: [1,2,3,...,n]
        weights = np.arange(0, self.history_size)

        # 3. Use a numpy functions to handle the rest :->)
        wma_x = np.average(self.previous_x_vals, axis=0, weights=weights)
        wma_y = np.average(self.previous_y_vals, axis=0, weights=weights)
        wma_z = np.average(self.previous_z_vals, axis=0, weights=weights)

        wma_pose = Pose()
        wma_pose.position.x = wma_x
        wma_pose.position.y = wma_y
        wma_pose.position.z = wma_z

        # IMU orientation is buggy. See imu_cb note.
        q = self.cached_imu.orientation
        heading_x = q.x * -0.48
        heading_y = q.y * 0.48
        wma_yaw = q

        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        wma_yaw = rpy[1] * -np.pi/2 + np.pi

        # q = cos(theta/2) + sin(theta/2)(xi + yj + zk)
        # Set x, y = 0 s.t. theta = yaw
        wma_pose.orientation.w = math.cos(wma_yaw/2)
        wma_pose.orientation.x = 0.0
        wma_pose.orientation.y = 0.0
        wma_pose.orientation.z = math.sin(wma_yaw/2)

        self.wma_pose = wma_pose

    def _quat_to_yaw_(self, q: Quaternion):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return yaw

    def raw_odom_cb(self, msg: Odometry):

        current_pos: Point = msg.pose.pose.position

        # Form an odom message to store our result
        odom_msg = Odometry()

        # Copy the header from the GNSS odom message
        odom_msg.header = msg.header
        odom_msg.child_frame_id = msg.child_frame_id

        self._update_odom_weighted_moving_average_(current_pos)

        odom_msg.pose.pose = self.wma_pose

        odom_msg.pose.covariance[0] = 2.0  # This is the variance of x
        odom_msg.pose.covariance[7] = 2.0  # This is the variance of y

        # LOCK Z to 0
        odom_msg.pose.pose.position.z = 0.7

        # Publish our odometry message, converted from GNSS
        self.odom_pub.publish(odom_msg)

        # Update our timestamp (used to check staleness)
        self.latest_timestamp = odom_msg.header.stamp

        # self.get_logger().info("{}".format(str(self.previous_y_vals)))
        # self.get_logger().info(f"CURRENT Y: {current_pos.y}")

        # Publish our map->base_link tf
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'hero'
        transl = Vector3()
        transl.x = self.wma_pose.position.x
        transl.y = self.wma_pose.position.y
        transl.z = self.wma_pose.position.z
        t.transform.translation = transl
        t.transform.rotation = self.wma_pose.orientation

        # Uncomment to enable direct map->base_link tf
        # self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    gnss_processing_node = GnssProcessingNode()

    rclpy.spin(gnss_processing_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gnss_processing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
