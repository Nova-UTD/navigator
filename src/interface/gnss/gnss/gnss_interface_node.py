import io
import math
from dataclasses import dataclass

import pynmea2
import pyproj
import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_EQUA = 6378137.0

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

class GnssInterfaceNode(Node):

    def __init__(self):
        super().__init__('gnss_interface_node')

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.status = DiagnosticStatus()
        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_statuses', 1)

        self.clock = Clock().clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.retry_connection_timer = self.create_timer(
            1.0, self.connectToPort)
        
        self.odom_pub = self.create_publisher(Odometry, '/gnss/odometry', 1)
        self.navsat_pub = self.create_publisher(NavSatFix, '/gnss/fix', 1)

        self.lat0 = 32.989488 # TODO: Get this foom the map manager
        self.lon0 = -96.750437
        utm_zone = 14
        self.proj = pyproj.Proj(proj='utm', zone=utm_zone, ellipsis='WGS84', preserve_units=True)

    def getOdomMsg(self, lat: float, lon: float) -> Odometry:
        x0, y0 = self.proj(latitude=self.lat0, longitude=self.lon0)

        x, y = self.proj(longitude=lon, latitude=lat)

        position_x = x - x0
        position_y = y - y0

        msg = Odometry()
        msg.pose.pose.position.x = position_x
        msg.pose.pose.position.y = position_y

        return msg


    def connectToPort(self):
        # TODO: Stabilize this device path somehow
        self.bus = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:6.4.4.3.1:1.0', 115200, timeout=0.05)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.bus,self.bus))

        while True:
            try:
                line = self.sio.readline()
                msg = pynmea2.parse(line)


                if msg.sentence_type == "GGA":
                    navsat_msg = NavSatFix()
                    navsat_msg.header.frame_id = 'gnss'
                    navsat_msg.header.stamp = self.clock
                    navsat_msg.latitude = msg.latitude
                    navsat_msg.longitude = msg.longitude
                    navsat_msg.altitude = msg.altitude

                    self.navsat_pub.publish(navsat_msg)

                    odom_msg = self.getOdomMsg(msg.latitude, msg.longitude)
                    odom_msg.header = navsat_msg.header


                    self.odom_pub.publish(odom_msg)
                    print(f"{msg.latitude}, {msg.longitude}")
                # else:
                    # print(msg.sentence_type)
                # print(repr(msg))
            except serial.SerialException as e:
                print('Device error: {}'.format(e))
                break
            except pynmea2.ParseError as e:
                # print('Parse error: {}'.format(e))
                continue
        return

    def initStatusMsg(self) -> DiagnosticStatus:
        status = DiagnosticStatus()

        status.name = self.get_name()

        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock.sec+self.clock.nanosec*1e-9)
        status.values.append(stamp)

        status.level = DiagnosticStatus.OK

        return status

    def clockCb(self, msg: Clock):
        self.clock = msg.clock


def main(args=None):
    rclpy.init(args=args)
    gnss_interface_node = GnssInterfaceNode()
    rclpy.spin(gnss_interface_node)
    gnss_interface_node.destroy_node()
    rclpy.shutdown()
