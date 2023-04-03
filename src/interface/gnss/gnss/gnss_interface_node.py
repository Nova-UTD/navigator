import io
import math
from dataclasses import dataclass

import pynmea2
import pyproj
import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


class GnssInterfaceNode(Node):

    def __init__(self):
        super().__init__('gnss_interface_node')

        self.sio = None
        self.orientation = Quaternion()
        self.heading = 0.0
        self.cached_odom = None

        self.tf_broadcaster = TransformBroadcaster(self)

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

        self.speed = 0.0

        self.read_timer = self.create_timer(0.05, self.getData)
        self.tf_timer = self.create_timer(0.05, self.publishTf)

    def publishTf(self):
        if self.cached_odom is None:
            return

        # Publish our map->base_link tf
        t = TransformStamped()
        t.header = self.cached_odom.header
        t.child_frame_id = self.cached_odom.child_frame_id
        transl = Vector3()
        transl.x = self.cached_odom.pose.pose.position.x
        transl.y = self.cached_odom.pose.pose.position.y
        transl.z = self.cached_odom.pose.pose.position.z
        t.transform.translation = transl
        t.transform.rotation = self.cached_odom.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t)

    def getData(self):

        if self.sio is None:
            return
        
        self.status = self.initStatusMsg()
        
        # Try to read the next 30 lines from the serial port
        for i in range(30):
            try:
                line = self.sio.readline()
                msg = pynmea2.parse(line)


                if msg.sentence_type == "GGA":
                    navsat_msg = NavSatFix()
                    navsat_msg.header.frame_id = 'base_link'
                    navsat_msg.header.stamp = self.clock
                    navsat_msg.latitude = msg.latitude
                    navsat_msg.longitude = msg.longitude
                    try:
                        navsat_msg.altitude = msg.altitude
                    except AssertionError as e:
                        self.status.message = f"{e}. Alt was {msg.altitude}"
                        self.status.level = DiagnosticStatus.WARN

                    self.navsat_pub.publish(navsat_msg)

                    odom_msg = self.getOdomMsg(msg.latitude, msg.longitude)
                    odom_msg.header = navsat_msg.header
                    odom_msg.header.frame_id = 'map'
                    odom_msg.child_frame_id = 'base_link'


                    self.odom_pub.publish(odom_msg)
                    self.cached_odom = odom_msg
                    # print(f"{msg.latitude}, {msg.longitude}")

                elif msg.sentence_type == "VTG":
                    if msg.spd_over_grnd_kmph is None:
                        return
                    self.speed = msg.spd_over_grnd_kmph * 0.277778
                    if msg.true_track is not None and self.speed > 1.0:

                        hdg_degrees = 90. - msg.true_track

                        if hdg_degrees < 0.:
                            hdg_degrees += 360.
                        elif hdg_degrees >= 360.:
                            hdg_degrees -= 360.

                        q = Quaternion()
                        hdg_radians = hdg_degrees * math.pi / 180.0

                        try:
                            q.w = math.cos(hdg_radians / 2)
                            q.z = math.sin(hdg_radians / 2)
                        except ValueError as e:
                            self.get_logger().error(f"{hdg_radians} was out of bounds!")
                        self.orientation = q
                        self.heading = hdg_radians

                self.status_pub.publish(self.status)
            except serial.SerialException as e:
                self.status.message = 'Device error: {}'.format(e)
                self.status.level = DiagnosticStatus.ERROR
                return
            except pynmea2.ParseError as e:
                # print('Parse error: {}'.format(e))
                return

    def getOdomMsg(self, lat: float, lon: float) -> Odometry:
        x0, y0 = self.proj(latitude=self.lat0, longitude=self.lon0)

        x, y = self.proj(longitude=lon, latitude=lat)

        position_x = x - x0
        position_y = y - y0

        msg = Odometry()
        msg.pose.pose.position.x = position_x
        msg.pose.pose.position.y = position_y

        msg.pose.pose.orientation = self.orientation

        msg.twist.twist.linear.x = math.cos(self.heading) * self.speed
        msg.twist.twist.linear.y = math.sin(self.heading) * self.speed

        return msg


    def connectToPort(self):
        # TODO: Stabilize this device path somehow
        self.bus = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:6.4.4.4.4.1:1.0', 115200, timeout=0.05)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.bus,self.bus))
                
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
