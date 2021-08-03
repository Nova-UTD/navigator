# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
    Subscribe to our NDT pose and republish as
    a VehicleKinematicState. Simple!
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from autoware_auto_msgs.msg import VehicleKinematicState
from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from autoware_auto_msgs.msg import VehicleControlCommand
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from voltron_msgs.msg import Gear
from lgsvl_msgs.msg import CanBusData
from lgsvl_msgs.msg import VehicleControlData
import math

class VehicleBridge(Node):

    def __init__(self):
        self.speed_mps = 0.0

        super().__init__('translator')
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/ndt_pose',
            self.pose_cb,
            10)

        self.gear_pub =  self.create_publisher(
            Gear, '/vehicle/gear', 10
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.kinematic_state_pub = self.create_publisher(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            10)

        self.vehicle_command_sub = self.create_subscription(
            VehicleControlCommand,
            '/vehicle/vehicle_command',
            self.vehicle_command_cb,
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleControlData,
            '/lgsvl/vehicle_control_cmd',
            10)

        self.svl_can_sub = self.create_subscription(
            CanBusData,
            '/lgsvl/state_report',
            self.svl_can_cb,
            10
        )
        # self.path_pub_ = self.create_publisher(Path, '/planning/path_viz', 10)
        # self.command_viz_pub_ = self.create_publisher(Marker, '/vehicle/vehicle_command_viz', 10)
        # self.hack_header = Header()

        # self.subscription  # prevent unused variable warning

    def timer_callback(self):
        gear_msg = Gear()
        gear_msg.state = 1;
        self.gear_pub.publish(gear_msg)

    def pose_cb(self, msg):
        position = msg.pose.pose.position
        self.get_logger().debug("Pose: {},{},{}".format(position.x, position.y, position.z))

        # Simply form and publish a VehicleKinematicState from NDT Pose
        kstate = VehicleKinematicState()
        kstate.header = msg.header
        kstate.state.x = position.x
        kstate.state.y = position.y

        self.kinematic_state_pub.publish(kstate)

        # printstring = "\n\n";
        # printstring+= "Turn: {}\n".format(msg.front_wheel_angle_rad)
        # printstring+= "Accel: {}\n".format(msg.long_accel_mps2)
        # self.get_logger().info(printstring)
        # commandMarker = Marker()
        # commandMarker.header = self.hack_header
        # commandMarker.header.frame_id = "base_link"
        # commandMarker.type = Marker.ARROW
        # commandMarker.ns = 'vehicle_command_viz'
        # commandMarker.id = 0
        # commandMarker.action = Marker.ADD

        # markerStart = Point()
        # markerStart.x = 0.0
        # markerStart.y = 0.0
        # markerStart.z = 0.0
        # markerEnd = Point()
        # markerEnd.y = 5*msg.front_wheel_angle_rad # math.asin(msg.front_wheel_angle_rad)
        # markerEnd.x = 0.0 #math.acos(msg.front_wheel_angle_rad)
        # markerEnd.z = 0.0
        # # commandMarker.pose.position.x = 0.0
        # # commandMarker.pose.position.y = 0.0
        # # commandMarker.pose.position.z = 0.0
        # # commandMarker.pose.orientation.x = 0.0
        # # commandMarker.pose.orientation.y = 0.0
        # # commandMarker.pose.orientation.z = 0.0
        # # commandMarker.pose.orientation.w = 1.0
        # commandMarker.scale.x = 0.5
        # commandMarker.scale.y = 1.0
        # commandMarker.scale.z = 1.0
        # commandMarker.color.a = 1.0
        # commandMarker.color.r = 0.0
        # commandMarker.color.g = 1.0
        # commandMarker.color.b = 0.0
        # commandMarker.points = [markerStart, markerEnd]
        # self.command_viz_pub_.publish(commandMarker)
    
    # def trajectory_cb(self, msg):
    #     print(len(msg.points))
    #     path = Path()
    #     path.header = msg.header
    #     poseArray = []
    #     for trajPoint in msg.points:
    #         pose = PoseStamped()
    #         pose.pose.position.x = trajPoint.x
    #         pose.pose.position.y = trajPoint.y
    #         poseArray.append(pose)
    #     path.poses = poseArray
    #     self.path_pub_.publish(path)
    #     self.hack_header = msg.header

    def vehicle_command_cb(self, msg):
        desired_speed = 6.0 # mps. How fast we want to go. Manual override.

        svl_command = VehicleControlData()

        # Use bang-bang for accel and brake
        if (self.speed_mps > desired_speed):
            # Slow down
            svl_command.acceleration_pct = 0.0
            svl_command.braking_pct = 0.5
        else:
            # Speed up
            svl_command.acceleration_pct = 0.5
            svl_command.braking_pct = 0.0
        
        svl_command.target_gear = VehicleControlData.GEAR_DRIVE
        svl_command.target_wheel_angle = msg.front_wheel_angle_rad
        svl_command.target_wheel_angular_rate = 3.0 # radians... This is arbitrary... right?
        
        self.vehicle_command_pub.publish(svl_command)

    def svl_can_cb(self, msg):
        self.speed_mps = msg.speed_mps


def main(args=None):
    rclpy.init(args=args)

    bridge = VehicleBridge()

    rclpy.spin(bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
