'''
Package:   guardian
Filename:  guardian_node.py
Author:    Pranav Boyapati

The Guardian node aggregates Navigator's diagnostic data.
'''

import numpy as np
from rosgraph_msgs.msg import Clock
import time
from navigator_msgs.msg import VehicleControl
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from nav_msgs.msg import Path
from navigator_msgs.msg import Mode
import rclpy
from rclpy.node import Node
from dataclasses import dataclass
from std_msgs.msg import String

@dataclass
class StatusData:
    node_name: str
    status: str
    timestamp: float

class guardian_node(Node):
    def __init__(self):
        super().__init__('guardian_node')

        self.clock = 0.0
        self.STALENESS_TOLERANCE = 0.8
        self.auto_disabled = False
        self.manual_disabled = False
        self.current_mode = None

        status_sub = self.create_subscription(String, '/node_statuses', self.statusCb, 10)
        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)
        mode_request_sub = self.create_subscription(Mode, '/requested_mode', self.modeRequestCb, 1)

        self.status_array_pub = self.create_publisher(DiagnosticArray, '/status', 1)
        self.current_mode_pub = self.create_publisher(Mode, '/guardian/mode', 1)

        self.mandatory_nodes = {
            "map_manager": StatusData(),
            "joy_translation": StatusData(),
            "gnss_averager": StatusData(),
            "mcl": StatusData(),
            "routing_monitor": StatusData(),
            "grid_route_costmap": StatusData(),
            "grid_summation": StatusData(),
            "intersection_manager": StatusData(),
            "junction_manager": StatusData(),
            "path_planner": StatusData(),
            "path_planner_nav2": StatusData(),
            "pure_pursuit_controller": StatusData(),
            "airbags": StatusData(),
            "semantic_projection": StatusData(),
            "ground_seg": StatusData(),
            "static_grid": StatusData(),
            "traffic_light_detector": StatusData(),
            "prednet_inference": StatusData(),
            "driveable_area": StatusData(),
            "road_signs_classifier": StatusData(),
            "depth_processing": StatusData(),
            "occupancy_grid_node": StatusData(),
            "image_segmentation": StatusData(),
            "lane_type_detector": StatusData(),
        }
        self.secondary_nodes = {
            "costmap_recorder": StatusData(),
            "object_viz_deteced_node": StatusData(),
            "multi_object_tracker_3d_node": StatusData(),
            "object_viz_tracked_node": StatusData(),
            "pedestrian_intent_to_enter_road": StatusData(),
            "pedestrian_skeleton": StatusData(),
            "road_user_detector": StatusData(),
        }

        status_timer = self.create_timer(0.2, self.publishStatusArray)


    def clockCb(self, msg: Clock):
        self.clock = msg.clock.sec + msg.clock.nanosec*1e-9

    
    def statusCb(self, msg: String):
        data = msg.data.split(', ')
        if len(data) != 3:
            self.get_logger().error(f"Invalid status message: {msg.data}")
            return
        
        node_name, status, timestamp_str = data
        timestamp = float(timestamp_str)

        if node_name in self.mandatory_nodes:
            self.mandatory_nodes[node_name] = StatusData(node_name, status, timestamp)
        elif node_name in self.secondary_nodes:
            self.secondary_nodes[node_name] = StatusData(node_name, status, timestamp)
        else:
            self.get_logger().warn(f"Unknown node name: {node_name}")

    
    def modeRequestCb(self, msg: Mode):
        if self.manual_disabled:
            self.current_mode = Mode.DISABLED
        elif self.auto_disabled:
            self.current_mode = Mode.MANUAL
        else:
            self.current_mode = msg.mode

        mode_msg = Mode()
        mode_msg.mode = self.current_mode
        self.current_mode_pub.publish(mode_msg)

    
    def isStale(self, status: StatusData):
        return (self.clock - status.timestamp) > self.STALENESS_TOLERANCE


    def initStatusMsg(self, name: str):
        status = DiagnosticStatus()
        status.name = name
        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock)
        status.values.append(stamp)
        return status


    def publishStatusArray(self):
        global_status = self.initStatusMsg('global')
        array_msg = DiagnosticArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()

        for node_name, status_data in self.mandatory_nodes.items():
            status_msg = DiagnosticStatus()
            status_msg.name = node_name

            if self.isStale(status_data):
                self.auto_disabled = True
                self.manual_disabled = True
                status_msg.level = DiagnosticStatus.STALE
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = f"{node_name} was stale."
                status_msg.message = f"{node_name} was stale."
            elif status_data == StatusData():
                self.auto_disabled = True
                self.manual_disabled = True
                status_msg.level = DiagnosticStatus.ERROR
                global_status.level = DiagnosticStatus.ERROR
                global_status.message += f"{node_name} not received."
                status_msg.message = f"{node_name} not received."
            else:
                if status_data.status == 'OK' and not self.auto_disabled and not self.manual_disabled:
                    self.auto_disabled = False
                    self.manual_disabled = False
                    status_msg.level = DiagnosticStatus.OK
                    status_msg.message = f"{node_name} is OK."
                elif status_data.status == 'OK' and (self.auto_disabled or self.manual_disabled):
                    status_msg.level = DiagnosticStatus.OK
                    status_msg.message = f"{node_name} is OK."
                elif status_data.status == 'ERROR':
                    self.auto_disabled = True
                    self.manual_disabled = True
                    status_msg.level = DiagnosticStatus.ERROR
                    global_status.level = DiagnosticStatus.ERROR
                    global_status.message += f"{node_name} reported ERROR."
                    status_msg.message = f"{node_name} reported ERROR."

            array_msg.status.append(status_msg)
                
        for node_name, status_data in self.secondary_nodes.items():
            status_msg = DiagnosticStatus()
            status_msg.name = node_name

            if self.isStale(status_data):
                self.manual_disabled = True
                status_msg.level = DiagnosticStatus.STALE
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = f"{node_name} was stale."
                status_msg.message = f"{node_name} was stale."
            elif status_data == StatusData():
                self.manual_disabled = True
                status_msg.level = DiagnosticStatus.ERROR
                global_status.level = DiagnosticStatus.ERROR
                global_status.message += f"{node_name} not received."
                status_msg.message = f"{node_name} not received."
            else:
                if status_data.status == 'OK' and not self.manual_disabled:
                    self.manual_disabled = False
                    status_msg.level = DiagnosticStatus.OK
                    status_msg.message = f"{node_name} is OK."
                elif status_data.status == 'OK' and self.manual_disabled:
                    status_msg.level = DiagnosticStatus.OK
                    status_msg.message = f"{node_name} is OK."
                elif status_data.status == 'ERROR':
                    self.manual_disabled = True
                    status_msg.level = DiagnosticStatus.ERROR
                    global_status.level = DiagnosticStatus.ERROR
                    global_status.message += f"{node_name} reported ERROR."
                    status_msg.message = f"{node_name} reported ERROR."

            array_msg.status.append(status_msg)

        capability_kv = KeyValue()
        capability_kv.key = 'capability'
        if self.manual_disabled:
            capability_kv.value = 'DISABLED'
        elif self.auto_disabled:
            capability_kv.value = 'MANUAL'
        else:
            capability_kv.value = 'AUTO'

        global_status.values.append(capability_kv)
        array_msg.status.append(global_status)
        self.status_array_pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    guardian = guardian_node()
    rclpy.spin(guardian)
    guardian_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()