'''
Package:   guardian
Filename:  guardian_node.py
Author:    Will Heitman (w at heit.mn)

The Guardian node aggregates Navigator's diagnostic data and publishes:
- Global state description ("Stopping at intersection"),
- Mode (Enabled, Manual, Disabled),
- and global status level (OK, WARN, or ERROR).

Subscribes to:
 /node_statuses (diagnostic_msgs/DiagnosticStatus)

Publishes to:
/status (diagnostic_msgs/DiagnosticArray)
'''

import numpy as np
from rosgraph_msgs.msg import Clock
import time

from carla_msgs.msg import CarlaEgoVehicleControl
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from nav_msgs.msg import Path
from navigator_msgs.msg import Mode
import rclpy
from rclpy.node import Node
from dataclasses import dataclass


STALENESS_TOLERANCE = 0.8  # sec. Statuses staler than this will be marked as stale
NOT_RECEIVED = b'\xff'

# DISENGAGES GUARDIAN FOR TESTING. OTHERWISE SET TO TRUE
ENGAGED = True


@dataclass
class StatusEntry:
    values: any
    level: DiagnosticStatus.level = NOT_RECEIVED
    message: str = ""
    stamp: float = -1.


class guardian_node(Node):
    joy_sub = 0.0
    command_pub = None
    current_mode = Mode.DISABLED

    def __init__(self):
        super().__init__('guardian_node')

        self.clock = 0.0
        self.path_receive_time = time.time()

        simulated = self.declare_parameter(
            'simulated', False)

        mode_request_sub = self.create_subscription(
            Mode, '/requested_mode', self.modeRequestCb, 1)

        status_sub = self.create_subscription(
            DiagnosticStatus, '/node_statuses', self.statusCb, 10)

        self.status_array_pub = self.create_publisher(
            DiagnosticArray, '/status', 1)

        self.current_mode_pub = self.create_publisher(
            Mode, '/guardian/mode', 1)

        self.path_sub = self.create_subscription(
            Path, '/planning/path', self.pathCb, 1)

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1)

        self.manual_nodes = {
            "joy_translation_node": StatusEntry([]),
        }

        is_in_simulation = simulated.get_parameter_value().bool_value
        self.get_logger().error(str(simulated.get_parameter_value()))

        if not is_in_simulation:
            self.manual_nodes["epas_node"] = StatusEntry([])
            self.manual_nodes["linear_actuator_node"] = StatusEntry([])

        self.auto_nodes = {
            "rtp_node": StatusEntry([]),
        }

        if not is_in_simulation:
            self.manual_nodes["gnss_interface_node"] = StatusEntry([])
        # else:
        #     self.manual_nodes["gnss_processing_node"] = StatusEntry([])

        self.other_nodes = {
            "recording": StatusEntry([])
        }

        status_timer = self.create_timer(0.2, self.publishStatusArray)

        self.auto_disabled = True
        self.manual_disabled = False

    def pathCb(self, msg: Path):
        """Here we simply note the time that the message was received.
        Used to ensure that the path planner is alive.

        Args:
            msg (Path)
        """
        self.path_receive_time = time.time()

    def clockCb(self, msg: Clock):
        self.clock = msg.clock.sec + msg.clock.nanosec*1e-9

    def initStatusMsg(self, name: str) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        stamp = KeyValue()
        stamp.key = 'stamp'
        stamp.value = str(self.clock)
        status.values.append(stamp)
        return status

    def isStale(self, status: StatusEntry, node_name: str) -> bool:

        if node_name == 'rtp_node':
            return (time.time() - self.path_receive_time) > STALENESS_TOLERANCE

        return (self.clock - status.stamp) > STALENESS_TOLERANCE

    def publishStatusArray(self):
        """The idea here is pretty simple. 
        - Iterate through the manual and auto watchlists.
        - If any entry in the auto watchlist is either empty or 
            "error", disable auto driving.
        - If any entry in the manual watchlist is either empty or
            "error", disable manual AND auto driving.
        - Add all entries from both lists to a DiagnosticStatusArray,
            including a global status that reflects the Guardian's state
        """

        global_status = self.initStatusMsg('global')
        array_msg = DiagnosticArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()

        self.auto_disabled = False
        auto_error_description = ""
        self.manual_disabled = False
        manual_error_description = ""

        for dict_entry in self.manual_nodes:
            status_msg = DiagnosticStatus()
            status_msg.name = dict_entry
            status: StatusEntry = self.manual_nodes[dict_entry]
            # print(value)
            status_msg.level = status.level
            status_msg.values = status.values

            if status.level == DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = status.message
                self.manual_disabled = False
                manual_error_description += status.message + '; '
            elif status.level == NOT_RECEIVED:
                global_status.level = DiagnosticStatus.ERROR

                if dict_entry == "joy_translation_node":
                    global_status.message += f"{dict_entry} not received. Is the joystick connected?"
                else:
                    global_status.message += f"{dict_entry} not received."

                # self.manual_disabled = True
            elif self.isStale(status, dict_entry):
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = f"{dict_entry} was stale."
                self.manual_disabled = False
                manual_error_description += f"{dict_entry} was stale. "
            elif status.level == DiagnosticStatus.WARN and global_status.level != DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.WARN
                global_status.message += status.message + '; '

            status_msg.message = status.message
            array_msg.status.append(status_msg)

        for dict_entry in self.auto_nodes:
            status_msg = DiagnosticStatus()
            status_msg.name = dict_entry
            status: StatusEntry = self.auto_nodes[dict_entry]
            # print(value)
            status_msg.level = status.level
            status_msg.values = status.values

            if status.level == DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = status.message
                self.auto_disabled = True
                manual_error_description += status.message + '; '
            elif status.level == NOT_RECEIVED:
                global_status.level = DiagnosticStatus.ERROR

                if dict_entry == "joy_translation_node":
                    global_status.message += f"{dict_entry} not received. Is the joystick connected?"
                else:
                    global_status.message += f"{dict_entry} not received."

                self.auto_disabled = True
            elif self.isStale(status, dict_entry):
                global_status.level = DiagnosticStatus.ERROR
                global_status.message += f"{dict_entry} was stale. "
                self.auto_disabled = True
                manual_error_description += f"{dict_entry} was stale. "
            elif status.level == DiagnosticStatus.WARN and global_status.level != DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.WARN
                global_status.message += status.message + '; '

            status_msg.message = status.message
            array_msg.status.append(status_msg)

        for dict_entry in self.other_nodes:
            status_msg = DiagnosticStatus()
            status_msg.name = dict_entry
            status: StatusEntry = self.other_nodes[dict_entry]
            # print(value)
            status_msg.level = status.level
            status_msg.values = status.values

            if status.level == DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = status.message
                manual_error_description += status.message + '; '
            elif status.level == NOT_RECEIVED:
                global_status.level = DiagnosticStatus.WARN

                if dict_entry == "joy_translation_node":
                    global_status.message += f"{dict_entry} not received. Is the joystick connected?"
                else:
                    global_status.message += f"{dict_entry} not received."
            elif self.isStale(status, dict_entry):
                global_status.level = DiagnosticStatus.ERROR
                global_status.message = f"{dict_entry} was stale. "
                manual_error_description += f"{dict_entry} was stale. "
            elif status.level == DiagnosticStatus.WARN and global_status.level != DiagnosticStatus.ERROR:
                global_status.level = DiagnosticStatus.WARN
                global_status.message += status.message + '; '

            status_msg.message = status.message
            array_msg.status.append(status_msg)

        capability_kv = KeyValue()
        capability_kv.key = 'capability'
        if self.manual_disabled:
            global_status.message += 'System disabled; '
            capability_kv.value = 'DISABLED'
        elif self.auto_disabled:
            capability_kv.value = 'MANUAL'
            # global_status.message += 'Auto disabled; '
        else:
            capability_kv.value = 'AUTO'

        global_status.values.append(capability_kv)

        array_msg.status.append(global_status)

        self.status_array_pub.publish(array_msg)

    def getStamp(self, msg: DiagnosticStatus) -> float:
        """Extract timestamp (seconds) from values in DiagnosticStatus

        Args:
            msg (DiagnosticStatus): message to parse

        Returns:
            float: timestamp (seconds)
        """
        for val in msg.values:
            val: KeyValue
            if val.key == 'stamp':
                return float(val.value)

        self.get_logger().warning(f"Status {msg.name} was unstamped.")

    def statusCb(self, msg: DiagnosticStatus):
        stamp = self.getStamp(msg)
        if msg.name in self.auto_nodes:
            self.auto_nodes[msg.name] = StatusEntry(msg.values,
                                                    msg.level, msg.message, stamp)
        elif msg.name in self.manual_nodes:
            # print(f"{msg.name} was in MANUAL")
            self.manual_nodes[msg.name] = StatusEntry(msg.values,
                                                      msg.level, msg.message, stamp)
        elif msg.name in self.other_nodes:
            # print(f"{msg.name} was in MANUAL")
            self.other_nodes[msg.name] = StatusEntry(msg.values,
                                                     msg.level, msg.message, stamp)
        else:
            self.get_logger().warning(
                f"Received status from node outside watchlist: {msg.name}")
            return

    def modeRequestCb(self, msg: Mode):
        # Here we can choose to accept or deny the requested mode.

        if self.manual_disabled:
            self.current_mode = Mode.DISABLED
        elif self.auto_disabled and msg.mode == Mode.AUTO:
            self.current_mode = Mode.DISABLED
        else:
            self.current_mode = msg.mode

        # self.current_mode = msg.mode

        mode_msg = Mode()
        mode_msg.mode = self.current_mode
        self.current_mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    guardian = guardian_node()
    rclpy.spin(guardian)
    guardian_node.destroy_node()
    rclpy.shutdown()
