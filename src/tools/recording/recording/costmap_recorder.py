"""
Package:   recording
Filename:  costmap_recorder.py
Author:    Bennett Daniel

Subscribes to:
/grid/steering_cost (nav_msgs/OccupancyGrid)
/gnss/odometry (nav_msgs/Odometry)
/clock (rosgraph_msgs/Clock)

Records costmap data and saves it as images in a costmaps directory.
"""

from datetime import datetime
import os
import numpy as np
import rclpy
import cv2

# Add necessary imports for parameters and services
from rclpy.parameter import Parameter
from rclpy.node import Node  # Keep only one import
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import SetBool  # Import the service type

# Message definitions
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock

# Larger = fewer writes, but they take longer.
FRAME_RATE = 1  # FPS. Messages will be recorded at this rate.


class costmap_recorder(Node):

    def __init__(self):
        # Use a consistent name for the node and status reporting
        super().__init__("costmap_recorder")

        # Declare the parameter for the output directory
        self.declare_parameter(
            "output_directory", "/navigator/costmaps"
        )  # Default to costmaps subdirectory

        # Get the parameter value
        output_directory_param = self.get_parameter("output_directory")
        self.costmaps_directory = (
            output_directory_param.get_parameter_value().string_value
        )
        self.get_logger().info(f"Using output directory: {self.costmaps_directory}")

        # Objects to store data in memory
        self.current_time = 0.0
        self.current_costmap_msg = None
        self.current_odom_msg = None
        self.total_disk_usage = 0  # Bytes
        self.image_counter = 0  # Counter for sequential image naming

        # Recording starts disabled by default
        self.recording_enabled = True
        # self.get_logger().info(
        #     "Recording is initially DISABLED. Use the '~/toggle_recording' service to enable."
        # )

        self.setUpDirectory()

        # Create the service to toggle recording
        self.toggle_recording_service = self.create_service(
            SetBool, "~/toggle_recording", self.handle_toggle_recording_request
        )

        odom_sub = self.create_subscription(Odometry, "/gnss/odometry", self.odomCb, 10)

        clock_sub = self.create_subscription(Clock, "/clock", self.clockCb, 1)

        costmap_sub = self.create_subscription(
            OccupancyGrid, "/grid/steering_cost", self.costmapCb, 1
        )

        append_timer = self.create_timer(
            (1.0 / FRAME_RATE),
            self.recordCostmap,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.status_pub = self.create_publisher(DiagnosticStatus, "/node_statuses", 1)

    def getStatus(self):
        msg = DiagnosticStatus()
        msg.name = "recording"
        msg.level = DiagnosticStatus.OK

        stamp = KeyValue()
        stamp.key = "stamp"
        stamp.value = str(self.current_time)
        msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = "state"
        if self.recording_enabled:
            state_kv.value = "recording"
        else:
            state_kv.value = "idle"

        msg.values.append(state_kv)

        disk_usage_kv = KeyValue()
        disk_usage_kv.key = "disk_usage"
        disk_usage_kv.value = f"{self.total_disk_usage}"
        msg.values.append(disk_usage_kv)

        return msg

    def setUpDirectory(self):
        """Set up the costmaps directory if it doesn't exist."""
        # Directory path is now set in __init__ from the parameter
        # Create the directory if it doesn't exist, with error logging
        self.get_logger().info(f"Create directory initiated: {self.costmaps_directory}")
        try:
            if not os.path.exists(self.costmaps_directory):
                # Use os.makedirs to create parent directories if needed
                os.makedirs(self.costmaps_directory)
                self.get_logger().info(f"Created directory: {self.costmaps_directory}")
            else:
                self.get_logger().info(
                    f"Using existing directory: {self.costmaps_directory}"
                )
        except OSError as e:
            # Log error if directory creation fails (e.g., permissions)
            self.get_logger().error(
                f"Failed to create directory {self.costmaps_directory}: {e}"
            )
            # Consider how to handle this - maybe prevent recording?
            # For now, recordCostmap will check if the dir exists.

    # Add the service callback function
    def handle_toggle_recording_request(self, request, response):
        """Callback for the SetBool service to enable/disable recording."""
        self.recording_enabled = request.data
        if self.recording_enabled:
            self.get_logger().info("Recording ENABLED via service call.")
            response.message = "Recording enabled"
        else:
            self.get_logger().info("Recording DISABLED via service call.")
            response.message = "Recording disabled"
        response.success = True
        return response

    def recordCostmap(self):
        """
        Record the latest costmap and save it as an image.
        """
        # Check if recording is enabled first
        self.get_logger().warning("record costmap initiated")
        if not self.recording_enabled:
            # Optionally publish status even when idle, or just return silently
            # status_msg = self.getStatus()
            # self.status_pub.publish(status_msg)
            return  # Return early if not enabled

        # Add a check for the directory existence in case setUpDirectory failed
        if not os.path.isdir(self.costmaps_directory):
            self.get_logger().error(
                f"Output directory {self.costmaps_directory} does not exist. Cannot save costmap."
            )
            # Publish status indicating error?
            return

        if self.current_costmap_msg is None:
            self.get_logger().warning("Costmap not received")
            return

        if self.current_odom_msg is None:
            self.get_logger().warning("Odom not received")
            return

        # Process costmap and save as image
        self.saveCostmapAsImage()

        # Publish status
        status_msg = self.getStatus()
        self.status_pub.publish(status_msg)

    def saveCostmapAsImage(self):
        """Convert costmap to image and save it with sequential numbering"""
        # Convert OccupancyGrid data to numpy array
        costmap_array = np.asarray(
            self.current_costmap_msg.data, dtype=np.int8
        ).reshape(
            self.current_costmap_msg.info.height, self.current_costmap_msg.info.width
        )

        # Normalize to 0-255 for image
        # -1 in occupancy grid means unknown, map to 128 (gray)
        # 0-100 in occupancy grid means probability of occupancy, map to 0-255
        image_array = np.zeros_like(costmap_array, dtype=np.uint8)
        image_array[costmap_array == -1] = 128
        image_array[costmap_array >= 0] = costmap_array[costmap_array >= 0] * 255 // 100

        # Create filename with sequential numbering
        filename = f"{self.costmaps_directory}/costmap_{self.image_counter:06d}.png"

        # Save as image
        try:
            cv2.imwrite(filename, image_array)
            # Log successful save (already present, but good to confirm)
            self.get_logger().info(f"Saved costmap to {filename}")

            # Update disk usage
            self.total_disk_usage += os.path.getsize(filename)

            # Increment counter for next image
            self.image_counter += 1
        except cv2.error as e:
            self.get_logger().error(f"Failed to save costmap image {filename}: {e}")
        except OSError as e:
            self.get_logger().error(f"File system error saving {filename}: {e}")
        except e:
            self.get_logger().error((e))

    def costmapCb(self, msg: OccupancyGrid):
        """Caches the latest costmap message

        Args:
            msg (OccupancyGrid)
        """
        # Add debug logging when receiving a costmap
        self.get_logger().debug(
            f"Received costmap message with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )
        self.current_costmap_msg = msg

    def clockCb(self, msg: Clock):
        """Caches the latest message, to be used by recordCostmap()

        Args:
            msg (Clock)
        """
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odomCb(self, msg: Odometry):
        """Caches the latest message, to be used by recordCostmap()

        Args:
            msg (Odometry)
        """
        self.current_odom_msg = msg

    def close(self):
        status_msg = DiagnosticStatus()
        status_msg.name = "recording"
        status_msg.message = f"Recording stopped"
        status_msg.level = DiagnosticStatus.OK

        stamp = KeyValue()
        stamp.key = "stamp"
        stamp.value = str(0.0)
        status_msg.values.append(stamp)

        state_kv = KeyValue()
        state_kv.key = "state"
        state_kv.value = f"idle"
        status_msg.values.append(state_kv)

        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = costmap_recorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    finally:
        # Clean up before closing
        node.close()
    rclpy.shutdown()