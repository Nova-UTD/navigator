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

import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarProcessingNode(Node):

    def __init__(self):
        super().__init__('lidar_processing_node')
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_cb, 10)

    def lidar_cb(self, msg: PointCloud2):
        pcd_array: np.array = rnp.numpify(msg)
        print(pcd_array)


def main(args=None):
    rclpy.init(args=args)

    lidar_processor = LidarProcessingNode()

    rclpy.spin(lidar_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
