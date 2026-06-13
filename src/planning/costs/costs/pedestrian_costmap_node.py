"""
Package: costs
   File: pedestrian_costmap_node.py
 Author: Shrey Joshi

Publishes /grid/pedestrian — a pedestrian-intent cost layer. SHADOW MODE:
this grid is observable in RViz but is NOT registered in grid_summation_node,
so the planned path is unaffected. Subscribes /pedestrians
(navigator_msgs/PedestrianInfoDetections); all math is delegated to the pure
module pedestrian_costmap.py.

Grid contract (must match grid_summation_node / path_planner_node):
  frame_id   = base_link
  size       = 151 x 151 cells, resolution 0.4 m/cell
  origin     = (x=-20.0, y=-30.0) in base_link  (this exact order: #494 regression)
  width/height never transposed (#493 regression; square here but stated so)
  data       = arr.flatten().tolist(), row-major, clipped 0-100
  stamped from /clock; timer-driven publish; keeps only the newest /pedestrians
"""

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from navigator_msgs.msg import PedestrianInfoDetections

from costs.pedestrian_costmap import (
    distance_to_cost, pose_to_grid_coords, paint_disk,
)

# ── Grid constants (match the other cost layers) ───────────────────────────────
GRID_SIZE  = 151
RESOLUTION = 0.4      # m/cell
ORIGIN_X   = -20.0    # m in base_link (longitudinal)
ORIGIN_Y   = -30.0    # m in base_link (lateral)
FRAME_ID   = 'base_link'


class PedestrianCostmapNode(Node):

    def __init__(self):
        super().__init__('pedestrian_costmap_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('d_max_m', 10.0)
        self.declare_parameter('inflation_radius_m', 0.8)

        self._d_max     = float(self.get_parameter('d_max_m').value)
        self._inflation = float(self.get_parameter('inflation_radius_m').value)

        # ── Internal state ────────────────────────────────────────────────────
        self._sim_clock = Clock()   # avoids shadowing rclpy.Node._clock
        self._latest = None         # newest PedestrianInfoDetections

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Clock, '/clock', self._cb_clock, 1)
        self.create_subscription(
            PedestrianInfoDetections, '/pedestrians', self._cb_pedestrians, 1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(OccupancyGrid, '/grid/pedestrian', 1)

        # ── Publish timer ─────────────────────────────────────────────────────
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info('PedestrianCostmapNode ready (shadow mode).')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_clock(self, msg: Clock) -> None:
        self._sim_clock = msg

    def _cb_pedestrians(self, msg: PedestrianInfoDetections) -> None:
        self._latest = msg

    # ── Costmap painting ──────────────────────────────────────────────────────

    def _paint(self) -> np.ndarray:
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        if self._latest is None:
            return np.clip(grid, 0, 100).astype(np.int8)

        for ped in self._latest.pedestrians:
            cost = distance_to_cost(ped.distance, self._d_max)
            if cost <= 0:
                continue
            rc = pose_to_grid_coords(
                ped.pos_x, ped.pos_y, ORIGIN_X, ORIGIN_Y, RESOLUTION, GRID_SIZE)
            if rc is None:
                self.get_logger().debug(
                    f'pedestrian off-grid: pos=({ped.pos_x:.1f}, {ped.pos_y:.1f})')
                continue
            row, col = rc
            paint_disk(grid, row, col, self._inflation, RESOLUTION, cost)

        return np.clip(grid, 0, 100).astype(np.int8)

    # ── Timer callback ────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        arr = self._paint()

        msg = OccupancyGrid()
        msg.header.stamp           = self._sim_clock.clock
        msg.header.frame_id        = FRAME_ID
        msg.info.resolution        = RESOLUTION
        msg.info.width             = GRID_SIZE   # cols — never transposed (#493)
        msg.info.height            = GRID_SIZE   # rows
        msg.info.origin.position.x = ORIGIN_X    # this order — #494 regression
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.data = arr.flatten().tolist()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
