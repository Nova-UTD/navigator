"""
Main ROS 2 node for navigator_lane_change (PR 1 — shadow mode).

Runs at a fixed rate (default 10 Hz).
All behavior logic lives in the imported modules; this node only handles
ROS plumbing: subscriptions, parameters, timer, and publishers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path

from .scene_builder import SceneBuilder
from .need_detector import NeedDetector
from .target_lane_selector import TargetLaneSelector
from .gap_selector import GapSelector
from .safety_checker import SafetyChecker
from .lane_change_state_machine import LaneChangeStateMachine
from .diagnostics import Diagnostics


class LaneChangeNode(Node):
    def __init__(self):
        super().__init__("lane_change_node")

        # ----------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------
        self.declare_parameter("loop_rate_hz", 10.0)
        self.declare_parameter("shadow_mode", True)

        # Topics
        self.declare_parameter("odom_topic", "/gnss/odometry")
        self.declare_parameter("path_topic", "/planning/path")
        self.declare_parameter("objects_topic", "/objdet3d_tracked")
        self.declare_parameter("intersection_topic", "/intersection_status")

        # Need detection
        self.declare_parameter("lookahead_distance_m", 35.0)
        self.declare_parameter("blocked_path_distance_m", 20.0)
        self.declare_parameter("stopped_object_speed_mps", 0.5)
        self.declare_parameter("min_blockage_duration_s", 1.0)

        # Lane selection
        self.declare_parameter("lane_width_m", 3.5)
        self.declare_parameter("allow_left_lane_change", True)
        self.declare_parameter("allow_right_lane_change", True)
        self.declare_parameter("min_target_lane_confidence", 0.6)

        # Gap selection
        self.declare_parameter("min_front_gap_m", 12.0)
        self.declare_parameter("min_rear_gap_m", 10.0)
        self.declare_parameter("min_rear_ttc_s", 4.0)
        self.declare_parameter("min_side_clearance_m", 2.0)

        # Safety / execution
        self.declare_parameter("max_lane_change_speed_mps", 8.0)
        self.declare_parameter("stale_input_timeout_s", 0.5)

        # State machine timing
        self.declare_parameter("wait_for_gap_timeout_s", 8.0)
        self.declare_parameter("abort_cooldown_s", 3.0)
        self.declare_parameter("complete_stabilization_time_s", 1.0)

        p = self.get_parameter

        self._shadow_mode: bool = p("shadow_mode").value
        rate_hz: float = p("loop_rate_hz").value

        # ----------------------------------------------------------------
        # Components
        # ----------------------------------------------------------------
        self._scene = SceneBuilder(
            stale_timeout_s=p("stale_input_timeout_s").value
        )
        self._need = NeedDetector(
            lookahead_distance_m=p("lookahead_distance_m").value,
            blocked_path_distance_m=p("blocked_path_distance_m").value,
            stopped_object_speed_mps=p("stopped_object_speed_mps").value,
            min_blockage_duration_s=p("min_blockage_duration_s").value,
        )
        self._target = TargetLaneSelector(
            lane_width_m=p("lane_width_m").value,
            allow_left=p("allow_left_lane_change").value,
            allow_right=p("allow_right_lane_change").value,
            min_confidence=p("min_target_lane_confidence").value,
        )
        self._gap = GapSelector(
            min_front_gap_m=p("min_front_gap_m").value,
            min_rear_gap_m=p("min_rear_gap_m").value,
            min_rear_ttc_s=p("min_rear_ttc_s").value,
            min_side_clearance_m=p("min_side_clearance_m").value,
        )
        self._safety = SafetyChecker(
            stale_input_timeout_s=p("stale_input_timeout_s").value,
            max_lane_change_speed_mps=p("max_lane_change_speed_mps").value,
        )
        self._sm = LaneChangeStateMachine(
            wait_for_gap_timeout_s=p("wait_for_gap_timeout_s").value,
            abort_cooldown_s=p("abort_cooldown_s").value,
            complete_stabilization_time_s=p("complete_stabilization_time_s").value,
        )
        self._diag = Diagnostics()

        # ----------------------------------------------------------------
        # Subscriptions
        # ----------------------------------------------------------------
        self.create_subscription(
            Odometry, p("odom_topic").value, self._cb_odom, 10
        )
        self.create_subscription(
            Path, p("path_topic").value, self._cb_path, 10
        )
        self.create_subscription(
            String, p("intersection_topic").value, self._cb_intersection, 10
        )
        self._subscribe_objects(p("objects_topic").value)

        # ----------------------------------------------------------------
        # Publishers (shadow-mode outputs only)
        # ----------------------------------------------------------------
        self._pub_state = self.create_publisher(
            String, "/behavior/lane_change_state", 10
        )
        self._pub_decision = self.create_publisher(
            String, "/behavior/lane_change_decision", 10
        )
        self._pub_debug = self.create_publisher(
            String, "/behavior/lane_change_debug", 10
        )

        # ----------------------------------------------------------------
        # Main loop timer
        # ----------------------------------------------------------------
        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f"LaneChangeNode ready — shadow_mode={self._shadow_mode}, "
            f"rate={rate_hz:.0f} Hz"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _subscribe_objects(self, topic: str) -> None:
        try:
            from navigator_msgs.msg import Object3DArray  # type: ignore
            self.create_subscription(
                Object3DArray, topic, self._cb_objects, 10
            )
            self.get_logger().info(f"Objects: {topic} (Object3DArray)")
        except ImportError:
            self.get_logger().warn(
                "navigator_msgs not available — object subscription skipped. "
                "Gap and need detection will run without object data."
            )

    def _cb_odom(self, msg) -> None:
        self._scene.update_odom(msg)

    def _cb_path(self, msg) -> None:
        self._scene.update_path(msg)

    def _cb_objects(self, msg) -> None:
        self._scene.update_objects(msg)

    def _cb_intersection(self, msg) -> None:
        self._scene.update_intersection(msg)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        scene = self._scene.build()
        need = self._need.detect(scene)
        target = self._target.select(scene, need.suggested_direction)
        gap = self._gap.assess(scene, target)
        safety = self._safety.check(scene, gap)
        state = self._sm.update(scene, need, target, gap, safety)

        prev = self._sm.prev_state
        reason = self._sm.transition_reason

        # Log every state transition
        if prev != state:
            self.get_logger().info(
                f"[LC] {prev.name} → {state.name}  ({reason})"
            )

        # Publish state
        state_msg = String()
        state_msg.data = state.name
        self._pub_state.publish(state_msg)

        # Publish compact decision summary
        decision_msg = String()
        decision_msg.data = Diagnostics.to_json(
            self._diag.decision_summary(state, need, target, gap, safety)
        )
        self._pub_decision.publish(decision_msg)

        # Publish full debug dump every cycle
        debug_msg = String()
        debug_msg.data = Diagnostics.to_json(
            self._diag.build(
                state=state,
                prev_state=prev,
                transition_reason=reason,
                scene=scene,
                need=need,
                target=target,
                gap=gap,
                safety=safety,
                shadow_mode=self._shadow_mode,
            )
        )
        self._pub_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneChangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
