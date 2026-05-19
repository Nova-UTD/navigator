"""
Converts raw ROS topic data into a normalized Scene for behavior logic.
All behavior modules consume Scene, not raw ROS messages.
"""

import math
import time
from typing import Optional

from .types import (
    Scene, EgoPose, TrackedObject, TopicHealth,
)


class SceneBuilder:
    def __init__(self, stale_timeout_s: float = 0.5):
        self._stale = stale_timeout_s

        self._odom = None
        self._path = None
        self._objects = None
        self._intersection = None

        self._t_odom = 0.0
        self._t_path = 0.0
        self._t_objects = 0.0
        self._t_intersection = 0.0

        self._commanded_direction = "none"
        self._t_command = 0.0
        self._command_timeout_s = 30.0

    # ------------------------------------------------------------------
    # ROS callback handlers — store latest message + receipt timestamp
    # ------------------------------------------------------------------

    def update_odom(self, msg) -> None:
        self._odom = msg
        self._t_odom = time.time()

    def update_path(self, msg) -> None:
        self._path = msg
        self._t_path = time.time()

    def update_objects(self, msg) -> None:
        self._objects = msg
        self._t_objects = time.time()

    def update_intersection(self, msg) -> None:
        self._intersection = msg
        self._t_intersection = time.time()

    def update_command(self, direction: str) -> None:
        """Store an operator or planner lane-change command. Expires after command_timeout_s."""
        self._commanded_direction = direction
        self._t_command = time.time()

    # ------------------------------------------------------------------
    # Build Scene
    # ------------------------------------------------------------------

    def build(self) -> Scene:
        now = time.time()

        # Command expires after timeout to prevent stale commands driving behaviour.
        commanded = self._commanded_direction
        if commanded != "none" and (now - self._t_command) > self._command_timeout_s:
            commanded = "none"
            self._commanded_direction = "none"

        health = TopicHealth(
            odom_fresh=(now - self._t_odom) < self._stale,
            path_fresh=(now - self._t_path) < self._stale,
            # Objects and intersection can arrive slower — allow 4× timeout
            objects_fresh=(now - self._t_objects) < self._stale * 4,
            intersection_fresh=(now - self._t_intersection) < self._stale * 4,
        )

        ego_pose: Optional[EgoPose] = None
        ego_speed = 0.0
        ego_heading = 0.0
        if health.odom_fresh and self._odom is not None:
            ego_pose, ego_speed, ego_heading = self._parse_odom(self._odom)

        path = []
        if health.path_fresh and self._path is not None:
            path = self._parse_path(self._path)

        objects = []
        if self._objects is not None:
            objects = self._parse_objects(self._objects)

        intersection_stop = False
        if self._intersection is not None and health.intersection_fresh:
            intersection_stop = self._parse_intersection(self._intersection)

        return Scene(
            stamp=now,
            ego_pose=ego_pose,
            ego_speed=ego_speed,
            ego_heading=ego_heading,
            current_path=path,
            nearby_objects=objects,
            intersection_stop=intersection_stop,
            topic_health=health,
            commanded_direction=commanded,
        )

    # ------------------------------------------------------------------
    # Parsers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_odom(msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear

        yaw = SceneBuilder._quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
        speed = math.sqrt(lin.x ** 2 + lin.y ** 2)
        pose = EgoPose(x=pos.x, y=pos.y, z=pos.z, yaw=yaw)
        return pose, speed, yaw

    @staticmethod
    def _parse_path(msg):
        pts = []
        for ps in msg.poses:
            p = ps.pose.position
            pts.append((p.x, p.y, p.z))
        return pts

    @staticmethod
    def _parse_objects(msg):
        objs = []
        try:
            for obj in msg.objects:
                try:
                    pos = obj.pose.position
                    vel = getattr(obj, "velocity", None)
                    dims = getattr(obj, "dimensions", None)

                    vx = vel.linear.x if vel and hasattr(vel, "linear") else (vel.x if vel else 0.0)
                    vy = vel.linear.y if vel and hasattr(vel, "linear") else (vel.y if vel else 0.0)

                    length = dims.x if dims else 4.5
                    width = dims.y if dims else 2.0
                    height = dims.z if dims else 1.5

                    obj_id = str(getattr(obj, "id", "?"))
                    cls = str(getattr(obj, "classification", "unknown"))
                    conf = float(getattr(obj, "confidence", 1.0))

                    objs.append(TrackedObject(
                        object_id=obj_id,
                        classification=cls,
                        x=pos.x, y=pos.y, z=pos.z,
                        vx=vx, vy=vy,
                        length=length, width=width, height=height,
                        confidence=conf,
                    ))
                except Exception:
                    continue
        except Exception:
            pass
        return objs

    @staticmethod
    def _parse_intersection(msg) -> bool:
        # navigator_msgs/IntersectionStatus: status field — non-zero means stop required
        try:
            status = getattr(msg, "status", 0)
            return int(status) > 0
        except Exception:
            return False

    @staticmethod
    def _quat_to_yaw(x, y, z, w) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)
