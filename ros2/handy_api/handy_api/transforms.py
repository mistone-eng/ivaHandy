"""Frame transforms for perception: e.g. a point detected in a camera frame -> robot base frame.

Uses the TF tree, so any frame published there works (add a static transform for your camera:
`ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera_link
...`).
"""
from typing import Optional

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time

from handy_api.config import RobotConfig
from handy_api.geometry import Pose, quat_multiply, transform_point


class Transforms:
    def __init__(self, tf_buffer, config: RobotConfig):
        self._buf, self._cfg = tf_buffer, config

    def lookup(self, target: str, source: str, timeout: float = 1.0):
        """(translation (3,), rotation quaternion (4,)) mapping `source` points into `target`."""
        t = self._buf.lookup_transform(target, source, Time(), timeout=Duration(seconds=timeout))
        tr, rot = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([rot.x, rot.y, rot.z, rot.w])

    def transform_point(self, point, source: str, target: Optional[str] = None) -> np.ndarray:
        translation, rotation = self.lookup(target or self._cfg.base_frame, source)
        return transform_point(translation, rotation, point)

    def to_base(self, point, source: str) -> np.ndarray:
        """Point (3,) given in frame `source`, expressed in the robot base frame."""
        return self.transform_point(point, source, self._cfg.base_frame)

    def transform_pose(self, pose: Pose, target: Optional[str] = None) -> Pose:
        target = target or self._cfg.base_frame
        if pose.frame == target:
            return pose
        translation, rotation = self.lookup(target, pose.frame)
        return Pose(transform_point(translation, rotation, pose.position),
                    quat_multiply(rotation, pose.orientation), target)
