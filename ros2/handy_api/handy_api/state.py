"""Robot state: joints, end-effector pose (TF) and MoveIt Servo status."""
import threading
from typing import Dict, Optional

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformListener

from handy_api.config import RobotConfig
from handy_api.geometry import Pose
from handy_api.node import HandyNode, wait_until


class State:
    def __init__(self, hn: HandyNode, config: RobotConfig):
        self._hn, self._cfg = hn, config
        self._lock = threading.Lock()
        self._joints: Dict[str, float] = {}
        self._velocities: Dict[str, float] = {}
        self._servo_status: Optional[int] = None
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, hn.node, spin_thread=False)
        hn.node.create_subscription(JointState, config.joint_states_topic, self._on_joints, 10,
                                    callback_group=hn.cb_group)
        hn.node.create_subscription(Int8, config.servo_status_topic, self._on_status, 10,
                                    callback_group=hn.cb_group)

    def _on_joints(self, msg: JointState):
        with self._lock:
            self._joints.update(zip(msg.name, msg.position))
            if msg.velocity:
                self._velocities.update(zip(msg.name, msg.velocity))

    def _on_status(self, msg: Int8):
        self._servo_status = msg.data

    # ---- joints ----------------------------------------------------------------
    def joints(self) -> Dict[str, float]:
        """Latest position of every joint (rad)."""
        with self._lock:
            return dict(self._joints)

    def arm_joints(self) -> Dict[str, float]:
        j = self.joints()
        return {name: j[name] for name in self._cfg.arm_joints if name in j}

    def joint(self, name: str) -> float:
        return self.joints()[name]

    def gripper_position(self) -> float:
        return self.joint(self._cfg.hand_joint)

    def wait_for_joints(self, timeout: float = 10.0):
        needed = set(self._cfg.arm_joints) | {self._cfg.hand_joint}
        wait_until(lambda: needed <= set(self.joints()), timeout, 'joint states')

    # ---- end effector -------------------------------------------------------------
    def eef_pose(self, frame: Optional[str] = None, timeout: float = 1.0) -> Pose:
        """End-effector pose (default in the base frame), from TF."""
        frame = frame or self._cfg.base_frame
        t = self.tf_buffer.lookup_transform(frame, self._cfg.eef_frame, Time(),
                                            timeout=Duration(seconds=timeout))
        tr, rot = t.transform.translation, t.transform.rotation
        return Pose(np.array([tr.x, tr.y, tr.z]), np.array([rot.x, rot.y, rot.z, rot.w]), frame)

    def eef_position(self, frame: Optional[str] = None) -> np.ndarray:
        return self.eef_pose(frame).position

    # ---- servo ---------------------------------------------------------------------
    def servo_status(self) -> Optional[int]:
        """Latest /servo_node/status code (None until Servo has published)."""
        return self._servo_status

    def servo_status_text(self) -> str:
        s = self._servo_status
        return 'unknown' if s is None else self._cfg.status_names.get(s, str(s))
