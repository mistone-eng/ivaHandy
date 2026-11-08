"""Small pose / quaternion helpers (pure numpy; quaternions are (x, y, z, w) like ROS)."""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Pose:
    position: np.ndarray                                 # (3,) meters
    orientation: np.ndarray = field(  # (4,) quaternion x, y, z, w
        default_factory=lambda: np.array([0., 0., 0., 1.]))
    frame: str = 'base_link'

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.orientation = normalize(np.asarray(self.orientation, dtype=float).reshape(4))


def normalize(q):
    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError('zero quaternion')
    return q / n


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[:3]


def transform_point(translation, rotation, point):
    """Apply a transform (translation, quaternion) to a point: R * p + t."""
    rotated = rotate(np.asarray(rotation, float), np.asarray(point, float))
    return rotated + np.asarray(translation, float)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy,
                     cr * cp * cy + sr * sp * sy])
