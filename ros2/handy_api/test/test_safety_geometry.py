"""Pure safety and geometry helpers."""
import math

import numpy as np
import pytest

from handy_api import geometry, safety
from handy_api.config import Workspace


def test_clamp_norm_keeps_direction():
    v = safety.clamp_norm([0.3, 0.4, 0.0], 0.05)
    assert np.linalg.norm(v) == pytest.approx(0.05)
    assert v[0] / v[1] == pytest.approx(0.75)
    assert np.allclose(safety.clamp_norm([0.01, 0, 0], 0.05), [0.01, 0, 0])


def test_joint_target_limits():
    limits = {'joint_1': (-1.0, 1.0)}
    safety.check_joint_target({'joint_1': 0.5}, limits)
    with pytest.raises(ValueError):
        safety.check_joint_target({'joint_1': 1.5}, limits)
    with pytest.raises(ValueError):
        safety.check_joint_target({'joint_9': 0.0}, limits)


def test_workspace_blocks_only_outward_motion():
    box = Workspace((0.0, -0.5, 0.2), (0.6, 0.5, 1.0))
    v = safety.limit_to_workspace([0.3, 0.0, 0.2], [0.01, 0.0, -0.02], box)
    assert v[2] == 0.0 and v[0] == 0.01          # can't go lower, can still move in x
    v = safety.limit_to_workspace([0.3, 0.0, 0.2], [0.0, 0.0, 0.02], box)
    assert v[2] == 0.02                          # moving back inside is allowed
    assert np.allclose(safety.limit_to_workspace([9, 9, 9], [1, 1, 1], None), [1, 1, 1])


def test_quaternion_rotation_and_transform():
    q = geometry.quat_from_rpy(0, 0, math.pi / 2)
    assert np.allclose(geometry.rotate(q, [1, 0, 0]), [0, 1, 0])
    p = geometry.transform_point([1, 2, 3], q, [1, 0, 0])
    assert np.allclose(p, [1, 3, 3])
    pose = geometry.Pose([0, 0, 1])
    assert np.allclose(pose.orientation, [0, 0, 0, 1])
    assert pose.orientation is not geometry.Pose([0, 0, 0]).orientation  # no shared default
