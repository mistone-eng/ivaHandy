"""API-side safety checks (pure). Servo and ros2_control enforce their own limits as well."""
from typing import Dict, Optional

import numpy as np

from handy_api.config import Limits, Workspace


def clamp_norm(v, max_norm):
    """Scale vector v down so that |v| <= max_norm (direction kept)."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v if n <= max_norm or n == 0 else v * (max_norm / n)


def clamp_joint_velocities(vels: Dict[str, float], max_speed: float) -> Dict[str, float]:
    return {j: float(np.clip(v, -max_speed, max_speed)) for j, v in vels.items()}


def check_joint_target(target: Dict[str, float], limits: Limits, margin: float = 0.0) -> None:
    """Raise ValueError if a joint target is unknown or outside its limits (minus margin)."""
    for joint, value in target.items():
        if joint not in limits:
            raise ValueError(f'unknown joint {joint!r}')
        lo, hi = limits[joint]
        if not (lo + margin <= value <= hi - margin):
            raise ValueError(f'{joint} target {value:.3f} outside limits [{lo:.3f}, {hi:.3f}]')


def limit_to_workspace(position, linear, workspace: Optional[Workspace]):
    """Zero velocity components that would push the end effector further outside the box."""
    linear = np.array(linear, dtype=float)
    if workspace is None or position is None:
        return linear
    for i in range(3):
        if position[i] <= workspace.min_xyz[i] and linear[i] < 0:
            linear[i] = 0.0
        if position[i] >= workspace.max_xyz[i] and linear[i] > 0:
            linear[i] = 0.0
    return linear
