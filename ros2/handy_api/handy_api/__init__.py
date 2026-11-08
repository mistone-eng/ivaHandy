"""handy_api: a modular Python API for the Handy arm.

ROS 2 Humble: MoveIt Servo, MoveIt (Pilz), ros2_control.

Most scripts only need `Handy`; every part is importable on its own:
  ROS-free:  config, poses, geometry, safety, patterns
  ROS:       node, state, transforms, controllers, servo, motion, gripper, facade

Exports are loaded lazily, so `import handy_api.patterns` (or geometry, ...) does not pull in
rclpy.
"""
import importlib

_EXPORTS = {
    'Handy': 'handy_api.facade',
    'Pose': 'handy_api.geometry',
    'RobotConfig': 'handy_api.config',
    'Workspace': 'handy_api.config',
    'load_handy_config': 'handy_api.config',
    'MotionResult': 'handy_api.motion',
}

__all__ = list(_EXPORTS)


def __getattr__(name):
    if name in _EXPORTS:
        return getattr(importlib.import_module(_EXPORTS[name]), name)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
