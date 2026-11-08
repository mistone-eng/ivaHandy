"""Planned motion through MoveIt (Pilz PTP / LIN): named pose, joint goal or Cartesian pose.

Switches to plan mode (arm_controller) for the move and back to the previous mode afterwards.
Pilz needs an explicit planner id (PTP or LIN); RViz's "<unspecified>" does not work with it.
"""
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Union

from geometry_msgs.msg import Pose as PoseMsg
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

from handy_api import safety
from handy_api.config import RobotConfig
from handy_api.geometry import Pose
from handy_api.node import HandyNode, wait_future

Target = Union[str, Dict[str, float], Pose]


@dataclass
class MotionResult:
    success: bool
    error_code: int
    duration: float
    message: str = ''


class MotionHandle:
    """Returned by move_to(..., wait=False)."""

    def __init__(self, fn):
        self._result: Optional[MotionResult] = None
        self._error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, args=(fn,), daemon=True)
        self._thread.start()

    def _run(self, fn):
        try:
            self._result = fn()
        except BaseException as e:  # re-raised in wait()
            self._error = e

    def done(self) -> bool:
        return not self._thread.is_alive()

    def wait(self, timeout: Optional[float] = None) -> MotionResult:
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise TimeoutError('motion still running')
        if self._error:
            raise self._error
        return self._result


_CODES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


class Planner:
    def __init__(self, hn: HandyNode, config: RobotConfig, modes):
        self._hn, self._cfg, self._modes = hn, config, modes
        self._client = ActionClient(hn.node, MoveGroup, config.move_group_action,
                                    callback_group=hn.cb_group)

    def resolve(self, target: Target):
        """Named pose -> joints; joints -> limit-checked; Pose -> Pose (must be in base frame)."""
        if isinstance(target, str):
            if target not in self._cfg.named_poses:
                known = sorted(self._cfg.named_poses)
                raise ValueError(f'unknown pose {target!r}; known: {known}')
            target = self._cfg.named_poses[target]
        if isinstance(target, dict):
            safety.check_joint_target(target, self._cfg.joint_limits)
            return dict(target)
        if isinstance(target, Pose):
            if target.frame != self._cfg.base_frame:
                raise ValueError(f'Pose must be in {self._cfg.base_frame!r} '
                                 '(use Transforms.transform_pose first)')
            return target
        raise TypeError(f'unsupported target type {type(target).__name__}')

    def _goal(self, target, planner: str, velocity: float, acceleration: float):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self._cfg.planning_group
        r.pipeline_id, r.planner_id = self._cfg.planning_pipeline, planner
        r.num_planning_attempts, r.allowed_planning_time = 1, 5.0
        r.max_velocity_scaling_factor = velocity
        r.max_acceleration_scaling_factor = acceleration
        r.start_state.is_diff = True
        c = Constraints()
        if isinstance(target, dict):
            c.joint_constraints = [JointConstraint(joint_name=j, position=float(v),
                                                   tolerance_above=0.01, tolerance_below=0.01,
                                                   weight=1.0) for j, v in target.items()]
        else:
            pc = PositionConstraint(link_name=self._cfg.eef_frame, weight=1.0)
            pc.header.frame_id = self._cfg.base_frame
            center = PoseMsg()
            center.position.x, center.position.y, center.position.z = (
                float(v) for v in target.position)
            center.orientation.w = 1.0
            pc.constraint_region = BoundingVolume(
                primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])],
                primitive_poses=[center])
            oc = OrientationConstraint(link_name=self._cfg.eef_frame, weight=1.0,
                                       absolute_x_axis_tolerance=0.05,
                                       absolute_y_axis_tolerance=0.05,
                                       absolute_z_axis_tolerance=0.05)
            oc.header.frame_id = self._cfg.base_frame
            (oc.orientation.x, oc.orientation.y, oc.orientation.z,
             oc.orientation.w) = (float(v) for v in target.orientation)
            c.position_constraints, c.orientation_constraints = [pc], [oc]
        r.goal_constraints = [c]
        goal.planning_options.plan_only = False
        return goal

    def move_to(self, target: Target, planner: str = 'PTP', velocity: float = 0.2,
                acceleration: float = 0.2, timeout: float = 60.0,
                wait: bool = True) -> Union[MotionResult, MotionHandle]:
        """Plan and execute a move to `target` (named pose, {joint: rad} or Pose in base frame).

        planner: 'PTP' (joint-space, fastest) or 'LIN' (straight line, Cartesian targets).
        velocity / acceleration: scaling factors 0-1 of the joint limits.
        """
        resolved = self.resolve(target)
        goal = self._goal(resolved, planner, velocity, acceleration)

        def run() -> MotionResult:
            if not self._client.wait_for_server(timeout_sec=10.0):
                raise TimeoutError(f'{self._cfg.move_group_action} not available')
            previous = self._modes.current()
            self._modes.plan()
            start = time.monotonic()
            try:
                handle = wait_future(self._client.send_goal_async(goal), 10.0, 'MoveGroup goal')
                if not handle.accepted:
                    return MotionResult(False, MoveItErrorCodes.FAILURE, 0.0, 'goal rejected')
                res = wait_future(handle.get_result_async(), timeout, 'MoveGroup result')
                code = res.result.error_code.val
                return MotionResult(code == MoveItErrorCodes.SUCCESS, code,
                                    time.monotonic() - start, _CODES.get(code, str(code)))
            finally:
                if previous == self._modes.SERVO:
                    self._modes.servo()

        return run() if wait else MotionHandle(run)
