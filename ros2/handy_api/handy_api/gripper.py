"""Gripper through hand_controller's FollowJointTrajectory action (waits for completion)."""
from typing import List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from handy_api.config import RobotConfig
from handy_api.node import HandyNode, wait_future


def _duration(t: float) -> Duration:
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)


class Gripper:
    def __init__(self, hn: HandyNode, config: RobotConfig):
        self._hn, self._cfg = hn, config
        self._client = ActionClient(hn.node, FollowJointTrajectory, config.hand_action,
                                    callback_group=hn.cb_group)
        lo, hi = config.joint_limits.get(config.hand_joint, (-float('inf'), float('inf')))
        self._limits = (lo, hi)

    def _clamp(self, pos: float) -> float:
        return min(max(pos, self._limits[0]), self._limits[1])

    def follow(self, waypoints: Sequence[Tuple[float, float]], wait: bool = True,
               timeout: Optional[float] = None) -> bool:
        """Send (time_from_start, position) waypoints as one trajectory."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError(f'{self._cfg.hand_action} not available')
        traj = JointTrajectory(joint_names=[self._cfg.hand_joint])
        traj.points = [JointTrajectoryPoint(positions=[self._clamp(p)],
                                            time_from_start=_duration(t))
                       for t, p in waypoints]
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        handle = wait_future(self._client.send_goal_async(goal), 5.0, 'gripper goal')
        if not handle.accepted:
            raise RuntimeError('gripper goal rejected')
        if not wait:
            return True
        span = max(t for t, _ in waypoints)
        result = wait_future(handle.get_result_async(), timeout or span + 5.0, 'gripper motion')
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def set(self, position: float, duration: float = 1.0, wait: bool = True) -> bool:
        """Move the gripper joint to `position` (rad) in `duration` s."""
        return self.follow([(duration, position)], wait)

    def open(self, duration: float = 1.0, wait: bool = True) -> bool:
        return self.set(self._cfg.hand_positions['open'], duration, wait)

    def close(self, duration: float = 1.0, wait: bool = True) -> bool:
        return self.set(self._cfg.hand_positions['close'], duration, wait)

    def neutral(self, duration: float = 1.0, wait: bool = True) -> bool:
        return self.set(self._cfg.hand_positions['neutral'], duration, wait)

    def named(self) -> List[str]:
        return list(self._cfg.hand_positions)
