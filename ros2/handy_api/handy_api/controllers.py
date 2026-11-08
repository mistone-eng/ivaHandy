"""Controller modes: MoveIt Servo streaming vs. planned (MoveIt) trajectories.

servo_controller (Servo) and arm_controller (MoveIt plans) claim the same joints, so exactly one is
active. This is what `handy mode servo|plan` does, as an API.
"""
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController

from handy_api.config import RobotConfig
from handy_api.node import HandyNode, wait_future


class Modes:
    SERVO, PLAN = 'servo', 'plan'

    def __init__(self, hn: HandyNode, config: RobotConfig):
        self._hn, self._cfg = hn, config
        cm = config.controller_manager
        self._switch = hn.node.create_client(SwitchController, f'{cm}/switch_controller',
                                             callback_group=hn.cb_group)
        self._list = hn.node.create_client(ListControllers, f'{cm}/list_controllers',
                                           callback_group=hn.cb_group)

    def controller_states(self, timeout: float = 5.0) -> dict:
        if not self._list.wait_for_service(timeout_sec=timeout):
            raise TimeoutError('controller_manager not available')
        res = wait_future(self._list.call_async(ListControllers.Request()), timeout,
                          'list_controllers')
        return {c.name: c.state for c in res.controller}

    def current(self) -> str:
        states = self.controller_states()
        if states.get(self._cfg.servo_controller) == 'active':
            return self.SERVO
        if states.get(self._cfg.plan_controller) == 'active':
            return self.PLAN
        return 'none'

    def set(self, mode: str, timeout: float = 5.0):
        if mode not in (self.SERVO, self.PLAN):
            raise ValueError(f"mode must be 'servo' or 'plan', got {mode!r}")
        if self.current() == mode:
            return
        on, off = ((self._cfg.servo_controller, self._cfg.plan_controller) if mode == self.SERVO
                   else (self._cfg.plan_controller, self._cfg.servo_controller))
        req = SwitchController.Request(activate_controllers=[on], deactivate_controllers=[off],
                                       strictness=SwitchController.Request.STRICT,
                                       activate_asap=True, timeout=Duration(sec=int(timeout)))
        if not self._switch.wait_for_service(timeout_sec=timeout):
            raise TimeoutError('controller_manager not available')
        res = wait_future(self._switch.call_async(req), timeout + 1.0, 'switch_controller')
        if not res.ok:
            raise RuntimeError(f'switching to {mode} mode failed ({on} on, {off} off)')

    def servo(self):
        self.set(self.SERVO)

    def plan(self):
        self.set(self.PLAN)
