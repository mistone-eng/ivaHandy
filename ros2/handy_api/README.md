# handy_api

A modular Python API for the Handy arm, made to be called from computer vision and other Python
modules. You write plain Python: no nodes, no spinning, no message types. `Handy()` runs its own
ROS node in a background thread.

```python
from handy_api import Handy

with Handy() as h:                          # connects (joint states + Servo), servo mode
    h.move_to('wave')                       # planned Pilz PTP move
    h.gripper.open()
    print(h.state.eef_position())           # [x, y, z] m in base_link
    h.play('figure8')                       # a built-in Servo pattern
```

Needs the Handy stack running: `handy sim` (mock) or `handy real test`. Default limits are
`full`; use `Handy(limits='test')` when the stack runs with `limits:=test`.

## Modules

Every part is importable on its own. The ROS-free ones never import rclpy.

| Module | ROS | What it does |
|---|---|---|
| `config` | no | `RobotConfig`: joints, frames, topics, controllers, Servo scales, limits, named poses. `load_handy_config()` reads Handy's from `handy_moveit_config`. Another arm = another config. |
| `poses` | no | `READY`, `WAVE`, `HOME`, `HAND`, `SERVO_STATUS` |
| `geometry` | no | `Pose`, quaternion helpers, `transform_point` |
| `safety` | no | velocity clamping, joint-limit checks, workspace box |
| `patterns` | no | motion patterns as functions of time: `figure8`, `helix`, `snake`, `hello`, `custom` |
| `node` | yes | `HandyNode`: one node + background executor thread |
| `state` | yes | joints, gripper, end-effector pose (TF), Servo status |
| `transforms` | yes | `to_base(point, frame)`, `transform_pose`, `lookup`: perception frames → robot |
| `controllers` | yes | `Modes`: `servo()` / `plan()` (which controller drives the arm) |
| `servo` | yes | `ServoClient`: velocity streaming with keep-alive, `go_to` (joint move), `stream` |
| `motion` | yes | `Planner`: `move_to(named / joints / Pose)` with Pilz PTP or LIN |
| `gripper` | yes | `Gripper`: `open/close/neutral/set/follow`, waits for completion |
| `facade` | yes | `Handy`: all of the above, plus `go_to`, `play`, `stop`, context manager |

## API reference

**`Handy(config=None, limits='full', node_name='handy_api', connect=True, timeout=15.0)`**
- `h.move_to(target, planner='PTP', velocity=0.2, acceleration=0.2, wait=True)` → `MotionResult(success, error_code, duration, message)`
  - **target:** `'wave' | 'home' | 'ready'`, `{joint: rad}`, or `Pose(position, orientation)` in `base_link`
  - **planner:** `'LIN'` gives a straight line (Cartesian targets)
  - **`wait=False`:** returns a handle with `.done()` / `.wait()`
  - **modes:** switches to plan mode for the move and back afterwards
- `h.go_to(target, speed=0.5)`: joint move through Servo (no planning), closed loop. Returns `True` when reached.
- `h.play(pattern, speed=1.0)`: run `'figure8' | 'helix' | 'snake' | 'hello'` or a `Pattern`.
- `h.stop()`, `h.close()` (or use `with`).

**`h.servo`**: Servo velocity streaming, for vision loops.
- `set_twist(linear=(vx, vy, vz), angular=(wx, wy, wz), frame='base_link')`
  - units: m/s and rad/s, clamped to Servo's scale (`servo.yaml`: 0.05 m/s, 0.2 rad/s)
  - `frame='link_eef'` moves in the gripper's own frame
- `set_joint_velocity({joint: rad/s})`, clamped to 0.2 rad/s.
- **Keep-alive:** the last command is republished at 50 Hz. Refresh it from your loop at any rate
  faster than 2 Hz. If you stop refreshing (your loop hung or crashed), it is dropped after 0.5 s
  and Servo stops the arm.
- `stop()`, `active()`, `go_to(...)`, `stream(sample_fn, duration, kind)`.
- Speeds are capped by `servo.yaml`, and Humble Servo delivers about 57% of the commanded velocity
  (measured), so close the loop on what you observe rather than on time.

**`h.state`**: `joints()`, `arm_joints()`, `joint(name)`, `gripper_position()`,
`eef_pose(frame=None)` → `Pose`, `eef_position()`, `servo_status()` (0 OK; 1/2 singularity
slow/halt; 3/4 collision; 5 joint limit), `servo_status_text()`.

**`h.tf`**: `to_base(point, source_frame)`, `transform_point(point, source, target=None)`,
`transform_pose(pose, target=None)`, `lookup(target, source)`.

**`h.gripper`**: `open()`, `close()`, `neutral()`, `set(position, duration=1.0, wait=True)`,
`follow([(t, position), ...])` → `True` when the trajectory finished.

**`h.modes`**: `current()` → `'servo' | 'plan' | 'none'`, `servo()`, `plan()`, `controller_states()`.

## Computer vision integration

A detector gives you points in its camera frame. Publish where the camera is, once:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0 --z 0.8 --roll 0 --pitch 1.57 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
```

Then servo toward what you see:

```python
import numpy as np
from handy_api import Handy, Workspace, load_handy_config

cfg = load_handy_config()
# optional safety box for the gripper (m, base_link): never go below the table, etc.
cfg = cfg.__class__(**{**cfg.__dict__, 'workspace': Workspace((0.1, -0.4, 0.15), (0.6, 0.4, 0.9))})

with Handy(config=cfg) as h:
    h.move_to('wave')
    while True:
        p_cam = detector.find_object()              # your code: (x, y, z) in camera_link, or None
        if p_cam is None:
            h.servo.stop()
            continue
        target = h.tf.to_base(p_cam, 'camera_link')
        err = target - h.state.eef_position()
        if np.linalg.norm(err) < 0.01:
            h.servo.stop()
            break
        h.servo.set_twist(linear=2.0 * err)         # P-control; clamped to 0.05 m/s
    h.gripper.close()
```

For **moving** targets, add velocity feedforward: estimate the target's velocity from successive
detections and add it to the P term. Measured in the sim (4 cm circle, 10 s period, 15 Hz):

| Control | Mean tracking error |
|---|---|
| P only (gain 1.5) | 2.4 cm |
| feedforward + P (gain 2 / 4 / 6) | 0.9 / 0.5 / 0.4 cm |

Start with lower gains on the real arm (more latency).

`testing/handy_examples/handy_examples/track_target.py` is a runnable version with a simulated
moving target in place of the detector.

## Checking it works

```bash
handy sim                                   # in one terminal
ros2 run handy_api handy_api_selftest       # in another: moves the arm, prints PASS/FAIL
colcon test --packages-select handy_api     # unit tests (no robot needed) + lint
```
