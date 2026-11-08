"""handy_api_selftest: check the API end to end against a running stack (sim or real).

    handy sim                                  # or: handy real test
    ros2 run handy_api handy_api_selftest      # add --speed 0.3 on the real arm

Moves the arm: planned moves to 'wave' and 'home', a short Servo twist, the gripper, mode
switching.
"""
import argparse
import sys
import time

import numpy as np

from handy_api import Handy


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--speed', type=float, default=1.0, help='Servo speed fraction 0-1')
    parser.add_argument('--limits', default='full', choices=['full', 'test'])
    args = parser.parse_args()
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)

    with Handy(limits=args.limits) as h:
        check('connect', True, f'mode {h.modes.current()}, Servo {h.state.servo_status_text()}')

        for pose in ('wave', 'home') if args.limits == 'full' else ('home',):
            r = h.move_to(pose)
            err = max(abs(h.state.joint(j) - v) for j, v in h.config.named_poses[pose].items())
            check(f'move_to({pose!r}) PTP', r.success and err < 0.02,
                  f'{r.message}, {r.duration:.1f} s, max joint error {err:.3f} rad')
        check('mode restored after planning', h.modes.current() == 'servo')

        if args.limits == 'full':
            h.go_to('wave')
            p0 = h.state.eef_position()
            h.servo.set_twist(linear=(0.0, 0.03 * args.speed, 0.0))
            end = time.monotonic() + 2.0
            while time.monotonic() < end:  # a "vision loop" refreshing the command at 15 Hz
                h.servo.set_twist(linear=(0.0, 0.03 * args.speed, 0.0))
                time.sleep(1 / 15)
            h.servo.stop()
            t_stop = time.monotonic()
            p1 = h.state.eef_position()
            time.sleep(0.5)
            p2 = h.state.eef_position()
            d = p1 - p0
            check('servo.set_twist +Y', d[1] > 0.01 and abs(d[1]) > 3 * max(abs(d[0]), abs(d[2])),
                  f'moved {np.round(100 * d, 1)} cm in 2 s')
            drift = np.linalg.norm(p2 - p1)
            check('servo.stop halts the arm', drift < 0.01,
                  f'{100 * drift:.2f} cm after stop ({time.monotonic() - t_stop:.1f} s)')
            h.go_to('wave')

        for name in ('open', 'close', 'neutral'):
            ok = getattr(h.gripper, name)()
            pos = h.state.gripper_position()
            target = h.config.hand_positions[name]
            check(f'gripper.{name}()', ok and abs(pos - target) < 0.02,
                  f'joint_8 {pos:.3f} (target {target})')

    print(f'{sum(results)}/{len(results)} checks passed')
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
