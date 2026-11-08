#!/usr/bin/env python3
"""Follow a moving target with the gripper: the template for computer-vision code.

A real vision module replaces `fake_detector()` with its detector, returning a point in the camera
frame (plus `h.tf.to_base(point, 'camera_link')`). Here the "detection" is a point circling in
front of the arm, so it runs in the sim with no camera.

    handy sim
    ros2 run handy_examples track_target                 # 25 s at 15 Hz "camera" rate
    ros2 run handy_examples track_target --radius 0.03 --period 12
"""
import argparse
import math
import time

import numpy as np

from handy_api import Handy


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--radius', type=float, default=0.04, help='target circle radius (m)')
    parser.add_argument('--period', type=float, default=10.0, help='target circle period (s)')
    parser.add_argument('--duration', type=float, default=25.0, help='seconds to track')
    parser.add_argument('--rate', type=float, default=15.0, help='"camera" frame rate (Hz)')
    # Sim, 4 cm / 10 s circle: P only 2.4 cm mean error; + feedforward: gain 2 -> 0.9 cm,
    # 4 -> 0.5 cm, 6 -> 0.4 cm. Start lower (~2) on the real arm, where latency is higher.
    parser.add_argument('--gain', type=float, default=4.0, help='P gain (1/s)')
    parser.add_argument('--no-feedforward', action='store_true',
                        help='P control only (the gripper lags a moving target)')
    args = parser.parse_args()

    with Handy(node_name='track_target') as h:
        h.go_to('wave', speed=1.0)
        center = h.state.eef_position()

        def fake_detector(t):
            """Stand-in for a vision detector: target point in base_link (m)."""
            a = 2 * math.pi * t / args.period
            return center + np.array([0.0, args.radius * math.sin(a),
                                      args.radius * (1 - math.cos(a))])

        h.log.info(f'Tracking a {100 * args.radius:.0f} cm circle for {args.duration:.0f} s')
        errors, start = [], time.monotonic()
        last_target, last_t, target_vel = None, None, np.zeros(3)
        try:
            while (t := time.monotonic() - start) < args.duration:
                target = fake_detector(t)               # vision: h.tf.to_base(p, 'camera_link')
                # Feedforward: estimate the target's velocity from successive detections
                # (smoothed), so the arm moves *with* the target; P control only fixes the error.
                if last_target is not None and not args.no_feedforward:
                    measured = (target - last_target) / (t - last_t)
                    target_vel = 0.7 * target_vel + 0.3 * measured
                last_target, last_t = target, t
                err = target - h.state.eef_position()
                h.servo.set_twist(linear=target_vel + args.gain * err)  # m/s, clamped, kept alive
                if t > 5.0:                             # after the initial catch-up
                    errors.append(np.linalg.norm(err))
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            pass
        finally:
            h.servo.stop()
        if errors:
            mode = 'P only' if args.no_feedforward else 'feedforward + P'
            h.log.info(f'tracking error after 5 s ({mode}): mean {100 * np.mean(errors):.1f} cm, '
                       f'max {100 * np.max(errors):.1f} cm')
        h.go_to('wave', speed=1.0)


if __name__ == '__main__':
    main()
