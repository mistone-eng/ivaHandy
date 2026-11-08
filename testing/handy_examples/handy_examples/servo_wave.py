#!/usr/bin/env python3
"""Sine-wave MoveIt Servo demo for the Handy arm, built on handy_api.

Patterns (handy_api.patterns; each starts and ends at the WAVE pose, arm reaching forward):
  figure8  end effector traces a Lissajous figure-8 in the Y/Z plane (Cartesian twist)
  helix    circle in Y/Z while drifting in and out along X (Cartesian twist)
  snake    joints 4-7 oscillate with phase offsets: a wave travels down the arm (joint jog)
  hello    wrist roll waves while the wrist nods, and the gripper "breathes" open/closed

Run with the stack up (needs limits:=full, WAVE is outside the +/-0.3 test window):
    handy sim
    ros2 run handy_examples servo_wave                        # all patterns
    ros2 run handy_examples servo_wave --only figure8 snake   # a subset
    ros2 run handy_examples servo_wave --speed 0.5            # slower (use <= 0.5 on the real arm)

Ctrl+C stops immediately; Servo halts ~0.1 s after commands stop.
"""
import argparse

from handy_api import Handy

PATTERNS = ['figure8', 'helix', 'snake', 'hello']


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--speed', type=float, default=1.0,
                        help="0-1 fraction of Servo's scale "
                             '(default 1.0; use <= 0.5 on the real arm)')
    parser.add_argument('--only', nargs='+', choices=PATTERNS, default=PATTERNS,
                        help='run only these patterns, in this order')
    args = parser.parse_args()
    speed = max(0.0, min(1.0, args.speed))
    with Handy(node_name='servo_wave') as h:
        try:
            for name in args.only:
                if not h.play(name, speed=speed):
                    return
            h.log.info('Wave demo complete')
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
