#!/usr/bin/env python3
"""Hands-free MoveIt Servo demo for the Handy arm, built on handy_api.

Sequence:
  1. Joint-space: go to a bent "ready" pose (handy_api.poses.READY)
  2. Cartesian: trace a square with the end effector (Y/Z plane)
  3. Cartesian: rotate the wrist back and forth about Z
  4. Joint-space: return to the ready pose

Run with the stack up:
    handy sim                                       # limits:=full
    ros2 run handy_examples servo_demo              # full demo
    ros2 run handy_examples servo_demo --speed 0.3  # slower (recommended on the real arm)

Requires limits:=full (the ready pose is outside the +/-0.3 test window).
Ctrl+C stops immediately; Servo halts ~0.1 s after commands stop.
"""
import argparse

from handy_api import Handy

SQUARE = [((0, +1, 0), 'square 1/4 (+Y)'), ((0, 0, +1), 'square 2/4 (+Z)'),
          ((0, -1, 0), 'square 3/4 (-Y)'), ((0, 0, -1), 'square 4/4 (-Z)')]
WRIST = [((0, 0, +1), 1.5, 'wrist rotate +Z'), ((0, 0, -1), 3.0, 'wrist rotate -Z'),
         ((0, 0, +1), 1.5, 'wrist rotate back')]


def leg(h, linear=(0, 0, 0), angular=(0, 0, 0), seconds=2.0, label='', speed=0.5):
    """One constant unitless twist for `seconds`, then let Servo settle."""
    h.log.info(f'Cartesian: {label}')
    h.servo.stream(lambda t: (linear, angular), seconds, 'twist', speed)
    if h.state.servo_status():
        h.log.info(f'Servo status: {h.state.servo_status_text()}')


def run(h, speed):
    h.log.info('Joint jog -> ready pose')
    if not h.go_to('ready', speed=speed):
        h.log.warn('Ready pose not reached (check Servo status / limits)')
        return
    h.log.info('Reached ready pose')
    for lin, label in SQUARE:
        leg(h, linear=lin, seconds=2.0, label=label, speed=speed)
    for ang, seconds, label in WRIST:
        leg(h, angular=ang, seconds=seconds, label=label, speed=speed)
    h.log.info('Joint jog -> ready pose')
    h.go_to('ready', speed=speed)
    h.log.info('Demo complete')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0.5,
                        help="0-1 fraction of Servo's scale (use 0.3 on the real arm)")
    args = parser.parse_args()
    with Handy(node_name='servo_demo') as h:
        try:
            run(h, max(0.0, min(1.0, args.speed)))
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
