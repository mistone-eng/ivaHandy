"""The named poses used by the examples must be reachable within Handy's configured limits."""
import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from handy_examples.poses import ARM_JOINTS, HAND, HAND_JOINT, READY, SERVO_STATUS, WAVE


def _limits(file_name):
    path = os.path.join(
        get_package_share_directory('handy_moveit_config'), 'config', file_name)
    with open(path) as f:
        return yaml.safe_load(f)['joint_limits']


def _inside(limits, joint, value):
    lim = limits[joint]
    return lim['min_position'] <= value <= lim['max_position']


def test_ready_covers_the_arm_joints():
    assert sorted(READY) == sorted(ARM_JOINTS)


@pytest.mark.parametrize('joint', ARM_JOINTS)
def test_ready_inside_full_limits(joint):
    assert _inside(_limits('joint_limits.yaml'), joint, READY[joint])


def test_ready_needs_full_limits():
    """servo_demo documents that READY lies outside the +/-0.3 test window."""
    test = _limits('joint_limits_test.yaml')
    assert any(not _inside(test, j, READY[j]) for j in ARM_JOINTS)


@pytest.mark.parametrize('joint', ARM_JOINTS)
def test_wave_inside_full_limits(joint):
    assert sorted(WAVE) == sorted(ARM_JOINTS)
    assert _inside(_limits('joint_limits.yaml'), joint, WAVE[joint])


@pytest.mark.parametrize('name', sorted(HAND))
def test_hand_poses_inside_full_limits(name):
    assert _inside(_limits('joint_limits.yaml'), HAND_JOINT, HAND[name])


def test_servo_status_codes():
    assert SERVO_STATUS[0] == 'OK'
    assert set(SERVO_STATUS) == {0, 1, 2, 3, 4, 5}
