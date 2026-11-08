"""Handy's configuration loads from the installed handy_moveit_config and is self-consistent."""
import pytest

from handy_api import config, poses, safety


@pytest.fixture(scope='module')
def cfg():
    return config.load_handy_config('full')


def test_servo_scales_from_servo_yaml(cfg):
    assert cfg.servo_scale_linear == pytest.approx(0.05)
    assert cfg.servo_scale_rotational == pytest.approx(0.2)
    assert cfg.servo_scale_joint == pytest.approx(0.2)


def test_limits_cover_all_joints(cfg):
    for j in cfg.arm_joints + (cfg.hand_joint,):
        lo, hi = cfg.joint_limits[j]
        assert lo < hi


@pytest.mark.parametrize('name', ['ready', 'wave', 'home'])
def test_named_poses_inside_full_limits(cfg, name):
    safety.check_joint_target(cfg.named_poses[name], cfg.joint_limits)


def test_hand_positions_inside_limits(cfg):
    lo, hi = cfg.joint_limits[cfg.hand_joint]
    assert all(lo <= v <= hi for v in poses.HAND.values())


def test_test_limits_variant():
    test_cfg = config.load_handy_config('test')
    assert test_cfg.joint_limits['joint_1'] == (-0.3, 0.3)
    with pytest.raises(ValueError):
        config.load_handy_config('wide')
