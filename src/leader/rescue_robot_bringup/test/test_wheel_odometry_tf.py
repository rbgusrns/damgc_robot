"""Wheel TF preserves measured pose; VSLAM is not required to advance it."""

import importlib.util
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location(
    "wheel_odometry_tf", Path(__file__).resolve().parents[1] / "scripts/wheel_odometry_tf.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_pose_frame_timestamp_and_rotation_are_preserved():
    m = Odometry()
    m.header.frame_id = "odom"
    m.child_frame_id = "base_link"
    m.header.stamp.sec = 123
    m.pose.pose.position.x = 1.03
    m.pose.pose.position.y = 0.29
    m.pose.pose.orientation.w = 1.0
    t = module.wheel_transform(m)
    assert t.header == m.header
    assert t.child_frame_id == m.child_frame_id
    assert t.transform.translation.x == 1.03
    assert t.transform.translation.y == 0.29
    assert t.transform.rotation == m.pose.pose.orientation
    m.header.frame_id = "map"
    with pytest.raises(ValueError):
        module.wheel_transform(m)


def test_invalid_pose_is_not_broadcast():
    m = Odometry()
    m.header.frame_id = "odom"
    m.child_frame_id = "base_link"
    m.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError):
        module.wheel_transform(m)
    m.pose.pose.orientation.w = 1.0
    m.pose.pose.position.x = float("nan")
    with pytest.raises(ValueError):
        module.wheel_transform(m)
