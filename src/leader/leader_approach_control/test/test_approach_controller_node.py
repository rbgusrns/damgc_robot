"""Small ROS-message boundary tests for the Leader approach controller."""

from math import nan
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from leader_approach_control.approach_controller_logic import (
    BaseControlMeasurement,
    PlanarCommand,
)
from leader_approach_control.approach_controller_node import ApproachControllerNode


def make_pose() -> PoseStamped:
    """Create a finite forward-facing base pose."""
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp.sec = 10
    pose.pose.position.x = 0.60
    pose.pose.orientation.w = 1.0
    return pose


def test_twist_populates_only_differential_drive_axes() -> None:
    twist = ApproachControllerNode._to_twist(PlanarCommand(0.04, -0.10))

    assert twist.linear.x == 0.04
    assert twist.angular.z == -0.10
    assert twist.linear.y == 0.0
    assert twist.linear.z == 0.0
    assert twist.angular.x == 0.0
    assert twist.angular.y == 0.0


def test_pose_validation_accepts_finite_planar_target_pose() -> None:
    assert ApproachControllerNode._pose_is_valid(make_pose())


def test_pose_validation_rejects_nonfinite_or_invalid_quaternion() -> None:
    pose = make_pose()
    pose.pose.position.y = nan
    assert not ApproachControllerNode._pose_is_valid(pose)

    pose = make_pose()
    pose.pose.orientation.w = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)


def test_pose_validation_allows_negative_target_for_stop_state() -> None:
    pose = make_pose()
    pose.pose.position.x = -0.01
    assert ApproachControllerNode._pose_is_valid(pose)


def test_mode_is_bound_to_latest_pose_generation() -> None:
    harness = SimpleNamespace(
        _measurement=BaseControlMeasurement(0.1, 0.0, 0.0),
        _latest_pose_generation=7,
        _mode_generation=None,
        _coherent_mode=None,
        _mode_received_seconds=None,
        _invalidate_sample=lambda: None,
    )
    ApproachControllerNode._on_mode(harness, String(data="NEAR_ALIGN"))
    assert harness._mode_generation == 7
    assert harness._coherent_mode == "NEAR_ALIGN"
    assert harness._mode_received_seconds is not None


def test_unknown_mode_invalidates_cached_sample() -> None:
    invalidated = []
    harness = SimpleNamespace(
        _measurement=BaseControlMeasurement(0.1, 0.0, 0.0),
        _latest_pose_generation=1,
        _invalidate_sample=lambda: invalidated.append(True),
    )
    ApproachControllerNode._on_mode(harness, String(data="UNKNOWN"))
    assert invalidated == [True]
