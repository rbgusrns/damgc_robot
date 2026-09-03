"""Small ROS-message boundary tests for the Leader approach controller."""

from math import nan
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from leader_alignment_msgs.msg import LeaderAlignmentCommand

from leader_approach_control.approach_controller_logic import (
    BaseControlMeasurement,
    PlanarCommand,
)
from leader_approach_control.approach_controller_node import ApproachControllerNode


def make_pose() -> Pose:
    """Create a finite forward-facing base pose."""
    pose = Pose()
    pose.position.x = 0.60
    pose.orientation.w = 1.0
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
    pose.position.y = nan
    assert not ApproachControllerNode._pose_is_valid(pose)

    pose = make_pose()
    pose.orientation.w = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)


def test_pose_validation_allows_negative_target_for_stop_state() -> None:
    pose = make_pose()
    pose.position.x = -0.01
    assert ApproachControllerNode._pose_is_valid(pose)


def make_command(mode="NEAR_ALIGN", state="APPROACH") -> LeaderAlignmentCommand:
    command = LeaderAlignmentCommand()
    command.header.frame_id = "base_link"
    command.header.stamp.sec = 10
    command.target_pose = make_pose()
    command.control_mode = mode
    command.alignment_state = state
    return command


def test_command_contains_coherent_pose_mode_and_state() -> None:
    command = make_command()
    assert command.control_mode == "NEAR_ALIGN"
    assert command.alignment_state == "APPROACH"
    assert command.target_pose.position.x == 0.60


def test_invalid_command_mode_is_rejected_by_contract() -> None:
    invalidated = []
    harness = SimpleNamespace(
        _detected=True,
        _selected_tag_id=0,
        _target_tag_id=0,
        _base_frame="base_link",
        _pose_timeout=1.0,
        _measurement=None,
        _invalidate_sample=lambda: invalidated.append(True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
    )
    ApproachControllerNode._on_command(harness, make_command(mode="UNKNOWN"))
    assert invalidated == [True]
