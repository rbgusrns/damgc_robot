"""Small ROS-message boundary tests for the Leader approach controller."""

from math import nan

from geometry_msgs.msg import PoseStamped

from leader_approach_control.approach_controller_logic import PlanarCommand
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


def test_pose_validation_accepts_finite_forward_pose() -> None:
    assert ApproachControllerNode._pose_is_valid(make_pose())


def test_pose_validation_rejects_nonfinite_or_nonforward_pose() -> None:
    pose = make_pose()
    pose.pose.position.y = nan
    assert not ApproachControllerNode._pose_is_valid(pose)

    pose = make_pose()
    pose.pose.position.x = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)

    pose = make_pose()
    pose.pose.orientation.w = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)
