"""Node-boundary tests for atomic Follower alignment commands."""

from math import nan
from types import SimpleNamespace

import pytest
from follower_alignment_msgs.msg import FollowerAlignmentCommand
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

from follower_approach_control.approach_controller_logic import (
    COARSE_TRACK,
    FINAL_APPROACH,
    BaseControlMeasurement,
    PlanarCommand,
)
from follower_approach_control.approach_controller_node import ApproachControllerNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def command(state=FINAL_APPROACH, mode=FINAL_APPROACH) -> FollowerAlignmentCommand:
    message = FollowerAlignmentCommand()
    message.header.frame_id = "base_link"
    message.header.stamp.sec = 10
    message.target_pose.position.x = 0.1
    message.target_pose.orientation.w = 1.0
    message.control_mode = mode
    message.alignment_state = state
    return message


def harness():
    node = SimpleNamespace(
        _base_frame="base_link",
        _target_tag_id=0,
        _detected=True,
        _selected_tag_id=0,
        _measurement=None,
        _command_stamp_seconds=None,
        _command_received_seconds=None,
        _command_state=None,
        _command_mode=None,
        _pose_timeout=1.2,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)
        ),
    )
    node._pose_is_valid = ApproachControllerNode._pose_is_valid
    node._invalidate_sample = lambda: ApproachControllerNode._invalidate_sample(node)
    return node


def test_twist_populates_only_differential_drive_axes() -> None:
    message = ApproachControllerNode._to_twist(PlanarCommand(0.1, -0.2))
    assert message.linear.x == pytest.approx(0.1)
    assert message.angular.z == pytest.approx(-0.2)
    assert message.linear.y == message.linear.z == 0.0
    assert message.angular.x == message.angular.y == 0.0


def test_pose_validation_accepts_finite_and_rejects_invalid_pose() -> None:
    pose = Pose()
    pose.orientation.w = 1.0
    assert ApproachControllerNode._pose_is_valid(pose)
    pose.position.x = nan
    assert not ApproachControllerNode._pose_is_valid(pose)
    pose.position.x = 0.0
    pose.orientation.w = 0.0
    assert not ApproachControllerNode._pose_is_valid(pose)


def test_atomic_callback_caches_matching_pose_mode_and_state(monkeypatch) -> None:
    node = harness()
    monkeypatch.setattr(
        "follower_approach_control.approach_controller_node.time.monotonic",
        lambda: 20.0,
    )
    ApproachControllerNode._on_command(node, command())
    assert node._measurement == BaseControlMeasurement(0.1, 0.0, 0.0)
    assert node._command_mode == FINAL_APPROACH
    assert node._command_state == FINAL_APPROACH


def test_invalid_atomic_command_clears_all_fields(monkeypatch) -> None:
    node = harness()
    node._measurement = BaseControlMeasurement(1.0, 1.0, 1.0)
    monkeypatch.setattr(
        "follower_approach_control.approach_controller_node.time.monotonic",
        lambda: 20.0,
    )
    invalid = command(state=FINAL_APPROACH, mode=COARSE_TRACK)
    ApproachControllerNode._on_command(node, invalid)
    # Pair compatibility is checked in pure control logic, while all fields
    # still come from this one atomic generation.
    assert node._command_mode == COARSE_TRACK
    assert node._command_state == FINAL_APPROACH
    invalid.header.frame_id = "wrong"
    ApproachControllerNode._on_command(node, invalid)
    assert node._measurement is None
    assert node._command_mode is None
    assert node._command_state is None


def test_detection_and_id_transitions_invalidate_cache() -> None:
    node = harness()
    node._measurement = BaseControlMeasurement(0.1, 0.0, 0.0)
    node._invalidate_sample = lambda: ApproachControllerNode._invalidate_sample(node)
    ApproachControllerNode._on_detected(node, Bool(data=False))
    assert node._measurement is None
    node._detected = True
    node._measurement = BaseControlMeasurement(0.1, 0.0, 0.0)
    ApproachControllerNode._on_tag_id(node, Int32(data=1))
    assert node._measurement is None


def test_enable_transition_discards_cache_and_publishes_zero() -> None:
    node = harness()
    node._measurement = BaseControlMeasurement(0.1, 0.0, 0.0)
    node._raw_pub = Publisher()
    node._enabled_pub = Publisher()
    response = SetBool.Response()
    result = ApproachControllerNode._on_enable(
        node, SetBool.Request(data=True), response
    )
    assert result.success
    assert node._measurement is None
    assert isinstance(node._raw_pub.messages[-1], Twist)
    assert node._raw_pub.messages[-1].linear.x == 0.0
    assert node._enabled_pub.messages[-1].data is True
