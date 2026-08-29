"""Tests for validated Follower base-frame conversion and node behavior."""

from math import atan2, inf, nan, pi, sqrt
from types import SimpleNamespace
from typing import Tuple

import pytest
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformException

import follower_supply_perception.base_pose as base_pose_module
from follower_supply_perception.approach_logic import (
    ApproachState,
    TagObservation,
    compute_measurement,
)
from follower_supply_perception.apriltag_approach_node import AprilTagApproachNode
from follower_supply_perception.base_alignment_logic import (
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
)
from follower_supply_perception.base_pose import (
    compute_base_metrics,
    is_fresh_timestamp,
    transform_pose_preserving_stamp,
)


CAMERA_FRAME = "follower/follower_camera_optical_frame"


def make_pose(
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> PoseStamped:
    """Create a deterministic filtered camera pose."""
    pose = PoseStamped()
    pose.header.frame_id = CAMERA_FRAME
    pose.header.stamp.sec = 12
    pose.header.stamp.nanosec = 345
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = quaternion
    return pose


def identity_transform() -> TransformStamped:
    """Create an identity base-from-camera transform."""
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = CAMERA_FRAME
    transform.transform.rotation.w = 1.0
    return transform


def test_base_metrics_use_x_forward_y_lateral_and_atan2() -> None:
    left = compute_base_metrics(1.0, 0.2)
    right = compute_base_metrics(1.0, -0.2)
    assert left.forward_distance == pytest.approx(1.0)
    assert left.lateral_error == pytest.approx(0.2)
    assert left.bearing == pytest.approx(atan2(0.2, 1.0))
    assert left.bearing > 0.0
    assert right.lateral_error < 0.0
    assert right.bearing < 0.0
    assert compute_base_metrics(1.0, 1.0).bearing == pytest.approx(pi / 4.0)


@pytest.mark.parametrize(("x", "y"), [(nan, 0.0), (0.0, inf), (-inf, 0.0)])
def test_base_metrics_reject_non_finite_values(x: float, y: float) -> None:
    with pytest.raises(ValueError):
        compute_base_metrics(x, y)


def test_timestamp_freshness_boundaries_and_invalid_values() -> None:
    now = 12_500_000_000
    assert is_fresh_timestamp(12, 0, now, 0.5)
    assert not is_fresh_timestamp(11, 999_999_999, now, 0.5)
    assert not is_fresh_timestamp(0, 0, now, 1.0)
    assert not is_fresh_timestamp(13, 0, now, 1.0)
    assert not is_fresh_timestamp(12, 1_000_000_000, now, 1.0)
    assert not is_fresh_timestamp(12, 0, now, -1.0)


def test_known_transform_preserves_stamp_and_sets_base_frame() -> None:
    pose = make_pose(position=(1.0, 0.0, 0.0))
    transform = identity_transform()
    transform.header.stamp.sec = 99
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    transform.transform.rotation.z = sqrt(0.5)
    transform.transform.rotation.w = sqrt(0.5)

    result = transform_pose_preserving_stamp(pose, transform, "base_link")

    assert result is not None
    assert result.header.frame_id == "base_link"
    assert result.header.stamp == pose.header.stamp
    assert result.header.stamp != transform.header.stamp
    assert result.pose.position.x == pytest.approx(1.0)
    assert result.pose.position.y == pytest.approx(3.0)
    assert result.pose.orientation.z == pytest.approx(sqrt(0.5))
    assert result.pose.orientation.w == pytest.approx(sqrt(0.5))


@pytest.mark.parametrize(
    ("optical_x", "expected_base_y"),
    [(0.10, -0.10), (-0.10, 0.10)],
)
def test_goal1_optical_transform_produces_base_lateral_sign(
    optical_x: float, expected_base_y: float
) -> None:
    """Camera-right is base-right, while camera-left is positive base lateral."""
    transform = identity_transform()
    transform.transform.translation.x = 0.042
    transform.transform.translation.z = 0.120
    transform.transform.rotation.x = -0.5
    transform.transform.rotation.y = 0.5
    transform.transform.rotation.z = -0.5
    transform.transform.rotation.w = 0.5

    result = transform_pose_preserving_stamp(
        make_pose(position=(optical_x, 0.0, 1.0)), transform, "base_link"
    )

    assert result is not None
    assert result.pose.position.x == pytest.approx(1.042)
    assert result.pose.position.y == pytest.approx(expected_base_y)
    metrics = compute_base_metrics(result.pose.position.x, result.pose.position.y)
    assert metrics.lateral_error == pytest.approx(expected_base_y)
    assert (metrics.bearing > 0.0) == (expected_base_y > 0.0)


@pytest.mark.parametrize(
    "position",
    [(nan, 0.0, 0.0), (0.0, inf, 0.0), (0.0, 0.0, -inf)],
)
def test_transform_rejects_invalid_input_position(
    position: Tuple[float, float, float],
) -> None:
    assert transform_pose_preserving_stamp(
        make_pose(position=position), identity_transform(), "base_link"
    ) is None


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0), (nan, 0.0, 0.0, 1.0), (0.0, inf, 0.0, 1.0)],
)
def test_transform_rejects_invalid_input_quaternion(
    quaternion: Tuple[float, float, float, float],
) -> None:
    assert transform_pose_preserving_stamp(
        make_pose(quaternion=quaternion), identity_transform(), "base_link"
    ) is None


def test_transform_rejects_invalid_frames_and_transform() -> None:
    pose = make_pose()
    pose.header.frame_id = ""
    assert transform_pose_preserving_stamp(pose, identity_transform(), "base_link") is None
    assert transform_pose_preserving_stamp(make_pose(), identity_transform(), "") is None

    transform = identity_transform()
    transform.transform.translation.x = nan
    assert transform_pose_preserving_stamp(make_pose(), transform, "base_link") is None
    transform = identity_transform()
    transform.transform.rotation.w = 0.0
    assert transform_pose_preserving_stamp(make_pose(), transform, "base_link") is None


def test_transform_normalizes_pose_quaternion() -> None:
    result = transform_pose_preserving_stamp(
        make_pose(quaternion=(0.0, 0.0, 0.0, 2.0)),
        identity_transform(),
        "base_link",
    )
    assert result is not None
    assert result.pose.orientation.w == pytest.approx(1.0)


@pytest.mark.parametrize("invalid_output", ["position", "quaternion"])
def test_transform_rejects_invalid_transformed_pose(
    monkeypatch: pytest.MonkeyPatch, invalid_output: str
) -> None:
    transformed = Pose()
    transformed.orientation.w = 1.0
    if invalid_output == "position":
        transformed.position.x = nan
    else:
        transformed.orientation.w = 0.0
    monkeypatch.setattr(
        base_pose_module, "do_transform_pose", lambda _pose, _transform: transformed
    )
    assert transform_pose_preserving_stamp(
        make_pose(), identity_transform(), "base_link"
    ) is None


class RecordingPublisher:
    """Publisher double that records messages."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class RecordingBuffer:
    """TF buffer double that records exact lookup arguments."""

    def __init__(self, transform: TransformStamped) -> None:
        self.transform = transform
        self.calls = []

    def lookup_transform(self, target: str, source: str, time, timeout):
        self.calls.append((target, source, time.nanoseconds, timeout.nanoseconds))
        return self.transform


def base_harness(tf_buffer: object, now_ns: int = 12_500_000_000) -> SimpleNamespace:
    """Create the attributes used by the node's base output branch."""
    harness = SimpleNamespace(
        _base_frame="base_link",
        _tf_lookup_timeout=0.05,
        _tag_timeout=1.0,
        _active_tag_id=0,
        _tf_buffer=tf_buffer,
        _base_pose_pub=RecordingPublisher(),
        _base_forward_pub=RecordingPublisher(),
        _base_lateral_pub=RecordingPublisher(),
        _base_bearing_pub=RecordingPublisher(),
        _base_state_pub=RecordingPublisher(),
        _base_state_machine=BaseAlignmentStateMachine(
            BaseAlignmentThresholds(0.50, 0.03, 0.02, 5.0, 0.8, 1.0)
        ),
        _log_base_state_change=lambda _state: None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now_ns)
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda _message, **_kwargs: None),
    )
    harness._publish_base_lost = lambda now_seconds: (
        AprilTagApproachNode._publish_base_lost(harness, now_seconds)
    )
    return harness


def test_valid_node_cycle_uses_exact_pose_stamp_and_coherent_outputs() -> None:
    tf_buffer = RecordingBuffer(identity_transform())
    harness = base_harness(tf_buffer)
    camera_pose = make_pose(position=(1.0, 0.2, 0.0))

    AprilTagApproachNode._publish_base_outputs(harness, camera_pose)

    assert tf_buffer.calls == [("base_link", CAMERA_FRAME, 12_000_000_345, 50_000_000)]
    base_pose = harness._base_pose_pub.messages[0]
    assert base_pose.header.stamp == camera_pose.header.stamp
    assert base_pose.header.frame_id == "base_link"
    assert harness._base_forward_pub.messages[0].data == pytest.approx(1.0)
    assert harness._base_lateral_pub.messages[0].data == pytest.approx(0.2)
    assert harness._base_bearing_pub.messages[0].data > 0.0
    assert harness._base_state_pub.messages[0].data == ApproachState.TURN_LEFT.value


def test_camera_publish_uses_same_pose_for_base_branch() -> None:
    publishers = [RecordingPublisher() for _ in range(8)]
    base_inputs = []
    harness = SimpleNamespace(
        _source_frame=CAMERA_FRAME,
        _detected_pub=publishers[0],
        _tag_id_pub=publishers[1],
        _pose_pub=publishers[2],
        _distance_pub=publishers[3],
        _lateral_pub=publishers[4],
        _straight_pub=publishers[5],
        _angle_pub=publishers[6],
        _state_pub=publishers[7],
        _log_state_change=lambda _state: None,
        _publish_base_outputs=base_inputs.append,
    )
    selected = TagObservation(
        0, 0.1, 0.0, 1.0, (0.0, 0.0, 0.0, 1.0), 12_000_000_345
    )
    AprilTagApproachNode._publish_valid(
        harness, selected, compute_measurement(0.1, 0.0, 1.0), ApproachState.APPROACH
    )
    assert all(len(publisher.messages) == 1 for publisher in publishers)
    assert base_inputs == [publishers[2].messages[0]]


@pytest.mark.parametrize("failure", ["tf", "stale"])
def test_tf_failure_or_stale_pose_skips_base_pose_and_metrics(failure: str) -> None:
    class FailingBuffer:
        def lookup_transform(self, _target, _source, _time, timeout):
            raise TransformException("unavailable")

    harness = (
        base_harness(FailingBuffer())
        if failure == "tf"
        else base_harness(RecordingBuffer(identity_transform()), 14_000_000_000)
    )
    AprilTagApproachNode._publish_base_outputs(
        harness, make_pose(position=(1.0, 0.0, 0.0))
    )
    assert not harness._base_pose_pub.messages
    assert not harness._base_forward_pub.messages
    assert not harness._base_lateral_pub.messages
    assert not harness._base_bearing_pub.messages
    assert harness._base_state_pub.messages[-1].data == ApproachState.TAG_LOST.value


def test_camera_lost_publishes_only_loss_outputs_and_base_lost() -> None:
    harness = SimpleNamespace(
        _translation_filter=SimpleNamespace(reset=lambda: None),
        _active_tag_id=0,
        _state_machine=SimpleNamespace(
            update=lambda _sample, _now, _tag: ApproachState.TAG_LOST
        ),
        _base_state_machine=BaseAlignmentStateMachine(
            BaseAlignmentThresholds(0.50, 0.03, 0.02, 5.0, 0.8, 1.0)
        ),
        _detected_pub=RecordingPublisher(),
        _tag_id_pub=RecordingPublisher(),
        _state_pub=RecordingPublisher(),
        _base_state_pub=RecordingPublisher(),
        _log_state_change=lambda _state: None,
        _log_base_state_change=lambda _state: None,
    )
    harness._publish_base_lost = lambda now_seconds: (
        AprilTagApproachNode._publish_base_lost(harness, now_seconds)
    )
    AprilTagApproachNode._publish_lost(harness, 10.0)
    assert harness._detected_pub.messages[-1].data is False
    assert harness._tag_id_pub.messages[-1].data == -1
    assert harness._state_pub.messages[-1].data == ApproachState.TAG_LOST.value
    assert harness._base_state_pub.messages[-1].data == ApproachState.TAG_LOST.value
