"""Tests for validated Leader base-frame pose conversion and metrics."""

from math import atan2, inf, nan, pi, sqrt
from types import SimpleNamespace
from typing import Tuple

import pytest
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformException

import rescue_robot_apriltag.base_pose as base_pose_module
from rescue_robot_apriltag.approach_logic import (
    ApproachState,
    TagObservation,
    compute_measurement,
)
from rescue_robot_apriltag.apriltag_approach_node import AprilTagApproachNode
from rescue_robot_apriltag.base_pose import (
    compute_base_metrics,
    is_fresh_timestamp,
    transform_pose_preserving_stamp,
)


def make_pose(
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> PoseStamped:
    """Create a stamped camera-frame pose for deterministic tests."""
    pose = PoseStamped()
    pose.header.frame_id = "camera_color_optical_frame"
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


def make_identity_transform() -> TransformStamped:
    """Create an identity base-from-camera transform."""
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_color_optical_frame"
    transform.transform.rotation.w = 1.0
    return transform


def test_centered_base_point_has_zero_lateral_and_bearing() -> None:
    metrics = compute_base_metrics(1.0, 0.0)

    assert metrics.forward_distance == pytest.approx(1.0)
    assert metrics.lateral_error == pytest.approx(0.0)
    assert metrics.bearing == pytest.approx(0.0)


@pytest.mark.parametrize("y", [0.2, -0.2])
def test_base_lateral_and_bearing_signs_follow_y(y: float) -> None:
    metrics = compute_base_metrics(1.0, y)

    assert metrics.lateral_error == pytest.approx(y)
    assert metrics.bearing == pytest.approx(atan2(y, 1.0))
    assert (metrics.bearing > 0.0) == (y > 0.0)


def test_known_atan2_value() -> None:
    assert compute_base_metrics(1.0, 1.0).bearing == pytest.approx(pi / 4.0)


@pytest.mark.parametrize(
    ("x", "y", "expected"),
    [
        (0.0, 1.0, pi / 2.0),
        (0.0, -1.0, -pi / 2.0),
        (0.0, 0.0, 0.0),
        (-1.0, 0.0, pi),
    ],
)
def test_base_bearing_boundaries(x: float, y: float, expected: float) -> None:
    assert compute_base_metrics(x, y).bearing == pytest.approx(expected)


@pytest.mark.parametrize(("x", "y"), [(nan, 0.0), (0.0, inf), (-inf, 0.0)])
def test_base_metrics_reject_nan_and_inf(x: float, y: float) -> None:
    with pytest.raises(ValueError):
        compute_base_metrics(x, y)


def test_timestamp_freshness_includes_timeout_boundary() -> None:
    now = 12_500_000_000

    assert is_fresh_timestamp(12, 0, now, 0.5)
    assert not is_fresh_timestamp(11, 999_999_999, now, 0.5)


def test_timestamp_freshness_rejects_zero_future_and_invalid_timeout() -> None:
    assert not is_fresh_timestamp(0, 0, 1_000_000_000, 1.0)
    assert not is_fresh_timestamp(2, 0, 1_000_000_000, 1.0)
    assert not is_fresh_timestamp(1, 0, 1_000_000_000, -1.0)
    assert not is_fresh_timestamp(1, 1_000_000_000, 2_000_000_000, 1.0)


def test_known_transform_preserves_input_stamp_and_sets_target_frame() -> None:
    pose = make_pose(position=(1.0, 0.0, 0.0))
    transform = make_identity_transform()
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
    assert result.pose.position.z == pytest.approx(0.0)
    assert result.pose.orientation.z == pytest.approx(sqrt(0.5))
    assert result.pose.orientation.w == pytest.approx(sqrt(0.5))


@pytest.mark.parametrize(
    "position",
    [(nan, 0.0, 0.0), (0.0, inf, 0.0), (0.0, 0.0, -inf)],
)
def test_transform_rejects_invalid_input_position(
    position: Tuple[float, float, float],
) -> None:
    assert (
        transform_pose_preserving_stamp(
            make_pose(position=position), make_identity_transform(), "base_link"
        )
        is None
    )


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0, 0.0),
        (nan, 0.0, 0.0, 1.0),
        (0.0, inf, 0.0, 1.0),
    ],
)
def test_transform_rejects_invalid_input_quaternion(
    quaternion: Tuple[float, float, float, float],
) -> None:
    assert (
        transform_pose_preserving_stamp(
            make_pose(quaternion=quaternion),
            make_identity_transform(),
            "base_link",
        )
        is None
    )


def test_transform_rejects_empty_source_or_target_frame() -> None:
    pose = make_pose()
    pose.header.frame_id = ""
    assert (
        transform_pose_preserving_stamp(pose, make_identity_transform(), "base_link")
        is None
    )

    pose.header.frame_id = "camera_color_optical_frame"
    assert transform_pose_preserving_stamp(pose, make_identity_transform(), "") is None


def test_transform_rejects_invalid_transform_components() -> None:
    transform = make_identity_transform()
    transform.transform.translation.x = nan
    assert transform_pose_preserving_stamp(make_pose(), transform, "base_link") is None

    transform = make_identity_transform()
    transform.transform.rotation.w = 0.0
    assert transform_pose_preserving_stamp(make_pose(), transform, "base_link") is None


def test_transform_normalizes_input_and_output_quaternion() -> None:
    result = transform_pose_preserving_stamp(
        make_pose(quaternion=(0.0, 0.0, 0.0, 2.0)),
        make_identity_transform(),
        "base_link",
    )

    assert result is not None
    assert result.pose.orientation.x == pytest.approx(0.0)
    assert result.pose.orientation.y == pytest.approx(0.0)
    assert result.pose.orientation.z == pytest.approx(0.0)
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

    assert (
        transform_pose_preserving_stamp(
            make_pose(), make_identity_transform(), "base_link"
        )
        is None
    )


class RecordingPublisher:
    """Minimal publisher double that records messages in order."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, message: object) -> None:
        """Record one published message."""
        self.messages.append(message)


def test_valid_cycle_keeps_camera_outputs_and_passes_same_pose_to_base() -> None:
    publishers = [RecordingPublisher() for _ in range(8)]
    base_inputs = []
    harness = SimpleNamespace(
        _source_frame="camera_color_optical_frame",
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
        tag_id=0,
        x=0.1,
        y=0.0,
        z=1.0,
        quaternion=(0.0, 0.0, 0.0, 1.0),
        stamp_nanoseconds=12_000_000_345,
    )

    AprilTagApproachNode._publish_valid(
        harness, selected, compute_measurement(0.1, 0.0, 1.0), ApproachState.APPROACH
    )

    assert all(len(publisher.messages) == 1 for publisher in publishers)
    assert len(base_inputs) == 1
    assert base_inputs[0] is publishers[2].messages[0]
    assert base_inputs[0].header.stamp.sec == 12
    assert base_inputs[0].header.stamp.nanosec == 345


class RecordingBuffer:
    """TF buffer double that records exact-time lookup arguments."""

    def __init__(self, transform: TransformStamped) -> None:
        self.transform = transform
        self.calls = []

    def lookup_transform(self, target: str, source: str, time, timeout):
        """Record one lookup and return the configured transform."""
        self.calls.append((target, source, time.nanoseconds, timeout.nanoseconds))
        return self.transform


def make_base_publish_harness(tf_buffer: object) -> SimpleNamespace:
    """Create the attributes used by the node's base-output branch."""
    return SimpleNamespace(
        _base_frame="base_link",
        _tf_lookup_timeout=0.0,
        _tag_timeout=1.0,
        _tf_buffer=tf_buffer,
        _base_pose_pub=RecordingPublisher(),
        _base_forward_pub=RecordingPublisher(),
        _base_lateral_pub=RecordingPublisher(),
        _base_bearing_pub=RecordingPublisher(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=12_500_000_000)
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda _message, **_kwargs: None),
    )


def test_node_base_branch_looks_up_input_pose_timestamp() -> None:
    tf_buffer = RecordingBuffer(make_identity_transform())
    harness = make_base_publish_harness(tf_buffer)
    camera_pose = make_pose(position=(1.0, 0.2, 0.0))

    AprilTagApproachNode._publish_base_outputs(harness, camera_pose)

    assert tf_buffer.calls == [
        ("base_link", "camera_color_optical_frame", 12_000_000_345, 0)
    ]
    assert len(harness._base_pose_pub.messages) == 1
    assert harness._base_pose_pub.messages[0].header.stamp == camera_pose.header.stamp
    assert harness._base_forward_pub.messages[0].data == pytest.approx(1.0)
    assert harness._base_lateral_pub.messages[0].data == pytest.approx(0.2)
    assert harness._base_bearing_pub.messages[0].data > 0.0


def test_node_base_branch_publishes_nothing_on_tf_failure() -> None:
    class FailingBuffer:
        """TF buffer double that always reports an unavailable transform."""

        def lookup_transform(self, _target, _source, _time, timeout):
            """Raise the same exception as a real failed TF lookup."""
            raise TransformException("transform unavailable")

    harness = make_base_publish_harness(FailingBuffer())

    AprilTagApproachNode._publish_base_outputs(
        harness, make_pose(position=(1.0, 0.0, 0.0))
    )

    assert not harness._base_pose_pub.messages
    assert not harness._base_forward_pub.messages
    assert not harness._base_lateral_pub.messages
    assert not harness._base_bearing_pub.messages
