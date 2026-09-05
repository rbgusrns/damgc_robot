"""Tests for validated Leader base-frame pose conversion and metrics."""

from math import atan2, cos, inf, nan, pi, radians, sin, sqrt
from types import SimpleNamespace
from typing import Tuple

import pytest
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformException
from rclpy.time import Time

import rescue_robot_apriltag.base_pose as base_pose_module
from rescue_robot_apriltag.approach_logic import (
    ApproachState,
    TagObservation,
    compute_measurement,
)
from rescue_robot_apriltag.base_alignment_logic import (
    AlignmentDecision,
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
)
from rescue_robot_apriltag.apriltag_approach_node import AprilTagApproachNode
from rescue_robot_apriltag.base_pose import (
    PlanarNormalMedianFilter,
    compute_base_metrics,
    compute_target_geometry,
    is_fresh_timestamp,
    normalize_angle,
    rotate_tag_z_to_base_xy,
    select_robot_facing_normal,
    transform_pose_preserving_stamp,
)


def make_alignment_thresholds() -> BaseAlignmentThresholds:
    return BaseAlignmentThresholds(
        orientation_engage_distance=0.40,
        orientation_disengage_distance=0.43,
        turn_enter_error_deg=8.0,
        turn_exit_error_deg=3.0,
        tag_recenter_enter_deg=18.0,
        tag_recenter_exit_deg=11.0,
        near_normal_correction_limit_deg=6.0,
        pre_align_position_tolerance=0.02,
        final_position_tolerance=0.020,
        final_yaw_tolerance_deg=5.0,
        final_realign_yaw_error_deg=8.0,
        stable_time=0.30,
        sample_timeout=1.0,
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


def test_tag_plus_z_is_rotated_then_projected_instead_of_using_raw_yaw() -> None:
    # +90 degrees about tag Y maps tag +Z to base +X.
    normal = rotate_tag_z_to_base_xy((0.0, sqrt(0.5), 0.0, sqrt(0.5)))
    assert normal == pytest.approx((1.0, 0.0))


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0), (nan, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)],
)
def test_invalid_or_vertical_tag_normal_projection_is_rejected(
    quaternion: Tuple[float, float, float, float],
) -> None:
    with pytest.raises(ValueError):
        rotate_tag_z_to_base_xy(quaternion)


def test_targets_are_on_apriltag_printed_front_side_at_configured_distances() -> None:
    geometry = compute_target_geometry(0.50, 0.0, -1.0, 0.0, 0.30, 0.20)
    assert geometry.prealign_x == pytest.approx(0.20)
    assert geometry.final_x == pytest.approx(0.30)
    assert geometry.prealign_y == pytest.approx(0.0)
    assert geometry.final_y == pytest.approx(0.0)
    assert (geometry.prealign_x - 0.50) * geometry.normal_x == pytest.approx(0.30)
    assert (geometry.final_x - 0.50) * geometry.normal_x == pytest.approx(0.20)
    assert geometry.target_yaw == pytest.approx(0.0)


def test_robot_facing_normal_is_kept_and_opposite_normal_is_flipped() -> None:
    assert select_robot_facing_normal(0.50, 0.0, -1.0, 0.0) == pytest.approx(
        (-1.0, 0.0)
    )
    assert select_robot_facing_normal(0.50, 0.0, 1.0, 0.0) == pytest.approx(
        (-1.0, 0.0)
    )


@pytest.mark.parametrize(
    "values",
    [
        (0.0, 0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, nan, 0.0),
        (1.0, 0.0, 0.0, 1.0),
    ],
)
def test_robot_facing_normal_rejects_invalid_or_ambiguous_vectors(values) -> None:
    with pytest.raises(ValueError):
        select_robot_facing_normal(*values)


def test_measured_leader_tf_outward_normal_produces_front_side_targets() -> None:
    # Regression from the Leader hardware TF observed on 2026-09-03:
    # tag position ~= (0.312, 0.056), quaternion ~= (0.545,-0.428,-0.441,0.571).
    normal_x, normal_y = rotate_tag_z_to_base_xy(
        (0.545, -0.428, -0.441, 0.571)
    )
    geometry = compute_target_geometry(
        0.312, 0.056, normal_x, normal_y, 0.30, 0.20
    )

    assert normal_x == pytest.approx(-0.970, abs=0.002)
    assert normal_y == pytest.approx(-0.244, abs=0.002)
    assert normal_x * 0.312 + normal_y * 0.056 < 0.0
    assert geometry.prealign_x == pytest.approx(0.021, abs=0.002)
    assert geometry.prealign_y == pytest.approx(-0.017, abs=0.002)
    assert geometry.final_x == pytest.approx(0.118, abs=0.002)
    assert geometry.final_y == pytest.approx(0.007, abs=0.002)
    assert geometry.target_yaw == pytest.approx(0.247, abs=0.002)


def test_robot_looking_at_tag_from_the_side_must_not_be_aligned() -> None:
    geometry = compute_target_geometry(
        0.50, 0.0, -cos(radians(45.0)), cos(radians(45.0)), 0.30, 0.20
    )
    assert geometry.final_position_error > 0.015
    assert abs(geometry.final_yaw_error) > radians(4.0)
    machine = BaseAlignmentStateMachine(make_alignment_thresholds())
    measurement = BaseAlignmentMeasurement(
        0.50,
        0.0,
        geometry.prealign_x,
        geometry.prealign_y,
        geometry.final_x,
        geometry.final_y,
        geometry.final_yaw_error,
        10.0,
    )
    decision = machine.update(measurement, 10.0, 0)
    assert decision.state not in {ApproachState.STABILIZING, ApproachState.ALIGNED}


def test_tilted_tag_geometry_can_reach_correct_alignment() -> None:
    tilt = radians(30.0)
    normal = (-cos(tilt), -sin(tilt))
    initial = compute_target_geometry(
        0.50, 0.20, normal[0], normal[1], 0.30, 0.20
    )
    assert initial.target_yaw == pytest.approx(tilt)

    # At the generated final pose, rotate world vectors into the new base frame.
    relative_tag_world = (
        0.50 - initial.final_x,
        0.20 - initial.final_y,
    )
    c = cos(-initial.target_yaw)
    s = sin(-initial.target_yaw)
    tag_x = c * relative_tag_world[0] - s * relative_tag_world[1]
    tag_y = s * relative_tag_world[0] + c * relative_tag_world[1]
    normal_x = c * normal[0] - s * normal[1]
    normal_y = s * normal[0] + c * normal[1]
    reached = compute_target_geometry(
        tag_x, tag_y, normal_x, normal_y, 0.30, 0.20
    )
    assert reached.final_position_error == pytest.approx(0.0, abs=1.0e-12)
    assert reached.final_yaw_error == pytest.approx(0.0, abs=1.0e-12)

    decision = BaseAlignmentStateMachine(make_alignment_thresholds()).update(
        BaseAlignmentMeasurement(
            tag_x,
            tag_y,
            reached.prealign_x,
            reached.prealign_y,
            reached.final_x,
            reached.final_y,
            reached.final_yaw_error,
            10.0,
        ),
        10.0,
        0,
    )
    assert decision.state == ApproachState.STABILIZING


def test_angle_wraparound_uses_short_rotation() -> None:
    error = normalize_angle(radians(179.0) - radians(-179.0))
    assert error == pytest.approx(radians(-2.0))


def test_normal_filter_ignores_duplicate_timestamp_and_normalizes_median() -> None:
    filter_ = PlanarNormalMedianFilter(3)
    assert filter_.add(1.0, 0.0, 1) == pytest.approx((1.0, 0.0))
    assert filter_.add(0.0, 1.0, 1) == pytest.approx((1.0, 0.0))
    x, y = filter_.add(1.0, 0.1, 2)
    assert x > 0.99
    assert y > 0.0


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
        _publish_base_outputs=lambda pose, _is_new: base_inputs.append(pose),
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
    harness = SimpleNamespace(
        _base_frame="base_link",
        _tf_lookup_timeout=0.0,
        _tag_timeout=1.0,
        _pre_align_distance=0.30,
        _final_target_distance=0.20,
        _active_tag_id=0,
        _tf_buffer=tf_buffer,
        _normal_filter=PlanarNormalMedianFilter(5),
        _base_pose_pub=RecordingPublisher(),
        _base_forward_pub=RecordingPublisher(),
        _base_lateral_pub=RecordingPublisher(),
        _base_bearing_pub=RecordingPublisher(),
        _normal_heading_pub=RecordingPublisher(),
        _prealign_target_pub=RecordingPublisher(),
        _final_target_pub=RecordingPublisher(),
        _control_target_pub=RecordingPublisher(),
        _command_pub=RecordingPublisher(),
        _control_mode_pub=RecordingPublisher(),
        _final_position_error_pub=RecordingPublisher(),
        _final_yaw_error_pub=RecordingPublisher(),
        _base_state_pub=RecordingPublisher(),
        _base_state_machine=BaseAlignmentStateMachine(make_alignment_thresholds()),
        _log_base_state_change=lambda _state: None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: Time(nanoseconds=12_500_000_000)
        ),
        get_logger=lambda: SimpleNamespace(warning=lambda _message, **_kwargs: None),
    )
    harness._publish_base_lost = lambda now_seconds: (
        AprilTagApproachNode._publish_base_lost(harness, now_seconds)
    )
    harness._make_target_pose = lambda base_pose, geometry, prealign: (
        AprilTagApproachNode._make_target_pose(
            harness, base_pose, geometry, prealign=prealign
        )
    )
    harness._make_control_pose = lambda base_pose, geometry, decision: (
        AprilTagApproachNode._make_control_pose(
            harness, base_pose, geometry, decision
        )
    )
    harness._publish_atomic_command = lambda control_pose, decision: (
        AprilTagApproachNode._publish_atomic_command(
            harness, control_pose, decision
        )
    )
    return harness


def test_node_base_branch_looks_up_input_pose_timestamp() -> None:
    tf_buffer = RecordingBuffer(make_identity_transform())
    harness = make_base_publish_harness(tf_buffer)
    camera_pose = make_pose(
        position=(1.0, 0.2, 0.0),
        # -90 degrees about Y maps tag +Z toward the observing base (-X).
        quaternion=(0.0, -sqrt(0.5), 0.0, sqrt(0.5)),
    )

    AprilTagApproachNode._publish_base_outputs(harness, camera_pose)

    assert tf_buffer.calls == [
        ("base_link", "camera_color_optical_frame", 12_000_000_345, 0)
    ]
    assert len(harness._base_pose_pub.messages) == 1
    assert harness._base_pose_pub.messages[0].header.stamp == camera_pose.header.stamp
    assert harness._base_forward_pub.messages[0].data == pytest.approx(1.0)
    assert harness._base_lateral_pub.messages[0].data == pytest.approx(0.2)
    assert harness._base_bearing_pub.messages[0].data > 0.0
    assert harness._base_state_pub.messages[0].data == ApproachState.TURN_LEFT.value
    assert harness._prealign_target_pub.messages[0].pose.position.x == pytest.approx(0.70)
    assert harness._final_target_pub.messages[0].pose.position.x == pytest.approx(0.80)
    assert harness._control_target_pub.messages[0].header.stamp == camera_pose.header.stamp
    assert harness._command_pub.messages[0].header.stamp == camera_pose.header.stamp
    assert harness._command_pub.messages[0].alignment_state == harness._base_state_pub.messages[0].data


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
    assert harness._base_state_pub.messages[-1].data == ApproachState.TAG_LOST.value


def test_camera_lost_cycle_also_publishes_base_tag_lost() -> None:
    """A camera loss must not leave the previous base state looking current."""
    camera_state_machine = SimpleNamespace(
        update=lambda _measurement, _now, _tag_id: ApproachState.TAG_LOST
    )
    harness = SimpleNamespace(
        _translation_filter=SimpleNamespace(reset=lambda: None),
        _normal_filter=SimpleNamespace(reset=lambda: None),
        _base_frame="base_link",
        _active_tag_id=0,
        _state_machine=camera_state_machine,
        _base_state_machine=BaseAlignmentStateMachine(make_alignment_thresholds()),
        _detected_pub=RecordingPublisher(),
        _tag_id_pub=RecordingPublisher(),
        _state_pub=RecordingPublisher(),
        _base_state_pub=RecordingPublisher(),
        _control_mode_pub=RecordingPublisher(),
        _command_pub=RecordingPublisher(),
        _log_state_change=lambda _state: None,
        _log_base_state_change=lambda _state: None,
            get_clock=lambda: SimpleNamespace(
                now=lambda: Time(nanoseconds=12_500_000_000)
        ),
        _publish_atomic_command=lambda control_pose, decision: (
            AprilTagApproachNode._publish_atomic_command(
                harness, control_pose, decision
            )
        ),
    )
    harness._publish_base_lost = lambda now_seconds: (
        AprilTagApproachNode._publish_base_lost(harness, now_seconds)
    )

    AprilTagApproachNode._publish_lost(harness, 10.0)

    assert harness._detected_pub.messages[-1].data is False
    assert harness._tag_id_pub.messages[-1].data == -1
    assert harness._state_pub.messages[-1].data == ApproachState.TAG_LOST.value
    assert harness._base_state_pub.messages[-1].data == ApproachState.TAG_LOST.value


def make_tag_observation(stamp_nanoseconds: int) -> TagObservation:
    """Create a valid source-stamped observation for lifecycle tests."""
    return TagObservation(
        tag_id=0,
        x=0.27,
        y=0.0,
        z=1.0,
        quaternion=(0.0, 0.0, 0.0, 1.0),
        stamp_nanoseconds=stamp_nanoseconds,
    )


def test_cached_tf_stamp_is_not_a_new_observation_or_snapshot_refresh() -> None:
    harness = SimpleNamespace(
        _last_processed_observation_stamps={},
        _last_valid_tag_x=None,
        _last_valid_timestamp=None,
        _last_valid_yaw_error=None,
        _last_valid_cross_track=None,
    )
    first = make_tag_observation(10_000_000_000)

    assert AprilTagApproachNode._accept_observation_stamp(harness, first)
    assert not AprilTagApproachNode._accept_observation_stamp(harness, first)

    AprilTagApproachNode._remember_last_valid_final_sample(
        harness,
        SimpleNamespace(forward_distance=0.27),
        SimpleNamespace(final_yaw_error=0.0, final_y=0.0),
        AlignmentDecision(ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH),
        10.0,
        True,
    )
    AprilTagApproachNode._remember_last_valid_final_sample(
        harness,
        SimpleNamespace(forward_distance=0.10),
        SimpleNamespace(final_yaw_error=0.5, final_y=0.5),
        AlignmentDecision(ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH),
        10.0,
        False,
    )

    assert harness._last_valid_tag_x == pytest.approx(0.27)
    assert harness._last_valid_timestamp == pytest.approx(10.0)
    assert harness._last_valid_yaw_error == pytest.approx(0.0)
    assert harness._last_valid_cross_track == pytest.approx(0.0)


def test_close_range_freshness_can_plan_before_global_tag_timeout() -> None:
    harness = SimpleNamespace(
        _last_valid_tag_x=0.27,
        _last_valid_timestamp=10.0,
        _last_valid_yaw_error=0.0,
        _last_valid_cross_track=0.0,
        _blind_last_tag_max_age=0.25,
        _blind_handoff_max_age=0.40,
        _final_target_distance=0.20,
        _blind_max_distance=0.12,
        _blind_final_approach_enabled=True,
        _blind_activation_max_tag_x=0.30,
        _final_yaw_tolerance_deg=4.0,
        _final_position_tolerance=0.015,
        _fresh_odom=lambda _now: (0.0, 0.0, 0.0),
    )

    planned = AprilTagApproachNode._blind_plan(harness, 10.25)

    assert planned == pytest.approx(0.07)


def test_blind_completion_sets_terminal_latch_and_lost_cannot_reenter() -> None:
    published = []
    harness = SimpleNamespace(
        _blind_start_time=10.0,
        _blind_max_duration=5.0,
        _blind_start_odom=(0.0, 0.0, 0.0),
        _blind_previous_odom=(0.05, 0.0, 0.0),
        _blind_planned_distance=0.07,
        _blind_active=True,
        _fresh_odom=lambda _now: (0.071, 0.0, 0.0),
        _last_odom_progress=0.05,
        _publish_blind_command=lambda state, mode, _now: published.append(
            (state, mode)
        ),
        _publish_completed_cycle=lambda _now: published.append("latched"),
    )

    AprilTagApproachNode._publish_blind_cycle(harness, 10.2)

    assert harness._blind_active is False
    assert harness._blind_completed is True
    assert published[-1] == (ApproachState.ALIGNED, ControlMode.ALIGNED)

    AprilTagApproachNode._publish_lost(harness, 10.3)

    assert published[-1] == "latched"
    assert harness._blind_active is False


def test_timer_treats_duplicate_tf_as_loss_candidate_at_close_range() -> None:
    selected = make_tag_observation(10_000_000_000)
    lost_cycles = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=False,
        _last_processed_observation_stamps={0: selected.stamp_nanoseconds},
        _blind_last_tag_max_age=0.25,
        _last_valid_timestamp=10.0,
        _accept_observation_stamp=lambda observation: AprilTagApproachNode._accept_observation_stamp(
            harness, observation
        ),
        _final_sample_age=lambda now: AprilTagApproachNode._final_sample_age(
            harness, now
        ),
        _selection_mode="priority",
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda _tag_id, _now: selected,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10_250_000_000)),
        _publish_lost=lambda now: lost_cycles.append(now),
    )

    AprilTagApproachNode._on_timer(harness)

    assert lost_cycles == [pytest.approx(10.25)]


def test_timer_does_not_abort_active_blind_for_duplicate_tf() -> None:
    selected = make_tag_observation(10_000_000_000)
    blind_cycles = []
    harness = SimpleNamespace(
        _blind_completed=False,
        _blind_active=True,
        _last_processed_observation_stamps={0: selected.stamp_nanoseconds},
        _accept_observation_stamp=lambda observation: AprilTagApproachNode._accept_observation_stamp(
            harness, observation
        ),
        _selection_mode="priority",
        _candidate_ids=lambda: (0,),
        _lookup_observation=lambda _tag_id, _now: selected,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10_100_000_000)),
        _publish_blind_cycle=lambda now: blind_cycles.append(now),
    )

    AprilTagApproachNode._on_timer(harness)

    assert blind_cycles == [pytest.approx(10.1)]


def test_timer_prioritizes_completed_latch_over_tag_reacquisition() -> None:
    completed_cycles = []
    lookups = []
    harness = SimpleNamespace(
        _blind_completed=True,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10_100_000_000)),
        _publish_completed_cycle=lambda now: completed_cycles.append(now),
        _candidate_ids=lambda: lookups.append(True),
    )

    AprilTagApproachNode._on_timer(harness)

    assert completed_cycles == [pytest.approx(10.1)]
    assert not lookups
