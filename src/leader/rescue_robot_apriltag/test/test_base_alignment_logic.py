"""Tests for visibility-first hybrid base alignment decisions."""

from math import atan2, cos, inf, nan, radians, sin

import pytest

from rescue_robot_apriltag.approach_logic import ApproachState
from rescue_robot_apriltag.base_alignment_logic import (
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
    AlignmentDecision,
    compute_blind_remaining_distance,
    compute_forward_progress,
    compute_near_control,
    is_blind_final_approach_eligible,
)


def make_thresholds(**overrides: float) -> BaseAlignmentThresholds:
    values = {
        "orientation_engage_distance": 0.40,
        "orientation_disengage_distance": 0.43,
        "turn_enter_error_deg": 8.0,
        "turn_exit_error_deg": 3.0,
        "tag_recenter_enter_deg": 18.0,
        "tag_recenter_exit_deg": 11.0,
        "near_normal_correction_limit_deg": 6.0,
        "pre_align_position_tolerance": 0.02,
        "final_position_tolerance": 0.020,
        "final_yaw_tolerance_deg": 5.0,
        "final_realign_yaw_error_deg": 8.0,
        "stable_time": 0.30,
        "sample_timeout": 1.0,
    }
    values.update(overrides)
    return BaseAlignmentThresholds(**values)


def make_measurement(
    *,
    tag_range: float = 0.50,
    tag_bearing_deg: float = 0.0,
    prealign_x: float = 0.20,
    prealign_y: float = 0.0,
    final_x: float = 0.10,
    final_y: float = 0.0,
    final_yaw_error: float = 0.0,
    stamp: float = 10.0,
) -> BaseAlignmentMeasurement:
    bearing = radians(tag_bearing_deg)
    return BaseAlignmentMeasurement(
        tag_range * cos(bearing),
        tag_range * sin(bearing),
        prealign_x,
        prealign_y,
        final_x,
        final_y,
        final_yaw_error,
        stamp,
    )


def test_lost_or_nonfinite_measurement_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(None, 10.0, None).state == ApproachState.TAG_LOST
    invalid = make_measurement(prealign_x=nan)
    assert machine.update(invalid, 10.0, 0).mode == ControlMode.TAG_LOST
    invalid = make_measurement(final_y=inf)
    assert machine.update(invalid, 10.0, 0).mode == ControlMode.TAG_LOST


def test_far_turns_using_tag_center_not_prealign_point() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    sample = make_measurement(
        tag_range=0.60,
        tag_bearing_deg=-10.0,
        prealign_x=0.10,
        prealign_y=1.0,
    )
    decision = machine.update(sample, 10.0, 0)
    assert decision.mode == ControlMode.COARSE_TRACK
    assert decision.state == ApproachState.TURN_RIGHT
    assert decision.control_x == pytest.approx(sample.tag_x)
    assert decision.control_y == pytest.approx(sample.tag_y)


def test_far_left_tag_turns_left() -> None:
    decision = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(tag_range=0.60, tag_bearing_deg=10.0), 10.0, 0
    )
    assert decision.state == ApproachState.TURN_LEFT


def test_far_centered_tag_approaches_even_if_prealign_is_far_left() -> None:
    decision = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(prealign_y=1.0), 10.0, 0
    )
    assert decision.mode == ControlMode.COARSE_TRACK
    assert decision.state == ApproachState.APPROACH
    assert decision.control_y == pytest.approx(0.0)


def test_orientation_distance_hysteresis_prevents_mode_chatter() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(
        make_measurement(tag_range=0.399), 10.0, 0
    ).mode == ControlMode.NEAR_ALIGN
    assert machine.update(
        make_measurement(tag_range=0.42, stamp=10.1), 10.1, 0
    ).mode == ControlMode.NEAR_ALIGN
    assert machine.update(
        make_measurement(tag_range=0.431, stamp=10.2), 10.2, 0
    ).mode == ControlMode.COARSE_TRACK


def test_near_adds_only_limited_normal_alignment_correction() -> None:
    sample = make_measurement(
        tag_range=0.35,
        tag_bearing_deg=2.0,
        prealign_x=0.10,
        prealign_y=0.20,
    )
    decision = BaseAlignmentStateMachine(make_thresholds()).update(sample, 10.0, 0)
    steering = pytest.approx(radians(8.0))
    assert decision.mode == ControlMode.NEAR_ALIGN
    assert decision.state == ApproachState.APPROACH
    assert decision.control_y > 0.0
    assert atan2(decision.control_y, decision.control_x) == steering


def test_fov_warning_reduces_forward_and_normal_correction() -> None:
    normal = compute_near_control(
        radians(5.0), radians(30.0), radians(6.0), radians(11.0), radians(18.0)
    )
    warning = compute_near_control(
        radians(15.0), radians(40.0), radians(6.0), radians(11.0), radians(18.0)
    )
    assert warning.forward_scale < normal.forward_scale
    assert abs(warning.normal_correction) < abs(normal.normal_correction)


def test_fov_warning_reduces_actual_control_target_range() -> None:
    normal_machine = BaseAlignmentStateMachine(make_thresholds())
    warning_machine = BaseAlignmentStateMachine(make_thresholds())
    normal = normal_machine.update(
        make_measurement(tag_range=0.35, tag_bearing_deg=5.0), 10.0, 0
    )
    warning = warning_machine.update(
        make_measurement(tag_range=0.35, tag_bearing_deg=15.0), 10.0, 0
    )
    assert (warning.control_x**2 + warning.control_y**2) < (
        normal.control_x**2 + normal.control_y**2
    )


def test_recenter_stops_forward_and_uses_tag_center_direction() -> None:
    left = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(tag_range=0.35, tag_bearing_deg=19.0), 10.0, 0
    )
    right = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(tag_range=0.35, tag_bearing_deg=-19.0), 10.0, 0
    )
    assert (left.mode, left.state) == (ControlMode.RECENTER, ApproachState.TURN_LEFT)
    assert (right.mode, right.state) == (
        ControlMode.RECENTER,
        ApproachState.TURN_RIGHT,
    )


def test_recenter_hysteresis_does_not_chatter_in_deadband() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(
        make_measurement(tag_range=0.35, tag_bearing_deg=19.0), 10.0, 0
    ).mode == ControlMode.RECENTER
    assert machine.update(
        make_measurement(tag_range=0.35, tag_bearing_deg=14.0, stamp=10.1),
        10.1,
        0,
    ).mode == ControlMode.RECENTER
    assert machine.update(
        make_measurement(tag_range=0.35, tag_bearing_deg=10.0, stamp=10.2),
        10.2,
        0,
    ).mode == ControlMode.NEAR_ALIGN


def test_turn_hysteresis_does_not_chatter_between_thresholds() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(
        make_measurement(tag_bearing_deg=9.0), 10.0, 0
    ).state == ApproachState.TURN_LEFT
    assert machine.update(
        make_measurement(tag_bearing_deg=5.0, stamp=10.1), 10.1, 0
    ).state == ApproachState.TURN_LEFT
    assert machine.update(
        make_measurement(tag_bearing_deg=2.0, stamp=10.2), 10.2, 0
    ).state == ApproachState.APPROACH
    assert machine.update(
        make_measurement(tag_bearing_deg=5.0, stamp=10.3), 10.3, 0
    ).state == ApproachState.APPROACH


def enter_final(machine: BaseAlignmentStateMachine, yaw: float = 0.0):
    return machine.update(
        make_measurement(
            tag_range=0.30,
            prealign_x=0.0,
            final_x=0.10,
            final_yaw_error=yaw,
        ),
        10.0,
        0,
    )


@pytest.mark.parametrize(
    ("yaw", "state"),
    [
        (radians(6.0), ApproachState.FINE_ALIGN_LEFT),
        (radians(-6.0), ApproachState.FINE_ALIGN_RIGHT),
    ],
)
def test_final_yaw_alignment_has_correct_state_sign(yaw, state) -> None:
    decision = enter_final(BaseAlignmentStateMachine(make_thresholds()), yaw)
    assert decision.mode == ControlMode.FINAL_YAW_ALIGN
    assert decision.state == state


def test_final_approach_realign_hysteresis() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert enter_final(machine).mode == ControlMode.FINAL_APPROACH
    middle = make_measurement(
        tag_range=0.29,
        prealign_x=0.0,
        final_x=0.08,
        final_yaw_error=radians(6.0),
        stamp=10.1,
    )
    assert machine.update(middle, 10.1, 0).mode == ControlMode.FINAL_APPROACH
    excessive = make_measurement(
        tag_range=0.28,
        prealign_x=0.0,
        final_x=0.07,
        final_yaw_error=radians(9.0),
        stamp=10.2,
    )
    assert machine.update(excessive, 10.2, 0).mode == ControlMode.FINAL_YAW_ALIGN


def test_final_approach_slows_in_fov_warning_region() -> None:
    centered = enter_final(BaseAlignmentStateMachine(make_thresholds()))
    warning = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(
            tag_range=0.30,
            tag_bearing_deg=15.0,
            prealign_x=0.0,
            final_x=0.10,
        ),
        10.0,
        0,
    )
    assert warning.mode == ControlMode.FINAL_APPROACH
    assert warning.control_x < centered.control_x


def test_final_phase_does_not_return_to_near_when_range_moves() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert enter_final(machine).mode == ControlMode.FINAL_APPROACH
    moved = make_measurement(
        tag_range=0.50,
        prealign_x=0.20,
        final_x=0.08,
        stamp=10.1,
    )
    assert machine.update(moved, 10.1, 0).mode == ControlMode.FINAL_APPROACH


def test_missed_prealign_skips_to_final_when_final_target_is_ahead() -> None:
    decision = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(
            tag_range=0.28,
            prealign_x=-0.02,
            final_x=0.05,
        ),
        10.0,
        0,
    )
    assert decision.mode == ControlMode.FINAL_APPROACH


def test_overshot_final_target_stops_without_reverse() -> None:
    decision = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(
            tag_range=0.15,
            prealign_x=-0.15,
            final_x=-0.05,
        ),
        10.0,
        0,
    )
    assert decision.state == ApproachState.TOO_CLOSE


def final_approach_decision() -> object:
    return AlignmentDecision(
        ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH
    )


def test_blind_distance_is_forward_only_and_bounded() -> None:
    assert compute_blind_remaining_distance(0.27, 0.20, 0.12) == pytest.approx(0.07)
    assert compute_blind_remaining_distance(0.20, 0.20, 0.12) == pytest.approx(0.0)
    assert compute_blind_remaining_distance(0.19, 0.20, 0.12) is None
    assert compute_blind_remaining_distance(0.50, 0.20, 0.12) is None


def test_blind_eligibility_requires_visual_final_approach_and_fresh_alignment() -> None:
    kwargs = dict(
        enabled=True,
        phase=final_approach_decision(),
        last_valid_tag_x=0.27,
        last_valid_timestamp=10.0,
        now_seconds=10.1,
        last_valid_yaw_error=radians(2.0),
        last_valid_cross_track=0.01,
        final_target_distance=0.20,
        activation_max_tag_x=0.30,
        max_distance=0.12,
        handoff_max_age=0.40,
        yaw_tolerance=radians(4.0),
        cross_track_tolerance=0.015,
        odometry_valid=True,
    )
    assert is_blind_final_approach_eligible(**kwargs)
    invalid_odom = dict(kwargs)
    invalid_odom["odometry_valid"] = False
    assert not is_blind_final_approach_eligible(**invalid_odom)
    for key, value in (
        ("last_valid_yaw_error", radians(6.0)),
        ("last_valid_cross_track", 0.02),
        ("last_valid_timestamp", 9.0),
    ):
        invalid = dict(kwargs)
        invalid[key] = value
        assert not is_blind_final_approach_eligible(**invalid)


@pytest.mark.parametrize(
    ("age", "expected"),
    [
        (0.20, True),
        (0.25, True),
        (0.30, True),
        (0.39, True),
        (0.41, False),
    ],
)
def test_handoff_age_is_separate_from_sample_loss_age(
    age: float, expected: bool
) -> None:
    assert is_blind_final_approach_eligible(
        enabled=True,
        phase=final_approach_decision(),
        last_valid_tag_x=0.26,
        last_valid_timestamp=10.0,
        now_seconds=10.0 + age,
        last_valid_yaw_error=0.0,
        last_valid_cross_track=0.0,
        final_target_distance=0.20,
        activation_max_tag_x=0.30,
        max_distance=0.12,
        handoff_max_age=0.40,
        yaw_tolerance=radians(4.0),
        cross_track_tolerance=0.015,
        odometry_valid=True,
    ) is expected


@pytest.mark.parametrize(
    "phase",
    [
        AlignmentDecision(ApproachState.TURN_LEFT, ControlMode.COARSE_TRACK),
        AlignmentDecision(ApproachState.FINE_ALIGN_LEFT, ControlMode.FINAL_YAW_ALIGN),
        AlignmentDecision(ApproachState.TURN_RIGHT, ControlMode.RECENTER),
    ],
)
def test_blind_eligibility_rejects_non_final_phases(phase) -> None:
    assert not is_blind_final_approach_eligible(
        enabled=True,
        phase=phase,
        last_valid_tag_x=0.27,
        last_valid_timestamp=10.0,
        now_seconds=10.1,
        last_valid_yaw_error=0.0,
        last_valid_cross_track=0.0,
        final_target_distance=0.20,
        activation_max_tag_x=0.30,
        max_distance=0.12,
        handoff_max_age=0.40,
        yaw_tolerance=radians(4.0),
        cross_track_tolerance=0.015,
        odometry_valid=True,
    )


@pytest.mark.parametrize(
    ("start_yaw", "dx", "dy", "expected"),
    [
        (0.0, 0.07, 0.0, 0.07),
        (radians(90.0), 0.0, 0.07, 0.07),
        (radians(45.0), 0.05, 0.05, 0.05 * 2**0.5),
        (0.0, 0.0, 0.07, 0.0),
    ],
)
def test_forward_progress_projects_onto_blind_start_heading(
    start_yaw, dx, dy, expected
) -> None:
    progress = compute_forward_progress(
        1.0, 2.0, start_yaw, 1.0 + dx, 2.0 + dy
    )
    assert progress == pytest.approx(expected)


def test_forward_progress_rejects_nonfinite_odom() -> None:
    assert compute_forward_progress(0.0, 0.0, nan, 1.0, 0.0) is None


def test_blind_eligibility_rejects_nonfinite_cached_tag_data() -> None:
    assert not is_blind_final_approach_eligible(
        enabled=True,
        phase=final_approach_decision(),
        last_valid_tag_x=nan,
        last_valid_timestamp=10.0,
        now_seconds=10.1,
        last_valid_yaw_error=0.0,
        last_valid_cross_track=0.0,
        final_target_distance=0.20,
        activation_max_tag_x=0.30,
        max_distance=0.12,
        handoff_max_age=0.40,
        yaw_tolerance=radians(4.0),
        cross_track_tolerance=0.015,
        odometry_valid=True,
    )


def test_tag_loss_restarts_final_phase() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert enter_final(machine).mode == ControlMode.FINAL_APPROACH
    assert machine.update(None, 10.1, None).mode == ControlMode.TAG_LOST
    restarted = make_measurement(tag_range=0.35, stamp=10.2)
    assert machine.update(restarted, 10.2, 0).mode == ControlMode.NEAR_ALIGN


def test_position_or_yaw_error_forbids_false_alignment() -> None:
    yaw_wrong = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(
            tag_range=0.20,
            prealign_x=0.0,
            final_x=0.0,
            final_yaw_error=radians(6.0),
        ),
        10.0,
        0,
    )
    position_wrong = enter_final(BaseAlignmentStateMachine(make_thresholds()))
    assert yaw_wrong.state not in {ApproachState.STABILIZING, ApproachState.ALIGNED}
    assert position_wrong.state == ApproachState.FINAL_APPROACH


def test_both_errors_good_stabilize_then_align() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    good = make_measurement(
        tag_range=0.20,
        prealign_x=0.0,
        final_x=0.0,
        final_y=0.0,
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    for stamp in (10.3, 10.4):
        good_later = make_measurement(
            tag_range=0.20,
            prealign_x=0.0,
            final_x=0.0,
            final_y=0.0,
            stamp=stamp,
        )
        decision = machine.update(good_later, stamp, 0)
    assert decision.state == ApproachState.STABILIZING
    good_final = make_measurement(
        tag_range=0.20,
        prealign_x=0.0,
        final_x=0.0,
        final_y=0.0,
        stamp=10.5,
    )
    assert machine.update(good_final, 10.5, 0).state == ApproachState.ALIGNED


def test_confirmation_requires_three_fresh_samples_after_stability() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    for stamp in (10.0, 10.3, 10.4):
        decision = machine.update(
            make_measurement(
                tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
                stamp=stamp,
            ),
            stamp,
            0,
        )
        assert decision.state == ApproachState.STABILIZING
    decision = machine.update(
        make_measurement(
            tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
            stamp=10.5,
        ),
        10.5,
        0,
    )
    assert decision.state == ApproachState.ALIGNED


def test_duplicate_timestamp_does_not_confirm_or_latch() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    sample = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
        stamp=10.0,
    )
    assert machine.update(sample, 10.0, 0).state == ApproachState.STABILIZING
    assert machine.update(sample, 10.3, 0, is_new_observation=False).state == ApproachState.STABILIZING
    for stamp in (10.1, 10.2):
        assert machine.update(
            make_measurement(
                tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
                stamp=stamp,
            ),
            10.3,
            0,
        ).state == ApproachState.STABILIZING
    assert machine.update(
        make_measurement(
            tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
            stamp=10.4,
        ),
        10.4,
        0,
    ).state == ApproachState.ALIGNED


def test_stabilizing_loss_grace_pauses_time_and_recovers() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    good = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
        stamp=10.0,
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    assert machine.update(None, 10.19, None).state == ApproachState.STABILIZING
    assert machine.update(None, 10.20, None).state == ApproachState.STABILIZING
    assert machine.update(None, 10.38, None).state == ApproachState.STABILIZING
    assert machine.update(None, 10.39, None).state == ApproachState.STABILIZING
    assert machine.update(None, 10.40, None).state == ApproachState.TAG_LOST

    restarted = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
        stamp=11.0,
    )
    assert machine.update(restarted, 11.0, 0).state == ApproachState.STABILIZING
    recovered = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
        stamp=11.1,
    )
    assert machine.update(recovered, 11.1, 0).state == ApproachState.STABILIZING


def test_aligned_latch_survives_loss_and_resets_explicitly() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    for stamp in (10.0, 10.3, 10.4, 10.5):
        decision = machine.update(
            make_measurement(
                tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0,
                stamp=stamp,
            ),
            stamp,
            0,
        )
    assert decision.state == ApproachState.ALIGNED
    assert machine.update(None, 20.0, None).state == ApproachState.ALIGNED
    machine.reset()
    assert machine.update(
        make_measurement(tag_range=0.50, stamp=20.1), 20.1, 0
    ).state == ApproachState.APPROACH


def test_stabilization_resets_when_position_leaves_tolerance() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    good = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    moving = make_measurement(
        tag_range=0.21,
        prealign_x=0.0,
        final_x=0.021,
        stamp=10.5,
    )
    assert machine.update(moving, 10.5, 0).state == ApproachState.FINAL_APPROACH
    recovered = make_measurement(
        tag_range=0.20,
        prealign_x=0.0,
        final_x=0.0,
        final_y=0.0,
        stamp=10.6,
    )
    assert machine.update(recovered, 10.6, 0).state == ApproachState.STABILIZING


def test_stabilization_resets_when_yaw_leaves_tolerance() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    good = make_measurement(
        tag_range=0.20, prealign_x=0.0, final_x=0.0, final_y=0.0
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    wrong = make_measurement(
        tag_range=0.20,
        prealign_x=0.0,
        final_x=0.0,
        final_y=0.0,
        final_yaw_error=radians(9.0),
        stamp=10.5,
    )
    assert machine.update(wrong, 10.5, 0).mode == ControlMode.FINAL_YAW_ALIGN


def test_correct_final_pose_can_stabilize_after_node_restart() -> None:
    decision = BaseAlignmentStateMachine(make_thresholds()).update(
        make_measurement(
            tag_range=0.20,
            prealign_x=-0.10,
            final_x=0.0,
            final_y=0.0,
        ),
        10.0,
        0,
    )
    assert decision.state == ApproachState.STABILIZING


def test_stale_future_and_changed_tag_reset_temporal_state() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(
        make_measurement(stamp=8.9), 10.0, 0
    ).state == ApproachState.TAG_LOST
    assert machine.update(
        make_measurement(stamp=10.1), 10.0, 0
    ).state == ApproachState.TAG_LOST
    assert machine.update(make_measurement(), 10.0, 1).mode == ControlMode.COARSE_TRACK


@pytest.mark.parametrize(
    "thresholds",
    [
        make_thresholds(orientation_engage_distance=0.0),
        make_thresholds(orientation_disengage_distance=0.39),
        make_thresholds(turn_exit_error_deg=9.0),
        make_thresholds(tag_recenter_exit_deg=19.0),
        make_thresholds(near_normal_correction_limit_deg=-1.0),
        make_thresholds(final_realign_yaw_error_deg=4.0),
        make_thresholds(final_position_tolerance=nan),
    ],
)
def test_invalid_thresholds_are_rejected(thresholds) -> None:
    with pytest.raises(ValueError):
        BaseAlignmentStateMachine(thresholds)
