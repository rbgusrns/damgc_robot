"""Pure tests for Follower hybrid alignment and blind eligibility."""

from math import nan, radians

import pytest

from follower_supply_perception.approach_logic import ApproachState
from follower_supply_perception.base_alignment_logic import (
    AlignmentDecision,
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
    ControlMode,
    compute_blind_remaining_distance,
    compute_forward_progress,
    compute_near_control,
    is_blind_final_approach_eligible,
)


def thresholds(**overrides) -> BaseAlignmentThresholds:
    values = dict(
        orientation_engage_distance=0.40,
        orientation_disengage_distance=0.43,
        turn_enter_error_deg=8.0,
        turn_exit_error_deg=3.0,
        tag_recenter_enter_deg=18.0,
        tag_recenter_exit_deg=11.0,
        near_normal_correction_limit_deg=6.0,
        pre_align_position_tolerance=0.03,
        final_forward_tolerance=0.03,
        final_lateral_tolerance=0.02,
        final_yaw_tolerance_deg=5.0,
        final_realign_yaw_error_deg=8.0,
        stable_time=0.30,
        sample_timeout=0.35,
        aligned_confirm_samples=3,
        stabilizing_tag_loss_grace_sec=0.30,
    )
    values.update(overrides)
    return BaseAlignmentThresholds(**values)


def measurement(
    *,
    tag_x=0.60,
    tag_y=0.0,
    prealign_x=0.25,
    prealign_y=0.0,
    final_x=0.35,
    final_y=0.0,
    yaw=0.0,
    stamp=10.0,
) -> BaseAlignmentMeasurement:
    return BaseAlignmentMeasurement(
        tag_x, tag_y, prealign_x, prealign_y, final_x, final_y, yaw, stamp
    )


def enter_final(machine, *, yaw=0.0, final_x=0.08, final_y=0.0, stamp=10.0):
    return machine.update(
        measurement(
            tag_x=0.35,
            prealign_x=0.0,
            final_x=final_x,
            final_y=final_y,
            yaw=yaw,
            stamp=stamp,
        ),
        stamp,
        0,
    )


def test_far_center_left_and_right_track_tag_center() -> None:
    centered = BaseAlignmentStateMachine(thresholds()).update(measurement(), 10.0, 0)
    left = BaseAlignmentStateMachine(thresholds()).update(
        measurement(tag_y=0.12), 10.0, 0
    )
    right = BaseAlignmentStateMachine(thresholds()).update(
        measurement(tag_y=-0.12), 10.0, 0
    )
    assert centered == AlignmentDecision(
        ApproachState.APPROACH, ControlMode.COARSE_TRACK, 0.60, 0.0
    )
    assert left.state == ApproachState.TURN_LEFT
    assert left.control_y > 0.0
    assert right.state == ApproachState.TURN_RIGHT
    assert right.control_y < 0.0


def test_far_ignores_misaligned_normal_target() -> None:
    decision = BaseAlignmentStateMachine(thresholds()).update(
        measurement(prealign_y=-2.0), 10.0, 0
    )
    assert decision.mode == ControlMode.COARSE_TRACK
    assert decision.control_y == 0.0


def test_orientation_engage_and_disengage_hysteresis() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    near = machine.update(
        measurement(tag_x=0.40, prealign_x=0.10, stamp=10.0), 10.0, 0
    )
    deadband = machine.update(
        measurement(tag_x=0.42, prealign_x=0.12, stamp=10.1), 10.1, 0
    )
    far = machine.update(
        measurement(tag_x=0.44, prealign_x=0.14, stamp=10.2), 10.2, 0
    )
    assert near.mode == deadband.mode == ControlMode.NEAR_ALIGN
    assert far.mode == ControlMode.COARSE_TRACK


def test_turn_enter_and_exit_hysteresis() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(
        measurement(tag_y=0.09), 10.0, 0
    ).state == ApproachState.TURN_LEFT
    assert machine.update(
        measurement(tag_y=0.05, stamp=10.1), 10.1, 0
    ).state == ApproachState.TURN_LEFT
    assert machine.update(
        measurement(tag_y=0.02, stamp=10.2), 10.2, 0
    ).state == ApproachState.APPROACH


def test_near_control_limits_normal_bias_and_fov_forward_scale() -> None:
    limited = compute_near_control(0.0, radians(30.0), radians(6.0), 0.1, 0.3)
    warning = compute_near_control(0.2, 0.3, radians(6.0), 0.1, 0.3)
    assert limited.normal_correction == pytest.approx(radians(6.0))
    assert 0.0 < warning.forward_scale < 1.0
    assert abs(warning.normal_correction) < radians(6.0)


def test_recenter_enter_exit_and_zero_forward_semantics() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    entered = machine.update(
        measurement(tag_x=0.35, tag_y=0.12, prealign_x=0.1), 10.0, 0
    )
    held = machine.update(
        measurement(tag_x=0.35, tag_y=0.08, prealign_x=0.1, stamp=10.1),
        10.1,
        0,
    )
    exited = machine.update(
        measurement(tag_x=0.35, tag_y=0.05, prealign_x=0.1, stamp=10.2),
        10.2,
        0,
    )
    assert entered.mode == held.mode == ControlMode.RECENTER
    assert entered.state == ApproachState.TURN_LEFT
    assert exited.mode == ControlMode.NEAR_ALIGN


def test_final_yaw_align_and_final_approach_realign() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    yaw_align = enter_final(machine, yaw=radians(6.0))
    approach = enter_final(machine, yaw=radians(4.0), stamp=10.1)
    deadband = enter_final(machine, yaw=radians(7.0), stamp=10.2)
    realign = enter_final(machine, yaw=radians(9.0), stamp=10.3)
    assert yaw_align.mode == ControlMode.FINAL_YAW_ALIGN
    assert yaw_align.state == ApproachState.FINE_ALIGN_LEFT
    assert approach.mode == deadband.mode == ControlMode.FINAL_APPROACH
    assert realign.mode == ControlMode.FINAL_YAW_ALIGN


def test_stabilizing_requires_position_and_yaw_then_aligns() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    first = enter_final(machine, final_x=0.01, final_y=0.01, stamp=10.0)
    confirm_one = enter_final(machine, final_x=0.01, final_y=0.01, stamp=10.3)
    confirm_two = enter_final(machine, final_x=0.01, final_y=0.01, stamp=10.4)
    aligned = enter_final(machine, final_x=0.01, final_y=0.01, stamp=10.5)
    assert first.state == ApproachState.STABILIZING
    assert confirm_one.state == ApproachState.STABILIZING
    assert confirm_two.state == ApproachState.STABILIZING
    assert aligned.state == ApproachState.ALIGNED
    wrong_position = BaseAlignmentStateMachine(thresholds())
    assert enter_final(wrong_position, final_y=0.03).state == ApproachState.FINAL_APPROACH
    wrong_yaw = BaseAlignmentStateMachine(thresholds())
    assert (
        enter_final(wrong_yaw, final_x=0.01, yaw=radians(6.0)).mode
        == ControlMode.FINAL_YAW_ALIGN
    )


def test_loss_stale_invalid_and_tag_change_reset_state() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(None, 10.0, None).mode == ControlMode.TAG_LOST
    assert machine.update(measurement(stamp=9.0), 10.0, 0).mode == ControlMode.TAG_LOST
    assert machine.update(measurement(tag_x=nan), 10.0, 0).mode == ControlMode.TAG_LOST
    assert machine.update(measurement(), 10.0, 1).mode == ControlMode.COARSE_TRACK


def test_stabilizing_loss_grace_pauses_elapsed_time_and_recovers() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    good = measurement(
        tag_x=0.25, prealign_x=0.0, final_x=0.0, final_y=0.0, stamp=10.0
    )

    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    dropout = machine.update(None, 10.29, None)
    grace_boundary = machine.update(None, 10.30, None)
    assert dropout.state == grace_boundary.state == ApproachState.STABILIZING
    assert dropout.control_x == dropout.control_y == 0.0
    assert grace_boundary.control_x == grace_boundary.control_y == 0.0

    recovered = measurement(
        tag_x=0.25, prealign_x=0.0, final_x=0.0, final_y=0.0, stamp=10.31
    )
    assert machine.update(recovered, 10.31, 0).state == ApproachState.STABILIZING
    assert machine._stable_since == pytest.approx(10.02)


def test_stabilizing_loss_beyond_grace_enters_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    good = measurement(
        tag_x=0.25, prealign_x=0.0, final_x=0.0, final_y=0.0, stamp=10.0
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    assert machine.update(None, 10.1, None).state == ApproachState.STABILIZING
    assert machine.update(None, 10.400001, None).state == ApproachState.TAG_LOST


def test_duplicate_samples_do_not_confirm_alignment() -> None:
    machine = BaseAlignmentStateMachine(thresholds(stable_time=0.0))
    good = measurement(
        tag_x=0.25, prealign_x=0.0, final_x=0.0, final_y=0.0, stamp=10.0
    )
    assert machine.update(good, 10.0, 0).state == ApproachState.STABILIZING
    for now in (10.1, 10.2, 10.3):
        assert (
            machine.update(good, now, 0, is_new_observation=False).state
            == ApproachState.STABILIZING
        )
    assert machine.update(
        measurement(tag_x=0.25, prealign_x=0.0, final_x=0.0, final_y=0.0, stamp=10.4),
        10.4,
        0,
    ).state == ApproachState.STABILIZING


def test_aligned_latch_survives_short_loss_and_reset_starts_new_session() -> None:
    machine = BaseAlignmentStateMachine(thresholds(stable_time=0.0))
    for stamp in (10.0, 10.1, 10.2):
        decision = machine.update(
            measurement(
                tag_x=0.25,
                prealign_x=0.0,
                final_x=0.0,
                final_y=0.0,
                stamp=stamp,
            ),
            stamp,
            0,
        )
    assert decision.state == ApproachState.ALIGNED
    assert machine.update(None, 10.3, None).state == ApproachState.ALIGNED

    machine.reset()
    next_session = machine.update(measurement(stamp=11.0), 11.0, 0)
    assert next_session.state == ApproachState.APPROACH


def blind_kwargs(**overrides):
    values = dict(
        enabled=True,
        phase=AlignmentDecision(ApproachState.FINAL_APPROACH, ControlMode.FINAL_APPROACH),
        last_valid_tag_x=0.30,
        last_valid_receipt=20.0,
        receipt_now=20.1,
        last_valid_yaw_error=radians(2.0),
        last_valid_cross_track=0.01,
        final_target_distance=0.25,
        activation_max_tag_x=0.35,
        max_distance=0.10,
        handoff_max_age=0.40,
        yaw_tolerance=radians(5.0),
        cross_track_tolerance=0.02,
        odometry_valid=True,
    )
    values.update(overrides)
    return values


def test_blind_is_only_eligible_from_recent_final_approach() -> None:
    assert is_blind_final_approach_eligible(**blind_kwargs())
    for phase in (
        AlignmentDecision(ApproachState.APPROACH, ControlMode.COARSE_TRACK),
        AlignmentDecision(ApproachState.APPROACH, ControlMode.NEAR_ALIGN),
        AlignmentDecision(ApproachState.TURN_LEFT, ControlMode.RECENTER),
        AlignmentDecision(ApproachState.FINE_ALIGN_LEFT, ControlMode.FINAL_YAW_ALIGN),
    ):
        assert not is_blind_final_approach_eligible(
            **blind_kwargs(phase=phase)
        )


@pytest.mark.parametrize(
    "override",
    [
        {"enabled": False},
        {"odometry_valid": False},
        {"receipt_now": 20.5},
        {"last_valid_tag_x": 0.40},
        {"last_valid_yaw_error": radians(6.0)},
        {"last_valid_cross_track": 0.03},
    ],
)
def test_blind_safety_gate_rejects_invalid_condition(override) -> None:
    assert not is_blind_final_approach_eligible(**blind_kwargs(**override))


def test_blind_distance_and_forward_progress_are_bounded() -> None:
    assert compute_blind_remaining_distance(0.30, 0.25, 0.10) == pytest.approx(0.05)
    assert compute_blind_remaining_distance(0.40, 0.25, 0.10) is None
    assert compute_blind_remaining_distance(0.20, 0.25, 0.10) is None
    assert compute_forward_progress(0.0, 0.0, 0.0, 0.05, 0.01) == pytest.approx(0.05)
    assert compute_forward_progress(0.0, 0.0, nan, 0.05, 0.0) is None


@pytest.mark.parametrize(
    "invalid",
    [
        thresholds(orientation_engage_distance=0.0),
        thresholds(orientation_disengage_distance=0.39),
        thresholds(turn_exit_error_deg=9.0),
        thresholds(tag_recenter_exit_deg=19.0),
        thresholds(final_forward_tolerance=0.0),
        thresholds(final_lateral_tolerance=0.0),
        thresholds(final_realign_yaw_error_deg=5.0),
        thresholds(aligned_confirm_samples=0),
        thresholds(stabilizing_tag_loss_grace_sec=-0.01),
    ],
)
def test_invalid_thresholds_are_rejected(invalid) -> None:
    with pytest.raises(ValueError):
        BaseAlignmentStateMachine(invalid)
