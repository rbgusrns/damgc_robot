"""Tests for the two-stage tag-normal base alignment state machine."""

from math import inf, nan, radians

import pytest

from rescue_robot_apriltag.approach_logic import ApproachState
from rescue_robot_apriltag.base_alignment_logic import (
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
)


def make_thresholds(**overrides: float) -> BaseAlignmentThresholds:
    values = {
        "pre_align_position_tolerance": 0.02,
        "pre_align_heading_tolerance_deg": 5.0,
        "final_position_tolerance": 0.015,
        "final_yaw_tolerance_deg": 4.0,
        "stable_time": 0.8,
        "sample_timeout": 1.0,
    }
    values.update(overrides)
    return BaseAlignmentThresholds(**values)


def make_measurement(
    prealign_x: float = 0.0,
    prealign_y: float = 0.0,
    final_x: float = 0.0,
    final_y: float = 0.0,
    final_yaw_error: float = 0.0,
    stamp: float = 10.0,
) -> BaseAlignmentMeasurement:
    return BaseAlignmentMeasurement(
        prealign_x, prealign_y, final_x, final_y, final_yaw_error, stamp
    )


def evaluate(measurement: BaseAlignmentMeasurement) -> ApproachState:
    return BaseAlignmentStateMachine(make_thresholds()).update(measurement, 10.0, 0)


def test_lost_measurement_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(None, 10.0, None) == ApproachState.TAG_LOST


def test_prealign_target_on_left_turns_left() -> None:
    measurement = make_measurement(prealign_x=0.20, prealign_y=0.03)
    assert evaluate(measurement) == ApproachState.TURN_LEFT


def test_prealign_target_on_right_turns_right() -> None:
    measurement = make_measurement(prealign_x=0.20, prealign_y=-0.03)
    assert evaluate(measurement) == ApproachState.TURN_RIGHT


def test_centered_distant_prealign_target_requests_approach() -> None:
    assert evaluate(make_measurement(prealign_x=0.20)) == ApproachState.APPROACH


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (radians(5.0), ApproachState.FINE_ALIGN_LEFT),
        (radians(-5.0), ApproachState.FINE_ALIGN_RIGHT),
    ],
)
def test_prealign_reached_with_wrong_yaw_requests_final_yaw_align(
    yaw: float, expected: ApproachState
) -> None:
    assert evaluate(make_measurement(final_x=0.10, final_yaw_error=yaw)) == expected


def test_correct_yaw_with_distant_final_target_requests_final_approach() -> None:
    assert evaluate(make_measurement(final_x=0.10)) == ApproachState.FINAL_APPROACH


def test_final_position_correct_but_yaw_wrong_cannot_align() -> None:
    measurement = make_measurement(final_yaw_error=radians(5.0))
    assert evaluate(measurement) == ApproachState.FINE_ALIGN_LEFT


def test_yaw_correct_but_position_wrong_cannot_align() -> None:
    assert evaluate(make_measurement(final_x=0.02)) == ApproachState.FINAL_APPROACH


def test_first_complete_target_sample_starts_stabilizing() -> None:
    assert evaluate(make_measurement()) == ApproachState.STABILIZING


def test_continuous_position_and_yaw_stability_reaches_aligned() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(make_measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.5), 10.79, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED


def test_stabilization_resets_when_position_leaves_tolerance() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(make_measurement(), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.6), 10.6, 0) == ApproachState.STABILIZING
    moving = make_measurement(final_x=0.02, stamp=10.7)
    assert machine.update(moving, 10.7, 0) == ApproachState.FINAL_APPROACH
    assert machine.update(make_measurement(stamp=11.0), 11.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=11.79), 11.79, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=11.8), 11.8, 0) == ApproachState.ALIGNED


def test_stabilization_resets_when_yaw_leaves_tolerance() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(make_measurement(), 10.0, 0) == ApproachState.STABILIZING
    wrong_yaw = make_measurement(final_yaw_error=radians(5), stamp=10.4)
    assert machine.update(wrong_yaw, 10.4, 0) == ApproachState.FINE_ALIGN_LEFT
    assert machine.update(make_measurement(stamp=10.5), 10.5, 0) == ApproachState.STABILIZING


def test_final_phase_does_not_return_to_prealign() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(make_measurement(final_x=0.10), 10.0, 0) == ApproachState.FINAL_APPROACH
    moved = make_measurement(prealign_x=0.30, final_x=0.08, stamp=10.1)
    assert machine.update(moved, 10.1, 0) == ApproachState.FINAL_APPROACH


def test_missed_prealign_skips_to_final_when_final_target_is_ahead() -> None:
    measurement = make_measurement(prealign_x=-0.05, final_x=0.05)
    assert evaluate(measurement) == ApproachState.FINAL_APPROACH


def test_overshot_final_target_stops_without_reverse() -> None:
    measurement = make_measurement(prealign_x=-0.15, final_x=-0.05)
    assert evaluate(measurement) == ApproachState.TOO_CLOSE


def test_tag_lost_or_changed_tag_restarts_prealign_phase() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())
    assert machine.update(make_measurement(final_x=0.10), 10.0, 0) == ApproachState.FINAL_APPROACH
    assert machine.update(None, 10.1, None) == ApproachState.TAG_LOST
    next_tag = make_measurement(prealign_x=0.10, stamp=10.2)
    assert machine.update(next_tag, 10.2, 1) == ApproachState.APPROACH


@pytest.mark.parametrize(
    "measurement",
    [
        make_measurement(prealign_x=nan),
        make_measurement(prealign_y=inf),
        make_measurement(final_x=-inf),
        make_measurement(final_y=nan),
        make_measurement(final_yaw_error=inf),
        make_measurement(stamp=nan),
    ],
)
def test_invalid_measurement_reports_tag_lost(
    measurement: BaseAlignmentMeasurement,
) -> None:
    assert evaluate(measurement) == ApproachState.TAG_LOST


def test_stale_or_future_sample_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds(sample_timeout=1.0))
    assert machine.update(make_measurement(stamp=8.999), 10.0, 0) == ApproachState.TAG_LOST
    assert machine.update(make_measurement(stamp=10.1), 10.0, 0) == ApproachState.TAG_LOST


@pytest.mark.parametrize(
    "thresholds",
    [
        make_thresholds(pre_align_position_tolerance=0.0),
        make_thresholds(pre_align_heading_tolerance_deg=-1.0),
        make_thresholds(final_position_tolerance=0.0),
        make_thresholds(final_yaw_tolerance_deg=-1.0),
        make_thresholds(stable_time=-0.1),
        make_thresholds(sample_timeout=-0.1),
        make_thresholds(final_position_tolerance=nan),
    ],
)
def test_invalid_thresholds_are_rejected(
    thresholds: BaseAlignmentThresholds,
) -> None:
    with pytest.raises(ValueError):
        BaseAlignmentStateMachine(thresholds)
