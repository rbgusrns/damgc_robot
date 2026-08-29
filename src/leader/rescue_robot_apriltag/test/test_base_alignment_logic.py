"""ROS-independent tests for Leader base-frame alignment decisions."""

from math import atan2, inf, nan, radians

import pytest

from rescue_robot_apriltag.approach_logic import ApproachState
from rescue_robot_apriltag.base_alignment_logic import (
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
)


def make_thresholds(**overrides: float) -> BaseAlignmentThresholds:
    """Create provisional thresholds that make every state reachable."""
    values = {
        "target_forward": 0.50,
        "forward_tolerance": 0.03,
        "lateral_tolerance": 0.02,
        "bearing_tolerance_deg": 5.0,
        "stable_time": 0.8,
        "sample_timeout": 1.0,
    }
    values.update(overrides)
    return BaseAlignmentThresholds(**values)


def make_measurement(
    forward: float = 0.50,
    lateral: float = 0.0,
    bearing: float = 0.0,
    stamp: float = 10.0,
) -> BaseAlignmentMeasurement:
    """Create one base-frame measurement for deterministic tests."""
    return BaseAlignmentMeasurement(forward, lateral, bearing, stamp)


def evaluate(measurement: BaseAlignmentMeasurement) -> ApproachState:
    """Evaluate one fresh sample with the standard test thresholds."""
    return BaseAlignmentStateMachine(make_thresholds()).update(
        measurement, 10.0, 0
    )


def test_lost_measurement_reports_tag_lost() -> None:
    assert (
        BaseAlignmentStateMachine(make_thresholds()).update(None, 10.0, None)
        == ApproachState.TAG_LOST
    )


def test_positive_bearing_turns_left() -> None:
    assert evaluate(make_measurement(bearing=radians(6.0))) == ApproachState.TURN_LEFT


def test_negative_bearing_turns_right() -> None:
    assert evaluate(make_measurement(bearing=radians(-6.0))) == ApproachState.TURN_RIGHT


def test_centered_far_tag_requests_approach() -> None:
    assert evaluate(make_measurement(forward=0.54)) == ApproachState.APPROACH


def test_centered_near_tag_reports_too_close() -> None:
    assert evaluate(make_measurement(forward=0.46)) == ApproachState.TOO_CLOSE


def test_in_range_tag_on_left_requests_fine_align_left() -> None:
    lateral = 0.03
    assert evaluate(
        make_measurement(lateral=lateral, bearing=atan2(lateral, 0.50))
    ) == ApproachState.FINE_ALIGN_LEFT


def test_in_range_tag_on_right_requests_fine_align_right() -> None:
    lateral = -0.03
    assert evaluate(
        make_measurement(lateral=lateral, bearing=atan2(lateral, 0.50))
    ) == ApproachState.FINE_ALIGN_RIGHT


def test_first_in_tolerance_sample_starts_stabilizing() -> None:
    assert evaluate(make_measurement()) == ApproachState.STABILIZING


def test_continuous_stable_time_reaches_aligned() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())

    assert machine.update(make_measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.5), 10.79, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED


@pytest.mark.parametrize(
    "measurement",
    [
        make_measurement(forward=0.47),
        make_measurement(forward=0.53),
        make_measurement(lateral=0.02),
        make_measurement(lateral=-0.02),
        make_measurement(bearing=radians(5.0)),
        make_measurement(bearing=radians(-5.0)),
    ],
)
def test_tolerance_boundaries_are_in_range(
    measurement: BaseAlignmentMeasurement,
) -> None:
    assert evaluate(measurement) == ApproachState.STABILIZING


def test_stale_sample_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds(sample_timeout=1.0))
    assert machine.update(make_measurement(stamp=8.999), 10.0, 0) == ApproachState.TAG_LOST


@pytest.mark.parametrize(
    "measurement",
    [
        make_measurement(forward=nan),
        make_measurement(lateral=inf),
        make_measurement(bearing=-inf),
        make_measurement(stamp=nan),
        make_measurement(forward=0.0),
        make_measurement(forward=-0.1),
    ],
)
def test_invalid_measurement_reports_tag_lost(
    measurement: BaseAlignmentMeasurement,
) -> None:
    assert evaluate(measurement) == ApproachState.TAG_LOST


def test_leaving_tolerance_resets_stable_time() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())

    assert machine.update(make_measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.6), 10.6, 0) == ApproachState.STABILIZING
    assert (
        machine.update(make_measurement(forward=0.54, stamp=10.7), 10.7, 0)
        == ApproachState.APPROACH
    )
    assert machine.update(make_measurement(stamp=11.0), 11.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=11.79), 11.79, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=11.8), 11.8, 0) == ApproachState.ALIGNED


def test_tag_lost_then_recovery_restarts_stabilizing() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())

    assert machine.update(make_measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED
    assert machine.update(None, 10.9, None) == ApproachState.TAG_LOST
    assert machine.update(make_measurement(stamp=11.0), 11.0, 0) == ApproachState.STABILIZING


def test_invalid_or_changed_tag_resets_stability() -> None:
    machine = BaseAlignmentStateMachine(make_thresholds())

    assert machine.update(make_measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(make_measurement(stamp=10.8), 10.8, -1) == ApproachState.TAG_LOST
    assert machine.update(make_measurement(stamp=11.0), 11.0, 1) == ApproachState.STABILIZING


@pytest.mark.parametrize(
    "thresholds",
    [
        make_thresholds(target_forward=0.0),
        make_thresholds(target_forward=0.03, forward_tolerance=0.03),
        make_thresholds(forward_tolerance=-0.01),
        make_thresholds(lateral_tolerance=-0.01),
        make_thresholds(bearing_tolerance_deg=-1.0),
        make_thresholds(stable_time=-0.1),
        make_thresholds(sample_timeout=-0.1),
        make_thresholds(target_forward=nan),
    ],
)
def test_invalid_thresholds_are_rejected(
    thresholds: BaseAlignmentThresholds,
) -> None:
    with pytest.raises(ValueError):
        BaseAlignmentStateMachine(thresholds)
