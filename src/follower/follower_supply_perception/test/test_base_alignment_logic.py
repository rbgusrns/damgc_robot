"""ROS-independent tests for Follower base-frame alignment decisions."""

from math import atan2, inf, nan, radians

import pytest

from follower_supply_perception.approach_logic import ApproachState
from follower_supply_perception.base_alignment_logic import (
    BaseAlignmentMeasurement,
    BaseAlignmentStateMachine,
    BaseAlignmentThresholds,
)


def thresholds(**overrides: float) -> BaseAlignmentThresholds:
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


def measurement(
    forward: float = 0.50,
    lateral: float = 0.0,
    bearing: float = 0.0,
    stamp: float = 10.0,
) -> BaseAlignmentMeasurement:
    """Create one timestamped base sample."""
    return BaseAlignmentMeasurement(forward, lateral, bearing, stamp)


def evaluate(sample: BaseAlignmentMeasurement) -> ApproachState:
    """Evaluate a fresh sample with standard test thresholds."""
    return BaseAlignmentStateMachine(thresholds()).update(sample, 10.0, 0)


@pytest.mark.parametrize(
    ("sample", "expected"),
    [
        (measurement(bearing=radians(6.0)), ApproachState.TURN_LEFT),
        (measurement(bearing=radians(-6.0)), ApproachState.TURN_RIGHT),
        (measurement(forward=0.54), ApproachState.APPROACH),
        (measurement(forward=0.46), ApproachState.TOO_CLOSE),
        (
            measurement(lateral=0.03, bearing=atan2(0.03, 0.50)),
            ApproachState.FINE_ALIGN_LEFT,
        ),
        (
            measurement(lateral=-0.03, bearing=atan2(-0.03, 0.50)),
            ApproachState.FINE_ALIGN_RIGHT,
        ),
        (measurement(), ApproachState.STABILIZING),
    ],
)
def test_base_state_priority_states(
    sample: BaseAlignmentMeasurement, expected: ApproachState
) -> None:
    assert evaluate(sample) == expected


def test_lost_measurement_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(None, 10.0, None) == ApproachState.TAG_LOST


def test_continuous_stability_reaches_aligned() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=10.5), 10.79, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED


@pytest.mark.parametrize(
    "sample",
    [
        measurement(forward=0.47),
        measurement(forward=0.53),
        measurement(lateral=0.02),
        measurement(lateral=-0.02),
        measurement(bearing=radians(5.0)),
        measurement(bearing=radians(-5.0)),
    ],
)
def test_exact_tolerance_boundaries_are_inside(
    sample: BaseAlignmentMeasurement,
) -> None:
    assert evaluate(sample) == ApproachState.STABILIZING


def test_priority_checks_bearing_before_forward_and_lateral() -> None:
    sample = measurement(
        forward=0.80,
        lateral=-0.50,
        bearing=radians(8.0),
    )
    assert evaluate(sample) == ApproachState.TURN_LEFT


def test_stale_sample_reports_tag_lost() -> None:
    machine = BaseAlignmentStateMachine(thresholds(sample_timeout=1.0))
    assert machine.update(measurement(stamp=8.999), 10.0, 0) == ApproachState.TAG_LOST


@pytest.mark.parametrize(
    "sample",
    [
        measurement(forward=nan),
        measurement(lateral=inf),
        measurement(bearing=-inf),
        measurement(stamp=nan),
        measurement(forward=0.0),
        measurement(forward=-0.1),
    ],
)
def test_invalid_measurements_report_tag_lost(
    sample: BaseAlignmentMeasurement,
) -> None:
    assert evaluate(sample) == ApproachState.TAG_LOST


def test_leaving_tolerance_resets_stable_timer() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=10.6), 10.6, 0) == ApproachState.STABILIZING
    assert (
        machine.update(measurement(forward=0.54, stamp=10.7), 10.7, 0)
        == ApproachState.APPROACH
    )
    assert machine.update(measurement(stamp=11.0), 11.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=11.79), 11.79, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=11.8), 11.8, 0) == ApproachState.ALIGNED


def test_lost_then_recovery_restarts_stabilizing() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED
    assert machine.update(None, 10.9, None) == ApproachState.TAG_LOST
    assert machine.update(measurement(stamp=11.0), 11.0, 0) == ApproachState.STABILIZING


def test_tag_id_change_resets_stable_timer() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(measurement(stamp=10.0), 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement(stamp=10.8), 10.8, 0) == ApproachState.ALIGNED
    assert machine.update(measurement(stamp=10.9), 10.9, 1) == ApproachState.STABILIZING


def test_invalid_tag_id_resets_state() -> None:
    machine = BaseAlignmentStateMachine(thresholds())
    assert machine.update(measurement(), 10.0, -1) == ApproachState.TAG_LOST


@pytest.mark.parametrize(
    "invalid",
    [
        thresholds(target_forward=0.0),
        thresholds(target_forward=0.03, forward_tolerance=0.03),
        thresholds(forward_tolerance=-0.01),
        thresholds(lateral_tolerance=-0.01),
        thresholds(bearing_tolerance_deg=-1.0),
        thresholds(stable_time=-0.1),
        thresholds(sample_timeout=-0.1),
        thresholds(target_forward=nan),
    ],
)
def test_invalid_thresholds_are_rejected(invalid: BaseAlignmentThresholds) -> None:
    with pytest.raises(ValueError):
        BaseAlignmentStateMachine(invalid)
