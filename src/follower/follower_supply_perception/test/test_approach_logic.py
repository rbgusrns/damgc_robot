"""Camera-independent tests for AprilTag approach filtering and state logic."""

from math import atan2, inf, nan, radians

import pytest

from follower_supply_perception.approach_logic import (
    ApproachState,
    ApproachStateMachine,
    ApproachThresholds,
    MedianTranslationFilter,
    RelativeMeasurement,
    TagObservation,
    compute_measurement,
    is_valid_translation,
    normalize_quaternion,
    select_observation,
)


def thresholds(**overrides: float) -> ApproachThresholds:
    """Return compact thresholds that make every state independently reachable."""
    values = {
        "target_distance": 0.15,
        "distance_tolerance": 0.02,
        "lateral_tolerance": 0.01,
        "angle_tolerance_deg": 5.0,
        "stable_time": 0.8,
    }
    values.update(overrides)
    return ApproachThresholds(**values)


def evaluate(x: float, y: float, z: float) -> ApproachState:
    """Evaluate a single fresh measurement with the standard test thresholds."""
    machine = ApproachStateMachine(thresholds())
    return machine.update(compute_measurement(x, y, z), 0.0, 0)


def observation(tag_id: int, x: float, y: float, z: float) -> TagObservation:
    """Create a valid observation for selection tests."""
    return TagObservation(tag_id, x, y, z, (0.0, 0.0, 0.0, 1.0), tag_id + 1)


def test_negative_angle_outside_tolerance_turns_left() -> None:
    assert evaluate(-0.03, 0.0, 0.15) == ApproachState.TURN_LEFT


def test_positive_angle_outside_tolerance_turns_right() -> None:
    assert evaluate(0.03, 0.0, 0.15) == ApproachState.TURN_RIGHT


def test_centered_tag_beyond_target_approaches() -> None:
    assert evaluate(0.0, 0.0, 0.18) == ApproachState.APPROACH


def test_centered_tag_inside_target_is_too_close() -> None:
    assert evaluate(0.0, 0.0, 0.12) == ApproachState.TOO_CLOSE


def test_in_range_tag_left_of_lateral_tolerance_fine_aligns_left() -> None:
    assert evaluate(-0.012, 0.0, 0.15) == ApproachState.FINE_ALIGN_LEFT


def test_in_range_tag_right_of_lateral_tolerance_fine_aligns_right() -> None:
    assert evaluate(0.012, 0.0, 0.15) == ApproachState.FINE_ALIGN_RIGHT


def test_all_errors_in_range_start_stabilizing() -> None:
    assert evaluate(0.0, 0.0, 0.15) == ApproachState.STABILIZING


def test_continuous_stable_time_reaches_aligned() -> None:
    machine = ApproachStateMachine(thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert machine.update(measurement, 10.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement, 10.79, 0) == ApproachState.STABILIZING
    assert machine.update(measurement, 10.8, 0) == ApproachState.ALIGNED


def test_condition_violation_resets_stabilization_timer() -> None:
    machine = ApproachStateMachine(thresholds())
    aligned_measurement = compute_measurement(0.0, 0.0, 0.15)
    far_measurement = compute_measurement(0.0, 0.0, 0.18)

    assert machine.update(aligned_measurement, 0.0, 0) == ApproachState.STABILIZING
    assert machine.update(aligned_measurement, 0.6, 0) == ApproachState.STABILIZING
    assert machine.update(far_measurement, 0.7, 0) == ApproachState.APPROACH
    assert machine.update(aligned_measurement, 1.0, 0) == ApproachState.STABILIZING
    assert machine.update(aligned_measurement, 1.79, 0) == ApproachState.STABILIZING
    assert machine.update(aligned_measurement, 1.81, 0) == ApproachState.ALIGNED


def test_tag_change_resets_stabilization_timer() -> None:
    machine = ApproachStateMachine(thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert machine.update(measurement, 0.0, 0) == ApproachState.STABILIZING
    assert machine.update(measurement, 0.8, 0) == ApproachState.ALIGNED
    assert machine.update(measurement, 0.9, 1) == ApproachState.STABILIZING


def test_missing_measurement_reports_lost_and_resets_timer() -> None:
    machine = ApproachStateMachine(thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert machine.update(measurement, 0.0, 0) == ApproachState.STABILIZING
    assert machine.update(None, 0.5, None) == ApproachState.TAG_LOST
    assert machine.update(measurement, 1.0, 0) == ApproachState.STABILIZING


def test_metric_calculation_uses_atan2_and_euclidean_distance() -> None:
    measurement = compute_measurement(0.03, 0.04, 0.12)

    assert measurement.distance == pytest.approx(0.12)
    assert measurement.lateral_error == pytest.approx(0.03)
    assert measurement.straight_distance == pytest.approx(0.13)
    assert measurement.angle == pytest.approx(atan2(0.03, 0.12))


@pytest.mark.parametrize("z", [0.13, 0.17])
def test_distance_tolerance_boundaries_are_in_range(z: float) -> None:
    machine = ApproachStateMachine(thresholds(lateral_tolerance=0.02))
    state = machine.update(compute_measurement(0.0, 0.0, z), 0.0, 0)
    assert state == ApproachState.STABILIZING


@pytest.mark.parametrize("x", [-0.01, 0.01])
def test_lateral_tolerance_boundaries_are_in_range(x: float) -> None:
    state = ApproachStateMachine(thresholds()).update(
        compute_measurement(x, 0.0, 0.15), 0.0, 0
    )
    assert state == ApproachState.STABILIZING


@pytest.mark.parametrize("sign", [-1.0, 1.0])
def test_angle_tolerance_boundaries_do_not_turn(sign: float) -> None:
    angle = sign * radians(5.0)
    measurement = RelativeMeasurement(
        x=0.0,
        y=0.0,
        z=0.15,
        distance=0.15,
        lateral_error=0.0,
        straight_distance=0.15,
        angle=angle,
    )
    state = ApproachStateMachine(thresholds()).update(measurement, 0.0, 0)
    assert state == ApproachState.STABILIZING


def test_priority_selection_uses_allowed_id_order() -> None:
    observations = [
        observation(0, 0.0, 0.0, 0.10),
        observation(2, 0.0, 0.0, 0.30),
        observation(1, 0.0, 0.0, 0.20),
    ]

    selected = select_observation(observations, [2, 1, 0], "priority")
    assert selected is not None
    assert selected.tag_id == 2


def test_nearest_selection_uses_straight_distance() -> None:
    observations = [
        observation(0, 0.20, 0.0, 0.20),
        observation(1, 0.01, 0.0, 0.10),
        observation(2, 0.0, 0.0, 0.15),
    ]

    selected = select_observation(observations, [0, 1, 2], "nearest")
    assert selected is not None
    assert selected.tag_id == 1


def test_nearest_tie_uses_allowed_id_order() -> None:
    observations = [
        observation(1, 0.0, 0.0, 0.10),
        observation(2, 0.0, 0.0, 0.10),
    ]

    selected = select_observation(observations, [2, 1], "nearest")
    assert selected is not None
    assert selected.tag_id == 2


def test_selection_ignores_observations_not_in_allowed_ids() -> None:
    selected = select_observation(
        [observation(4, 0.0, 0.0, 0.10)], [0, 1, 2], "priority"
    )
    assert selected is None


def test_median_filter_rejects_outlier_and_duplicate_stamp() -> None:
    translation_filter = MedianTranslationFilter(3)
    translation_filter.add(0.0, 0.0, 0.10, 1)
    translation_filter.add(0.01, 0.01, 0.11, 2)
    filtered = translation_filter.add(10.0, 10.0, 10.0, 3)
    assert filtered.x == pytest.approx(0.01)
    assert filtered.y == pytest.approx(0.01)
    assert filtered.z == pytest.approx(0.11)

    duplicate = translation_filter.add(-10.0, -10.0, 0.01, 3)
    assert duplicate == filtered


@pytest.mark.parametrize(
    "invalid_thresholds",
    [
        thresholds(target_distance=0.0),
        thresholds(distance_tolerance=-0.01),
        thresholds(lateral_tolerance=-0.01),
        thresholds(angle_tolerance_deg=-1.0),
        thresholds(stable_time=-0.1),
        thresholds(target_distance=nan),
        thresholds(stable_time=inf),
    ],
)
def test_invalid_thresholds_are_rejected(
    invalid_thresholds: ApproachThresholds,
) -> None:
    with pytest.raises(ValueError):
        ApproachStateMachine(invalid_thresholds)


@pytest.mark.parametrize(
    "translation",
    [
        (nan, 0.0, 0.1),
        (0.0, inf, 0.1),
        (0.0, 0.0, inf),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -0.1),
    ],
)
def test_invalid_translations_are_rejected(translation) -> None:
    assert not is_valid_translation(*translation)
    with pytest.raises(ValueError):
        compute_measurement(*translation)


def test_invalid_filter_window_and_selection_mode_are_rejected() -> None:
    with pytest.raises(ValueError):
        MedianTranslationFilter(0)
    with pytest.raises(ValueError):
        select_observation([observation(0, 0.0, 0.0, 0.1)], [0], "random")


def test_quaternion_validation_normalizes_and_rejects_invalid_values() -> None:
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )
    assert normalize_quaternion((0.0, 0.0, 0.0, 0.0)) is None
    assert normalize_quaternion((nan, 0.0, 0.0, 1.0)) is None
