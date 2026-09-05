"""Camera-independent tests for the Leader AprilTag approach logic."""

from math import atan2, inf, nan, radians
from typing import Tuple

import pytest

from rescue_robot_apriltag.approach_logic import (
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


def make_thresholds(**overrides: float) -> ApproachThresholds:
    """Create thresholds that make each state independently reachable."""
    values = {
        "target_distance": 0.15,
        "distance_tolerance": 0.02,
        "lateral_tolerance": 0.01,
        "angle_tolerance_deg": 5.0,
        "stable_time": 0.8,
    }
    values.update(overrides)
    return ApproachThresholds(**values)


def evaluate_translation(x: float, y: float, z: float) -> ApproachState:
    """Evaluate one valid translation using the standard test thresholds."""
    state_machine = ApproachStateMachine(make_thresholds())
    return state_machine.update(compute_measurement(x, y, z), 0.0, 0)


def make_observation(tag_id: int, x: float, y: float, z: float) -> TagObservation:
    """Create a valid observation for deterministic tag-selection tests."""
    return TagObservation(
        tag_id=tag_id,
        x=x,
        y=y,
        z=z,
        quaternion=(0.0, 0.0, 0.0, 1.0),
        stamp_nanoseconds=tag_id + 1,
    )


def test_negative_angle_beyond_tolerance_turns_left() -> None:
    assert evaluate_translation(-0.03, 0.0, 0.15) == ApproachState.TURN_LEFT


def test_positive_angle_beyond_tolerance_turns_right() -> None:
    assert evaluate_translation(0.03, 0.0, 0.15) == ApproachState.TURN_RIGHT


def test_centered_far_tag_requests_approach() -> None:
    assert evaluate_translation(0.0, 0.0, 0.18) == ApproachState.APPROACH


def test_centered_near_tag_reports_too_close() -> None:
    assert evaluate_translation(0.0, 0.0, 0.12) == ApproachState.TOO_CLOSE


def test_in_range_tag_left_of_lateral_tolerance_fine_aligns_left() -> None:
    assert evaluate_translation(-0.012, 0.0, 0.15) == ApproachState.FINE_ALIGN_LEFT


def test_in_range_tag_right_of_lateral_tolerance_fine_aligns_right() -> None:
    assert evaluate_translation(0.012, 0.0, 0.15) == ApproachState.FINE_ALIGN_RIGHT


def test_first_fully_in_range_measurement_starts_stabilizing() -> None:
    assert evaluate_translation(0.0, 0.0, 0.15) == ApproachState.STABILIZING


def test_continuous_stable_time_reaches_aligned() -> None:
    state_machine = ApproachStateMachine(make_thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert state_machine.update(measurement, 10.0, 0) == ApproachState.STABILIZING
    assert state_machine.update(measurement, 10.79, 0) == ApproachState.STABILIZING
    assert state_machine.update(measurement, 10.8, 0) == ApproachState.ALIGNED


def test_leaving_tolerance_resets_stable_timer() -> None:
    state_machine = ApproachStateMachine(make_thresholds())
    aligned = compute_measurement(0.0, 0.0, 0.15)
    far = compute_measurement(0.0, 0.0, 0.18)

    assert state_machine.update(aligned, 0.0, 0) == ApproachState.STABILIZING
    assert state_machine.update(aligned, 0.6, 0) == ApproachState.STABILIZING
    assert state_machine.update(far, 0.7, 0) == ApproachState.APPROACH
    assert state_machine.update(aligned, 1.0, 0) == ApproachState.STABILIZING
    assert state_machine.update(aligned, 1.79, 0) == ApproachState.STABILIZING
    assert state_machine.update(aligned, 1.8, 0) == ApproachState.ALIGNED


def test_missing_measurement_reports_tag_lost_and_resets_timer() -> None:
    state_machine = ApproachStateMachine(make_thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert state_machine.update(measurement, 0.0, 0) == ApproachState.STABILIZING
    assert state_machine.update(None, 0.5, None) == ApproachState.TAG_LOST
    assert state_machine.update(measurement, 1.0, 0) == ApproachState.STABILIZING


def test_selected_tag_change_resets_stable_timer() -> None:
    state_machine = ApproachStateMachine(make_thresholds())
    measurement = compute_measurement(0.0, 0.0, 0.15)

    assert state_machine.update(measurement, 0.0, 0) == ApproachState.STABILIZING
    assert state_machine.update(measurement, 0.8, 0) == ApproachState.ALIGNED
    assert state_machine.update(measurement, 0.9, 1) == ApproachState.STABILIZING


def test_measurement_uses_atan2_and_euclidean_distance() -> None:
    measurement = compute_measurement(0.03, 0.04, 0.12)

    assert measurement.distance == pytest.approx(0.12)
    assert measurement.lateral_error == pytest.approx(0.03)
    assert measurement.straight_distance == pytest.approx(0.13)
    assert measurement.angle == pytest.approx(atan2(0.03, 0.12))


@pytest.mark.parametrize("z", [0.13, 0.17])
def test_distance_tolerance_boundaries_are_in_range(z: float) -> None:
    state_machine = ApproachStateMachine(
        make_thresholds(lateral_tolerance=0.02)
    )
    state = state_machine.update(compute_measurement(0.0, 0.0, z), 0.0, 0)
    assert state == ApproachState.STABILIZING


@pytest.mark.parametrize("x", [-0.01, 0.01])
def test_lateral_tolerance_boundaries_are_in_range(x: float) -> None:
    state = ApproachStateMachine(make_thresholds()).update(
        compute_measurement(x, 0.0, 0.15), 0.0, 0
    )
    assert state == ApproachState.STABILIZING


@pytest.mark.parametrize("sign", [-1.0, 1.0])
def test_angle_tolerance_boundaries_are_in_range(sign: float) -> None:
    measurement = RelativeMeasurement(
        x=0.0,
        y=0.0,
        z=0.15,
        distance=0.15,
        lateral_error=0.0,
        straight_distance=0.15,
        angle=sign * radians(5.0),
    )
    state = ApproachStateMachine(make_thresholds()).update(measurement, 0.0, 0)
    assert state == ApproachState.STABILIZING


def test_priority_selection_follows_allowed_id_order() -> None:
    observations = [
        make_observation(0, 0.0, 0.0, 0.10),
        make_observation(1, 0.0, 0.0, 0.20),
        make_observation(2, 0.0, 0.0, 0.30),
    ]

    selected = select_observation(observations, [2, 1, 0], "priority")
    assert selected is not None
    assert selected.tag_id == 2


def test_nearest_selection_uses_straight_distance() -> None:
    observations = [
        make_observation(0, 0.20, 0.0, 0.20),
        make_observation(1, 0.01, 0.0, 0.10),
        make_observation(2, 0.0, 0.0, 0.15),
    ]

    selected = select_observation(observations, [0, 1, 2], "nearest")
    assert selected is not None
    assert selected.tag_id == 1


def test_nearest_tie_uses_allowed_id_order() -> None:
    observations = [
        make_observation(1, 0.0, 0.0, 0.10),
        make_observation(2, 0.0, 0.0, 0.10),
    ]

    selected = select_observation(observations, [2, 1], "nearest")
    assert selected is not None
    assert selected.tag_id == 2


def test_selection_ignores_ids_outside_allowed_list() -> None:
    selected = select_observation(
        [make_observation(4, 0.0, 0.0, 0.10)], [0, 1, 2], "priority"
    )
    assert selected is None


def test_median_filter_rejects_translation_outlier() -> None:
    translation_filter = MedianTranslationFilter(3)
    translation_filter.add(0.0, 0.0, 0.10, 1)
    translation_filter.add(0.01, 0.01, 0.11, 2)
    filtered = translation_filter.add(10.0, 10.0, 10.0, 3)

    assert filtered.x == pytest.approx(0.01)
    assert filtered.y == pytest.approx(0.01)
    assert filtered.z == pytest.approx(0.11)


def test_duplicate_timestamp_is_not_inserted_again() -> None:
    translation_filter = MedianTranslationFilter(3)
    first = translation_filter.add(0.01, 0.02, 0.10, 10)
    duplicate = translation_filter.add(9.0, 9.0, 9.0, 10)

    assert duplicate == first


def test_filter_reset_allows_same_timestamp_as_a_new_sample() -> None:
    translation_filter = MedianTranslationFilter(3)
    translation_filter.add(0.0, 0.0, 0.10, 1)
    translation_filter.reset()
    filtered = translation_filter.add(0.03, 0.04, 0.20, 1)

    assert filtered.x == pytest.approx(0.03)
    assert filtered.y == pytest.approx(0.04)
    assert filtered.z == pytest.approx(0.20)


@pytest.mark.parametrize(
    "invalid_thresholds",
    [
        make_thresholds(target_distance=0.0),
        make_thresholds(distance_tolerance=-0.01),
        make_thresholds(lateral_tolerance=-0.01),
        make_thresholds(angle_tolerance_deg=-1.0),
        make_thresholds(stable_time=-0.1),
        make_thresholds(target_distance=nan),
        make_thresholds(stable_time=inf),
    ],
)
def test_invalid_threshold_parameters_are_rejected(
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
def test_nan_inf_and_nonpositive_z_are_rejected(
    translation: Tuple[float, float, float],
) -> None:
    assert not is_valid_translation(*translation)
    with pytest.raises(ValueError):
        compute_measurement(*translation)


def test_quaternion_is_normalized() -> None:
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )


@pytest.mark.parametrize(
    "quaternion",
    [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (nan, 0.0, 0.0, 1.0),
        (0.0, inf, 0.0, 1.0),
    ],
)
def test_invalid_quaternion_is_rejected(quaternion: Tuple[float, ...]) -> None:
    assert normalize_quaternion(quaternion) is None


def test_invalid_filter_window_is_rejected() -> None:
    with pytest.raises(ValueError):
        MedianTranslationFilter(0)


def test_invalid_selection_mode_is_rejected() -> None:
    with pytest.raises(ValueError):
        select_observation(
            [make_observation(0, 0.0, 0.0, 0.10)], [0], "random"
        )


@pytest.mark.parametrize("now_seconds", [nan, inf])
def test_nonfinite_state_machine_time_is_rejected(now_seconds: float) -> None:
    state_machine = ApproachStateMachine(make_thresholds())
    with pytest.raises(ValueError):
        state_machine.update(
            compute_measurement(0.0, 0.0, 0.15), now_seconds, 0
        )

