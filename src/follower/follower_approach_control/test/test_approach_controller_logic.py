"""ROS-independent tests for Follower raw approach commands."""

from math import inf, nan

import pytest

from follower_approach_control.approach_controller_logic import (
    ALIGNED,
    APPROACH,
    FINE_ALIGN_LEFT,
    FINE_ALIGN_RIGHT,
    STABILIZING,
    TAG_LOST,
    TOO_CLOSE,
    TURN_LEFT,
    TURN_RIGHT,
    BaseControlMeasurement,
    ControllerParameters,
    PlanarCommand,
    compute_approach_command,
    sample_is_fresh,
    samples_are_coherent,
)


def parameters(**overrides: object) -> ControllerParameters:
    """Create conservative software-validation parameters."""
    values = {
        "target_forward": 0.25,
        "linear_gain": 0.20,
        "angular_gain": 0.80,
        "lateral_gain": 0.50,
        "max_raw_linear_speed": 0.05,
        "max_raw_angular_speed": 0.20,
        "allow_reverse": False,
    }
    values.update(overrides)
    return ControllerParameters(**values)


def compute(
    state: str,
    sample: BaseControlMeasurement = BaseControlMeasurement(0.35, 0.0, 0.0),
    **gate_overrides: bool,
) -> PlanarCommand:
    """Compute with all safety gates passing unless overridden."""
    gates = {
        "enabled": True,
        "detected": True,
        "tag_valid": True,
        "fresh": True,
        "coherent": True,
    }
    gates.update(gate_overrides)
    return compute_approach_command(state, sample, parameters(), **gates)


def test_turn_left_is_positive_rotation_only() -> None:
    command = compute(TURN_LEFT, BaseControlMeasurement(0.25, 0.05, 0.10))
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_turn_right_is_negative_rotation_only() -> None:
    command = compute(TURN_RIGHT, BaseControlMeasurement(0.25, -0.05, -0.10))
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_approach_generates_positive_forward_command() -> None:
    assert compute(APPROACH).linear_x > 0.0


@pytest.mark.parametrize(
    ("sample", "expected_sign"),
    [
        (BaseControlMeasurement(0.35, 0.02, 0.04), 1.0),
        (BaseControlMeasurement(0.35, -0.02, -0.04), -1.0),
    ],
)
def test_approach_bearing_and_lateral_correction_signs(
    sample: BaseControlMeasurement, expected_sign: float
) -> None:
    command = compute(APPROACH, sample)
    assert command.angular_z * expected_sign > 0.0


@pytest.mark.parametrize("state", [TAG_LOST, TOO_CLOSE, STABILIZING, ALIGNED])
def test_stop_states_are_zero(state: str) -> None:
    assert compute(state) == PlanarCommand()


@pytest.mark.parametrize(
    "gate", ["enabled", "detected", "tag_valid", "fresh", "coherent"]
)
def test_failed_safety_gate_is_zero(gate: str) -> None:
    assert compute(APPROACH, **{gate: False}) == PlanarCommand()


@pytest.mark.parametrize(
    "sample",
    [
        BaseControlMeasurement(nan, 0.0, 0.0),
        BaseControlMeasurement(inf, 0.0, 0.0),
        BaseControlMeasurement(0.35, nan, 0.0),
        BaseControlMeasurement(0.35, 0.0, -inf),
        BaseControlMeasurement(0.0, 0.0, 0.0),
        BaseControlMeasurement(-0.1, 0.0, 0.0),
    ],
)
def test_invalid_measurement_is_zero(sample: BaseControlMeasurement) -> None:
    assert compute(APPROACH, sample) == PlanarCommand()


def test_raw_linear_and_angular_candidates_saturate() -> None:
    command = compute(APPROACH, BaseControlMeasurement(10.0, 10.0, 10.0))
    assert command.linear_x == pytest.approx(0.05)
    assert command.angular_z == pytest.approx(0.20)


def test_reverse_is_disabled() -> None:
    command = compute(APPROACH, BaseControlMeasurement(0.20, 0.0, 0.0))
    assert command == PlanarCommand()


def test_reverse_can_only_be_enabled_explicitly() -> None:
    command = compute_approach_command(
        APPROACH,
        BaseControlMeasurement(0.20, 0.0, 0.0),
        parameters(allow_reverse=True),
        enabled=True,
        detected=True,
        tag_valid=True,
        fresh=True,
        coherent=True,
    )
    assert command.linear_x < 0.0


def test_fine_align_rotates_without_forward_or_lateral_command() -> None:
    left = compute(FINE_ALIGN_LEFT, BaseControlMeasurement(0.25, 0.03, 0.04))
    right = compute(FINE_ALIGN_RIGHT, BaseControlMeasurement(0.25, -0.03, -0.04))
    assert left.linear_x == 0.0 and left.angular_z > 0.0
    assert right.linear_x == 0.0 and right.angular_z < 0.0


def test_state_and_error_sign_mismatch_fails_closed() -> None:
    assert compute(
        TURN_LEFT, BaseControlMeasurement(0.25, -0.03, -0.10)
    ) == PlanarCommand()
    assert compute(
        FINE_ALIGN_RIGHT, BaseControlMeasurement(0.25, 0.03, 0.04)
    ) == PlanarCommand()


def test_unknown_state_is_zero() -> None:
    assert compute("INVALID") == PlanarCommand()


def test_source_and_receipt_freshness_include_boundary() -> None:
    assert sample_is_fresh(10.35, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.351, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.35, 10.0, 20.351, 20.0, 0.35)
    assert not sample_is_fresh(9.9, 10.0, 20.0, 20.0, 0.35)
    assert not sample_is_fresh(10.0, 0.0, 20.0, 20.0, 0.35)


def test_state_must_follow_pose_within_sync_window() -> None:
    assert samples_are_coherent(20.0, 20.10, 0.10)
    assert not samples_are_coherent(20.0, 20.101, 0.10)
    assert not samples_are_coherent(20.1, 20.0, 0.10)
    assert not samples_are_coherent(20.0, 20.0, -0.1)


@pytest.mark.parametrize(
    "invalid",
    [
        parameters(target_forward=0.0),
        parameters(linear_gain=0.0),
        parameters(angular_gain=-1.0),
        parameters(lateral_gain=-1.0),
        parameters(max_raw_linear_speed=0.0),
        parameters(max_raw_angular_speed=inf),
    ],
)
def test_invalid_parameters_are_rejected(invalid: ControllerParameters) -> None:
    with pytest.raises(ValueError):
        invalid.validate()
