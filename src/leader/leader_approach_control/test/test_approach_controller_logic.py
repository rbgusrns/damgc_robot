"""ROS-independent tests for Leader raw approach command calculation."""

from math import inf, nan

import pytest

from leader_approach_control.approach_controller_logic import (
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


def make_parameters(**overrides: object) -> ControllerParameters:
    """Create conservative provisional parameters for deterministic tests."""
    values = {
        "target_forward": 0.50,
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
    measurement: BaseControlMeasurement = BaseControlMeasurement(0.60, 0.0, 0.0),
    **gate_overrides: bool,
) -> PlanarCommand:
    """Evaluate one command with every safety gate enabled by default."""
    gates = {
        "enabled": True,
        "detected": True,
        "tag_valid": True,
        "fresh": True,
        "coherent": True,
    }
    gates.update(gate_overrides)
    return compute_approach_command(
        state, measurement, make_parameters(), **gates
    )


def test_turn_left_is_positive_rotation_only() -> None:
    command = compute(TURN_LEFT, BaseControlMeasurement(0.50, 0.05, 0.10))
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_turn_right_is_negative_rotation_only() -> None:
    command = compute(TURN_RIGHT, BaseControlMeasurement(0.50, -0.05, -0.10))
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_approach_generates_positive_forward_command() -> None:
    command = compute(APPROACH)
    assert command.linear_x > 0.0


def test_approach_left_error_generates_positive_correction() -> None:
    command = compute(APPROACH, BaseControlMeasurement(0.60, 0.02, 0.04))
    assert command.angular_z > 0.0


def test_approach_right_error_generates_negative_correction() -> None:
    command = compute(APPROACH, BaseControlMeasurement(0.60, -0.02, -0.04))
    assert command.angular_z < 0.0


@pytest.mark.parametrize("state", [TAG_LOST, ALIGNED, STABILIZING, TOO_CLOSE])
def test_required_stop_states_are_zero(state: str) -> None:
    assert compute(state) == PlanarCommand()


@pytest.mark.parametrize(
    "gate",
    ["enabled", "detected", "tag_valid", "fresh", "coherent"],
)
def test_failed_safety_gate_is_zero(gate: str) -> None:
    assert compute(APPROACH, **{gate: False}) == PlanarCommand()


@pytest.mark.parametrize(
    "measurement",
    [
        BaseControlMeasurement(nan, 0.0, 0.0),
        BaseControlMeasurement(inf, 0.0, 0.0),
        BaseControlMeasurement(0.60, nan, 0.0),
        BaseControlMeasurement(0.60, 0.0, -inf),
        BaseControlMeasurement(0.0, 0.0, 0.0),
    ],
)
def test_invalid_measurement_is_zero(measurement: BaseControlMeasurement) -> None:
    assert compute(APPROACH, measurement) == PlanarCommand()


def test_raw_linear_and_angular_candidates_are_saturated() -> None:
    command = compute(APPROACH, BaseControlMeasurement(10.0, 10.0, 10.0))
    assert command.linear_x == pytest.approx(0.05)
    assert command.angular_z == pytest.approx(0.20)


def test_reverse_is_disabled() -> None:
    command = compute(APPROACH, BaseControlMeasurement(0.40, 0.0, 0.0))
    assert command.linear_x == 0.0


def test_fine_align_uses_rotation_without_lateral_or_forward_command() -> None:
    left = compute(FINE_ALIGN_LEFT, BaseControlMeasurement(0.50, 0.03, 0.04))
    right = compute(FINE_ALIGN_RIGHT, BaseControlMeasurement(0.50, -0.03, -0.04))

    assert left.linear_x == 0.0
    assert left.angular_z > 0.0
    assert right.linear_x == 0.0
    assert right.angular_z < 0.0


def test_state_and_error_sign_mismatch_fails_closed() -> None:
    assert compute(
        TURN_LEFT, BaseControlMeasurement(0.50, -0.03, -0.10)
    ) == PlanarCommand()
    assert compute(
        FINE_ALIGN_RIGHT, BaseControlMeasurement(0.50, 0.03, 0.04)
    ) == PlanarCommand()


def test_unknown_state_is_zero() -> None:
    assert compute("INVALID") == PlanarCommand()


def test_source_and_receipt_freshness_include_timeout_boundary() -> None:
    assert sample_is_fresh(10.35, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.351, 10.0, 20.35, 20.0, 0.35)
    assert not sample_is_fresh(10.35, 10.0, 20.351, 20.0, 0.35)
    assert not sample_is_fresh(9.9, 10.0, 20.0, 20.0, 0.35)


def test_state_must_follow_pose_within_sync_window() -> None:
    assert samples_are_coherent(20.0, 20.10, 0.10)
    assert not samples_are_coherent(20.0, 20.101, 0.10)
    assert not samples_are_coherent(20.1, 20.0, 0.10)


@pytest.mark.parametrize(
    "parameters",
    [
        make_parameters(target_forward=0.0),
        make_parameters(linear_gain=0.0),
        make_parameters(angular_gain=-1.0),
        make_parameters(lateral_gain=-1.0),
        make_parameters(max_raw_linear_speed=0.0),
        make_parameters(max_raw_angular_speed=inf),
    ],
)
def test_invalid_parameters_are_rejected(parameters: ControllerParameters) -> None:
    with pytest.raises(ValueError):
        parameters.validate()
