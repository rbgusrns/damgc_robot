"""ROS-independent tests for the Leader velocity guard."""

from math import inf, nan
from types import SimpleNamespace

import pytest

from leader_approach_control.velocity_guard_logic import (
    GuardParameters,
    PlanarVelocity,
    apply_slew_limit,
    candidate_or_zero,
    command_is_fresh,
    sanitize_velocity,
)


def make_parameters(**overrides: object) -> GuardParameters:
    """Create conservative validation parameters for deterministic tests."""
    values = {
        "max_linear_speed": 0.05,
        "max_angular_speed": 0.20,
        "max_linear_acceleration": 0.10,
        "max_angular_acceleration": 0.40,
        "command_timeout": 0.30,
        "max_slew_dt": 0.10,
        "axis_epsilon": 1.0e-9,
        "allow_reverse": False,
    }
    values.update(overrides)
    return GuardParameters(**values)


def sanitize(
    linear_x: float = 0.02,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_x: float = 0.0,
    angular_y: float = 0.0,
    angular_z: float = 0.1,
):
    """Sanitize a raw planar command using default guard limits."""
    return sanitize_velocity(
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        make_parameters(),
    )


def test_disabled_nonzero_raw_is_zero() -> None:
    assert candidate_or_zero(
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, make_parameters(), enabled=False
    ) == PlanarVelocity()


def test_enabled_valid_command_is_accepted() -> None:
    assert candidate_or_zero(
        enabled=True,
        parameters=make_parameters(),
        linear_x=0.02,
        linear_y=0.0,
        linear_z=0.0,
        angular_x=0.0,
        angular_y=0.0,
        angular_z=0.1,
    ) == PlanarVelocity(0.02, 0.1)


def test_linear_and_angular_values_are_clamped() -> None:
    command = sanitize(linear_x=1.0, angular_z=-2.0)
    assert command == PlanarVelocity(0.05, -0.20)


@pytest.mark.parametrize("value", [nan, inf, -inf])
def test_nonfinite_velocity_is_rejected(value: float) -> None:
    assert sanitize(linear_x=value) is None
    assert sanitize(angular_z=value) is None
    assert candidate_or_zero(
        value, 0.0, 0.0, 0.0, 0.0, 0.0, make_parameters(), enabled=True
    ) == PlanarVelocity()


def test_reverse_is_rejected_when_disabled() -> None:
    assert sanitize(linear_x=-0.001) is None


@pytest.mark.parametrize(
    "axis",
    ["linear_y", "linear_z", "angular_x", "angular_y"],
)
def test_unused_axis_is_rejected(axis: str) -> None:
    values = {axis: 0.001}
    assert sanitize(**values) is None


def test_command_freshness_includes_timeout_boundary() -> None:
    assert command_is_fresh(10.30, 10.0, 0.30)
    assert not command_is_fresh(10.301, 10.0, 0.30)
    assert not command_is_fresh(9.9, 10.0, 0.30)


def test_slew_limit_ramps_from_previous_output() -> None:
    parameters = make_parameters()
    result = apply_slew_limit(
        PlanarVelocity(), PlanarVelocity(0.05, 0.20), 0.05, parameters
    )
    assert result.linear_x == pytest.approx(0.005)
    assert result.angular_z == pytest.approx(0.02)


def test_slew_limit_applies_to_deceleration_and_large_dt_is_capped() -> None:
    parameters = make_parameters()
    result = apply_slew_limit(
        PlanarVelocity(0.05, 0.20), PlanarVelocity(), 1.0, parameters
    )
    assert result.linear_x == pytest.approx(0.04)
    assert result.angular_z == pytest.approx(0.16)


@pytest.mark.parametrize("elapsed", [0.0, -1.0, nan, inf])
def test_invalid_elapsed_time_fails_closed(elapsed: float) -> None:
    assert apply_slew_limit(
        PlanarVelocity(), PlanarVelocity(0.05, 0.20), elapsed, make_parameters()
    ) == PlanarVelocity()


@pytest.mark.parametrize(
    "parameters",
    [
        make_parameters(max_linear_speed=0.0),
        make_parameters(max_angular_speed=nan),
        make_parameters(max_linear_acceleration=-1.0),
        make_parameters(command_timeout=0.0),
        make_parameters(max_slew_dt=-0.1),
        make_parameters(axis_epsilon=-1.0),
    ],
)
def test_invalid_guard_parameters_are_rejected(
    parameters: GuardParameters,
) -> None:
    with pytest.raises(ValueError):
        parameters.validate()


def test_shutdown_stop_publishes_configured_zero_burst() -> None:
    messages = []
    harness = SimpleNamespace(
        _shutdown_stop_count=3,
        _safe_pub=SimpleNamespace(publish=messages.append),
    )

    from leader_approach_control.velocity_guard_node import VelocityGuardNode

    VelocityGuardNode.stop(harness)

    assert len(messages) == 3
    assert all(message.linear.x == 0.0 for message in messages)
    assert all(message.angular.z == 0.0 for message in messages)
