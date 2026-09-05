"""ROS-independent regression and enhanced Follower guard tests."""

import math

import pytest

from follower_control.velocity_guard_logic import (
    GuardParameters,
    PlanarVelocity,
    apply_slew_limit,
    candidate_or_zero,
    command_is_fresh,
    sanitize_twist,
    sanitize_velocity,
)


def make_parameters(**overrides: object) -> GuardParameters:
    """Create deterministic parameters while retaining Follower speed limits."""
    values = {
        "max_linear_speed": 0.25,
        "max_angular_speed": 0.8,
        "max_linear_acceleration": 0.25,
        "max_angular_acceleration": 0.8,
        "command_timeout": 0.3,
        "max_slew_dt": 0.1,
        "axis_epsilon": 1.0e-9,
        "allow_reverse": False,
    }
    values.update(overrides)
    return GuardParameters(**values)


def sanitize(
    linear_x: float = 0.1,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_x: float = 0.0,
    angular_y: float = 0.0,
    angular_z: float = 0.2,
    parameters: GuardParameters = None,
):
    """Sanitize a complete Twist represented as six scalars."""
    return sanitize_twist(
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        parameters or make_parameters(),
    )


# Original follower_control regression tests.
def test_velocity_is_clamped_to_limits():
    result = sanitize_velocity(1.0, -2.0, 0.25, 0.8)
    assert result.linear_x == 0.25
    assert result.angular_z == -0.8


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_velocity_is_rejected(value):
    with pytest.raises(ValueError):
        sanitize_velocity(value, 0.0, 0.25, 0.8)


def test_command_freshness_includes_timeout_boundary():
    assert command_is_fresh(10.3, 10.0, 0.3)
    assert not command_is_fresh(10.301, 10.0, 0.3)
    assert not command_is_fresh(9.9, 10.0, 0.3)


# Leader-level safety behavior added to the existing guard.
def test_disabled_nonzero_candidate_is_zero() -> None:
    assert candidate_or_zero(
        0.2, 0.0, 0.0, 0.0, 0.0, 0.4, make_parameters(), enabled=False
    ) == PlanarVelocity()


def test_enabled_valid_candidate_is_accepted() -> None:
    assert candidate_or_zero(
        0.2, 0.0, 0.0, 0.0, 0.0, -0.4, make_parameters(), enabled=True
    ) == PlanarVelocity(0.2, -0.4)


def test_complete_twist_is_clamped() -> None:
    assert sanitize(linear_x=1.0, angular_z=-2.0) == PlanarVelocity(0.25, -0.8)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize(
    "axis",
    [
        "linear_x",
        "linear_y",
        "linear_z",
        "angular_x",
        "angular_y",
        "angular_z",
    ],
)
def test_nonfinite_in_any_twist_axis_is_rejected(axis: str, value: float) -> None:
    assert sanitize(**{axis: value}) is None


@pytest.mark.parametrize(
    "axis", ["linear_y", "linear_z", "angular_x", "angular_y"]
)
def test_nonplanar_axis_is_rejected(axis: str) -> None:
    assert sanitize(**{axis: 0.001}) is None


def test_unused_axis_within_epsilon_is_accepted() -> None:
    assert sanitize(linear_y=1.0e-9) == PlanarVelocity(0.1, 0.2)


def test_reverse_is_rejected_by_default() -> None:
    assert sanitize(linear_x=-0.001) is None


def test_reverse_can_be_explicitly_allowed() -> None:
    parameters = make_parameters(allow_reverse=True)
    assert sanitize(linear_x=-0.1, parameters=parameters) == PlanarVelocity(
        -0.1, 0.2
    )


def test_slew_limit_ramps_both_axes() -> None:
    result = apply_slew_limit(
        PlanarVelocity(), PlanarVelocity(0.25, 0.8), 0.02, make_parameters()
    )
    assert result.linear_x == pytest.approx(0.005)
    assert result.angular_z == pytest.approx(0.016)


def test_slew_limit_applies_to_deceleration_and_caps_large_dt() -> None:
    result = apply_slew_limit(
        PlanarVelocity(0.25, 0.8), PlanarVelocity(), 10.0, make_parameters()
    )
    assert result.linear_x == pytest.approx(0.225)
    assert result.angular_z == pytest.approx(0.72)


@pytest.mark.parametrize("elapsed", [0.0, -1.0, math.nan, math.inf])
def test_invalid_slew_dt_fails_closed(elapsed: float) -> None:
    result = apply_slew_limit(
        PlanarVelocity(), PlanarVelocity(0.1, 0.2), elapsed, make_parameters()
    )
    assert result == PlanarVelocity()


@pytest.mark.parametrize(
    "parameters",
    [
        make_parameters(max_linear_speed=0.0),
        make_parameters(max_angular_speed=math.nan),
        make_parameters(max_linear_acceleration=-1.0),
        make_parameters(max_angular_acceleration=0.0),
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
