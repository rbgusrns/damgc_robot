import math

import pytest

from follower_control.velocity_guard_logic import command_is_fresh, sanitize_velocity


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
