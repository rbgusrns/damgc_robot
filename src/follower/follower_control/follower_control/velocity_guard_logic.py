"""ROS-independent validation and limiting for follower velocity commands."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class PlanarVelocity:
    """A differential-drive command in metres/radians per second."""

    linear_x: float
    angular_z: float


def clamp(value: float, limit: float) -> float:
    """Clamp a finite scalar to a symmetric limit."""
    return max(-limit, min(limit, value))


def sanitize_velocity(
    linear_x: float,
    angular_z: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> PlanarVelocity:
    """Reject non-finite data and clamp a valid planar command."""
    if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
        raise ValueError("speed limits must be greater than zero")
    values = (linear_x, angular_z, max_linear_speed, max_angular_speed)
    if not all(isfinite(value) for value in values):
        raise ValueError("velocity and limits must be finite")
    return PlanarVelocity(
        clamp(linear_x, max_linear_speed),
        clamp(angular_z, max_angular_speed),
    )


def command_is_fresh(now_seconds: float, received_seconds: float, timeout: float) -> bool:
    """Return whether the most recent command is still safe to apply."""
    if timeout <= 0.0 or not all(
        isfinite(value) for value in (now_seconds, received_seconds, timeout)
    ):
        return False
    age = now_seconds - received_seconds
    return 0.0 <= age <= timeout + 1.0e-9
