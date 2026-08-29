"""ROS-independent validation, limiting, and slew control for Leader Twist."""

from dataclasses import dataclass
from math import isfinite
from typing import Optional


@dataclass(frozen=True)
class PlanarVelocity:
    """The two permitted differential-drive velocity components."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class GuardParameters:
    """Finite-command and safety-boundary parameters."""

    max_linear_speed: float
    max_angular_speed: float
    max_linear_acceleration: float
    max_angular_acceleration: float
    command_timeout: float
    max_slew_dt: float
    axis_epsilon: float
    allow_reverse: bool

    def validate(self) -> None:
        """Raise ``ValueError`` for a configuration that cannot be safe."""
        values = (
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_linear_acceleration,
            self.max_angular_acceleration,
            self.command_timeout,
            self.max_slew_dt,
            self.axis_epsilon,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("Guard parameters must be finite")
        if self.max_linear_speed <= 0.0:
            raise ValueError("max_linear_speed must be greater than zero")
        if self.max_angular_speed <= 0.0:
            raise ValueError("max_angular_speed must be greater than zero")
        if self.max_linear_acceleration <= 0.0:
            raise ValueError("max_linear_acceleration must be greater than zero")
        if self.max_angular_acceleration <= 0.0:
            raise ValueError("max_angular_acceleration must be greater than zero")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")
        if self.max_slew_dt <= 0.0:
            raise ValueError("max_slew_dt must be greater than zero")
        if self.axis_epsilon < 0.0:
            raise ValueError("axis_epsilon must not be negative")


def clamp(value: float, limit: float) -> float:
    """Clamp a finite scalar to a symmetric limit."""
    return max(-limit, min(limit, value))


def command_is_fresh(now_seconds: float, received_seconds: float, timeout: float) -> bool:
    """Return whether a received raw command is still within its watchdog age."""
    values = (now_seconds, received_seconds, timeout)
    if timeout <= 0.0 or not all(isfinite(value) for value in values):
        return False
    age = now_seconds - received_seconds
    return 0.0 <= age <= timeout + 1.0e-9


def sanitize_velocity(
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    parameters: GuardParameters,
) -> Optional[PlanarVelocity]:
    """Reject invalid/non-planar commands and clamp permitted components."""
    parameters.validate()
    values = (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
    if not all(isfinite(value) for value in values):
        return None
    unused_axes = (linear_y, linear_z, angular_x, angular_y)
    if any(abs(value) > parameters.axis_epsilon for value in unused_axes):
        return None
    if not parameters.allow_reverse and linear_x < 0.0:
        return None
    return PlanarVelocity(
        linear_x=clamp(linear_x, parameters.max_linear_speed),
        angular_z=clamp(angular_z, parameters.max_angular_speed),
    )


def candidate_or_zero(
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    parameters: GuardParameters,
    *,
    enabled: bool,
) -> PlanarVelocity:
    """Apply the enabled gate and return zero for every rejected raw command."""
    if not enabled:
        return PlanarVelocity()
    candidate = sanitize_velocity(
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        parameters,
    )
    return candidate if candidate is not None else PlanarVelocity()


def apply_slew_limit(
    previous: PlanarVelocity,
    candidate: PlanarVelocity,
    elapsed_seconds: float,
    parameters: GuardParameters,
) -> PlanarVelocity:
    """Limit both acceleration directions, capping anomalous elapsed time."""
    parameters.validate()
    values = (
        previous.linear_x,
        previous.angular_z,
        candidate.linear_x,
        candidate.angular_z,
        elapsed_seconds,
    )
    if not all(isfinite(value) for value in values) or elapsed_seconds <= 0.0:
        return PlanarVelocity()
    dt = min(elapsed_seconds, parameters.max_slew_dt)
    linear_delta = parameters.max_linear_acceleration * dt
    angular_delta = parameters.max_angular_acceleration * dt
    linear = max(
        previous.linear_x - linear_delta,
        min(previous.linear_x + linear_delta, candidate.linear_x),
    )
    angular = max(
        previous.angular_z - angular_delta,
        min(previous.angular_z + angular_delta, candidate.angular_z),
    )
    return PlanarVelocity(
        linear_x=clamp(linear, parameters.max_linear_speed),
        angular_z=clamp(angular, parameters.max_angular_speed),
    )
