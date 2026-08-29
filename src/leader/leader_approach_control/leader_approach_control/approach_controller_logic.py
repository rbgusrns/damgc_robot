"""ROS-independent Leader approach command calculation and sample checks."""

from dataclasses import dataclass
from math import isfinite
from typing import Optional


TAG_LOST = "TAG_LOST"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
APPROACH = "APPROACH"
TOO_CLOSE = "TOO_CLOSE"
FINE_ALIGN_LEFT = "FINE_ALIGN_LEFT"
FINE_ALIGN_RIGHT = "FINE_ALIGN_RIGHT"
STABILIZING = "STABILIZING"
ALIGNED = "ALIGNED"

KNOWN_STATES = frozenset(
    {
        TAG_LOST,
        TURN_LEFT,
        TURN_RIGHT,
        APPROACH,
        TOO_CLOSE,
        FINE_ALIGN_LEFT,
        FINE_ALIGN_RIGHT,
        STABILIZING,
        ALIGNED,
    }
)


@dataclass(frozen=True)
class ControllerParameters:
    """Provisional gains and raw-output limits for software validation."""

    target_forward: float
    linear_gain: float
    angular_gain: float
    lateral_gain: float
    max_raw_linear_speed: float
    max_raw_angular_speed: float
    allow_reverse: bool

    def validate(self) -> None:
        """Raise ``ValueError`` when command calculation would be ambiguous."""
        numeric_values = (
            self.target_forward,
            self.linear_gain,
            self.angular_gain,
            self.lateral_gain,
            self.max_raw_linear_speed,
            self.max_raw_angular_speed,
        )
        if not all(isfinite(value) for value in numeric_values):
            raise ValueError("Controller parameters must be finite")
        if self.target_forward <= 0.0:
            raise ValueError("target_forward must be greater than zero")
        if self.linear_gain <= 0.0:
            raise ValueError("linear_gain must be greater than zero")
        if self.angular_gain <= 0.0:
            raise ValueError("angular_gain must be greater than zero")
        if self.lateral_gain < 0.0:
            raise ValueError("lateral_gain must not be negative")
        if self.max_raw_linear_speed <= 0.0:
            raise ValueError("max_raw_linear_speed must be greater than zero")
        if self.max_raw_angular_speed <= 0.0:
            raise ValueError("max_raw_angular_speed must be greater than zero")


@dataclass(frozen=True)
class BaseControlMeasurement:
    """Continuous planar errors derived from one stamped base-frame pose."""

    forward_distance: float
    lateral_error: float
    bearing: float


@dataclass(frozen=True)
class PlanarCommand:
    """The only two axes this differential-drive controller may command."""

    linear_x: float = 0.0
    angular_z: float = 0.0


def clamp(value: float, limit: float) -> float:
    """Clamp a finite scalar to a symmetric limit."""
    return max(-limit, min(limit, value))


def sample_is_fresh(
    now_seconds: float,
    stamp_seconds: float,
    received_now_seconds: float,
    received_seconds: float,
    timeout: float,
) -> bool:
    """Check both source-stamp age and local receipt age."""
    values = (
        now_seconds,
        stamp_seconds,
        received_now_seconds,
        received_seconds,
        timeout,
    )
    if timeout <= 0.0 or not all(isfinite(value) for value in values):
        return False
    source_age = now_seconds - stamp_seconds
    receipt_age = received_now_seconds - received_seconds
    return (
        stamp_seconds > 0.0
        and 0.0 <= source_age <= timeout + 1.0e-9
        and 0.0 <= receipt_age <= timeout + 1.0e-9
    )


def samples_are_coherent(
    pose_received_seconds: float,
    state_received_seconds: float,
    sync_tolerance: float,
) -> bool:
    """Require state receipt after the pose it describes and within one window."""
    values = (pose_received_seconds, state_received_seconds, sync_tolerance)
    if sync_tolerance < 0.0 or not all(isfinite(value) for value in values):
        return False
    delay = state_received_seconds - pose_received_seconds
    return 0.0 <= delay <= sync_tolerance + 1.0e-9


def compute_approach_command(
    state: str,
    measurement: Optional[BaseControlMeasurement],
    parameters: ControllerParameters,
    *,
    enabled: bool,
    detected: bool,
    tag_valid: bool,
    fresh: bool,
    coherent: bool,
) -> PlanarCommand:
    """Return a conservative raw command or zero for every unsafe condition."""
    parameters.validate()
    if (
        not enabled
        or not detected
        or not tag_valid
        or not fresh
        or not coherent
        or state not in KNOWN_STATES
        or measurement is None
    ):
        return PlanarCommand()

    values = (
        measurement.forward_distance,
        measurement.lateral_error,
        measurement.bearing,
    )
    if not all(isfinite(value) for value in values):
        return PlanarCommand()
    if measurement.forward_distance <= 0.0:
        return PlanarCommand()

    if state in {TAG_LOST, TOO_CLOSE, STABILIZING, ALIGNED}:
        return PlanarCommand()

    if state == TURN_LEFT:
        angular = clamp(
            parameters.angular_gain * measurement.bearing,
            parameters.max_raw_angular_speed,
        )
        return PlanarCommand(angular_z=angular) if angular > 0.0 else PlanarCommand()

    if state == TURN_RIGHT:
        angular = clamp(
            parameters.angular_gain * measurement.bearing,
            parameters.max_raw_angular_speed,
        )
        return PlanarCommand(angular_z=angular) if angular < 0.0 else PlanarCommand()

    angular = clamp(
        parameters.angular_gain * measurement.bearing
        + parameters.lateral_gain * measurement.lateral_error,
        parameters.max_raw_angular_speed,
    )

    if state == APPROACH:
        forward_error = measurement.forward_distance - parameters.target_forward
        linear = parameters.linear_gain * forward_error
        if not parameters.allow_reverse:
            linear = max(0.0, linear)
        linear = clamp(linear, parameters.max_raw_linear_speed)
        return PlanarCommand(linear_x=linear, angular_z=angular)

    if state == FINE_ALIGN_LEFT:
        return PlanarCommand(angular_z=angular) if angular > 0.0 else PlanarCommand()
    if state == FINE_ALIGN_RIGHT:
        return PlanarCommand(angular_z=angular) if angular < 0.0 else PlanarCommand()

    return PlanarCommand()
