"""ROS-independent deterministic command selection and validation."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional


class CommandSource(str, Enum):
    """The only command owners accepted by the selector."""

    STOP = "STOP"
    APPROACH = "APPROACH"
    COOPERATION = "COOPERATION"


@dataclass(frozen=True)
class PlanarCommand:
    """Validated differential-drive command."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class SelectorParameters:
    """Source-specific freshness and input-axis policy."""

    approach_timeout: float
    cooperation_timeout: float
    axis_epsilon: float

    def validate(self) -> None:
        """Reject timing or axis policies that cannot fail closed."""
        values = (
            self.approach_timeout,
            self.cooperation_timeout,
            self.axis_epsilon,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("Selector parameters must be finite")
        if self.approach_timeout <= 0.0:
            raise ValueError("approach_timeout must be greater than zero")
        if self.cooperation_timeout <= 0.0:
            raise ValueError("cooperation_timeout must be greater than zero")
        if self.axis_epsilon < 0.0:
            raise ValueError("axis_epsilon must not be negative")


def sanitize_command(
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    axis_epsilon: float,
) -> Optional[PlanarCommand]:
    """Return a finite planar command or ``None`` for malformed input."""
    values = (
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        axis_epsilon,
    )
    if not all(isfinite(value) for value in values) or axis_epsilon < 0.0:
        return None
    unused_axes = (linear_y, linear_z, angular_x, angular_y)
    if any(abs(value) > axis_epsilon for value in unused_axes):
        return None
    return PlanarCommand(linear_x=float(linear_x), angular_z=float(angular_z))


def command_is_fresh(
    now_seconds: float,
    received_seconds: Optional[float],
    timeout: float,
) -> bool:
    """Check a headerless Twist using its local monotonic receipt time."""
    if received_seconds is None:
        return False
    values = (now_seconds, received_seconds, timeout)
    if timeout <= 0.0 or not all(isfinite(value) for value in values):
        return False
    age = now_seconds - received_seconds
    return 0.0 <= age <= timeout + 1.0e-9


def select_command(
    source: CommandSource,
    now_seconds: float,
    parameters: SelectorParameters,
    *,
    approach_command: Optional[PlanarCommand],
    approach_received_seconds: Optional[float],
    cooperation_command: Optional[PlanarCommand],
    cooperation_received_seconds: Optional[float],
) -> PlanarCommand:
    """Return only the explicitly selected fresh source, otherwise zero."""
    parameters.validate()
    if source == CommandSource.STOP:
        return PlanarCommand()
    if source == CommandSource.APPROACH:
        if approach_command is not None and command_is_fresh(
            now_seconds,
            approach_received_seconds,
            parameters.approach_timeout,
        ):
            return approach_command
        return PlanarCommand()
    if source == CommandSource.COOPERATION:
        if cooperation_command is not None and command_is_fresh(
            now_seconds,
            cooperation_received_seconds,
            parameters.cooperation_timeout,
        ):
            return cooperation_command
        return PlanarCommand()
    return PlanarCommand()
