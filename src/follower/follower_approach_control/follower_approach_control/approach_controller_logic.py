"""ROS-independent Follower hybrid approach command calculation."""

from dataclasses import dataclass
from math import atan2, hypot, isfinite, sqrt
from typing import Optional, Sequence


TAG_LOST = "TAG_LOST"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
APPROACH = "APPROACH"
TOO_CLOSE = "TOO_CLOSE"
FINE_ALIGN_LEFT = "FINE_ALIGN_LEFT"
FINE_ALIGN_RIGHT = "FINE_ALIGN_RIGHT"
FINAL_APPROACH = "FINAL_APPROACH"
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
        FINAL_APPROACH,
        STABILIZING,
        ALIGNED,
    }
)

COARSE_TRACK = "COARSE_TRACK"
NEAR_ALIGN = "NEAR_ALIGN"
RECENTER = "RECENTER"
FINAL_YAW_ALIGN = "FINAL_YAW_ALIGN"
BLIND_FINAL_APPROACH = "BLIND_FINAL_APPROACH"

KNOWN_MODES = frozenset(
    {
        TAG_LOST,
        COARSE_TRACK,
        NEAR_ALIGN,
        RECENTER,
        FINAL_YAW_ALIGN,
        FINAL_APPROACH,
        BLIND_FINAL_APPROACH,
        STABILIZING,
        ALIGNED,
        TOO_CLOSE,
    }
)


@dataclass(frozen=True)
class ControllerParameters:
    linear_gain: float
    angular_gain: float
    lateral_gain: float
    max_raw_linear_speed: float
    max_raw_angular_speed: float
    near_max_angular_speed: float
    max_final_linear_speed: float
    max_final_angular_speed: float
    blind_final_speed: float

    def validate(self) -> None:
        values = tuple(self.__dict__.values())
        if not all(isfinite(value) for value in values):
            raise ValueError("Controller parameters must be finite")
        if self.linear_gain <= 0.0 or self.angular_gain <= 0.0:
            raise ValueError("linear_gain and angular_gain must be positive")
        if self.lateral_gain < 0.0:
            raise ValueError("lateral_gain must not be negative")
        if self.max_raw_linear_speed <= 0.0 or self.max_raw_angular_speed <= 0.0:
            raise ValueError("Raw speed limits must be positive")
        if not 0.0 < self.near_max_angular_speed <= self.max_raw_angular_speed:
            raise ValueError("near angular limit must be within raw angular limit")
        if not 0.0 < self.max_final_linear_speed <= self.max_raw_linear_speed:
            raise ValueError("final linear limit must be within raw linear limit")
        if not 0.0 < self.max_final_angular_speed <= self.max_raw_angular_speed:
            raise ValueError("final angular limit must be within raw angular limit")
        if not 0.0 < self.blind_final_speed <= self.max_final_linear_speed:
            raise ValueError("blind speed must be within final linear limit")


@dataclass(frozen=True)
class BaseControlMeasurement:
    target_x: float
    target_y: float
    target_yaw: float


@dataclass(frozen=True)
class PlanarCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def quaternion_yaw(quaternion: Sequence[float]) -> Optional[float]:
    if len(quaternion) != 4 or not all(isfinite(value) for value in quaternion):
        return None
    norm = sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        return None
    x, y, z, w = (value / norm for value in quaternion)
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def sample_is_fresh(
    now_seconds: float,
    stamp_seconds: float,
    received_now_seconds: float,
    received_seconds: float,
    timeout: float,
) -> bool:
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


def compute_approach_command(
    state: str,
    mode: str,
    measurement: Optional[BaseControlMeasurement],
    parameters: ControllerParameters,
    *,
    enabled: bool,
    detected: bool,
    tag_valid: bool,
    fresh: bool,
    coherent: bool,
) -> PlanarCommand:
    """Return zero for every invalid combination; otherwise one planar command."""
    parameters.validate()
    if (
        not enabled
        or not fresh
        or not coherent
        or state not in KNOWN_STATES
        or mode not in KNOWN_MODES
        or measurement is None
    ):
        return PlanarCommand()
    if not all(isfinite(value) for value in measurement.__dict__.values()):
        return PlanarCommand()
    if mode == BLIND_FINAL_APPROACH:
        if state != FINAL_APPROACH:
            return PlanarCommand()
        return PlanarCommand(linear_x=parameters.blind_final_speed)
    if not detected or not tag_valid:
        return PlanarCommand()
    if state in {TAG_LOST, TOO_CLOSE, STABILIZING, ALIGNED}:
        return PlanarCommand()

    valid_pairs = {
        COARSE_TRACK: {TURN_LEFT, TURN_RIGHT, APPROACH},
        NEAR_ALIGN: {APPROACH},
        RECENTER: {TURN_LEFT, TURN_RIGHT},
        FINAL_YAW_ALIGN: {FINE_ALIGN_LEFT, FINE_ALIGN_RIGHT},
        FINAL_APPROACH: {FINAL_APPROACH},
    }
    if state not in valid_pairs.get(mode, set()):
        return PlanarCommand()

    target_bearing = atan2(measurement.target_y, measurement.target_x)
    if state in {TURN_LEFT, TURN_RIGHT}:
        angular_limit = (
            parameters.near_max_angular_speed
            if mode == RECENTER
            else parameters.max_raw_angular_speed
        )
        angular = clamp(parameters.angular_gain * target_bearing, angular_limit)
        if state == TURN_LEFT and angular > 0.0:
            return PlanarCommand(angular_z=angular)
        if state == TURN_RIGHT and angular < 0.0:
            return PlanarCommand(angular_z=angular)
        return PlanarCommand()

    if state == APPROACH:
        if measurement.target_x <= 0.0:
            return PlanarCommand()
        linear = min(
            parameters.linear_gain * hypot(measurement.target_x, measurement.target_y),
            parameters.max_raw_linear_speed,
        )
        angular_limit = (
            parameters.near_max_angular_speed
            if mode == NEAR_ALIGN
            else parameters.max_raw_angular_speed
        )
        angular = clamp(parameters.angular_gain * target_bearing, angular_limit)
        return PlanarCommand(linear, angular)

    if state in {FINE_ALIGN_LEFT, FINE_ALIGN_RIGHT}:
        angular = clamp(
            parameters.angular_gain * measurement.target_yaw,
            parameters.max_final_angular_speed,
        )
        if state == FINE_ALIGN_LEFT and angular > 0.0:
            return PlanarCommand(angular_z=angular)
        if state == FINE_ALIGN_RIGHT and angular < 0.0:
            return PlanarCommand(angular_z=angular)
        return PlanarCommand()

    if state == FINAL_APPROACH:
        if measurement.target_x <= 0.0:
            return PlanarCommand()
        linear = min(
            parameters.linear_gain * measurement.target_x,
            parameters.max_final_linear_speed,
        )
        angular = clamp(
            parameters.angular_gain * measurement.target_yaw
            + parameters.lateral_gain * measurement.target_y,
            parameters.max_final_angular_speed,
        )
        return PlanarCommand(linear, angular)
    return PlanarCommand()
