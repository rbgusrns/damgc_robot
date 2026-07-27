"""Pure Python filtering, selection, and state logic for AprilTag approach.

The camera optical frame is assumed: x points right, y points down, and z points
forward.  This module deliberately has no ROS imports so its behavior can be unit
tested without a running ROS graph.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import atan2, isclose, isfinite, radians, sqrt
from statistics import median
from typing import Deque, Iterable, Optional, Sequence, Tuple


class ApproachState(str, Enum):
    """States published by the AprilTag approach node."""

    TAG_LOST = "TAG_LOST"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    APPROACH = "APPROACH"
    TOO_CLOSE = "TOO_CLOSE"
    FINE_ALIGN_LEFT = "FINE_ALIGN_LEFT"
    FINE_ALIGN_RIGHT = "FINE_ALIGN_RIGHT"
    STABILIZING = "STABILIZING"
    ALIGNED = "ALIGNED"


@dataclass(frozen=True)
class ApproachThresholds:
    """Thresholds used by :class:`ApproachStateMachine`."""

    target_distance: float
    distance_tolerance: float
    lateral_tolerance: float
    angle_tolerance_deg: float
    stable_time: float

    def validate(self) -> None:
        """Raise ``ValueError`` when a threshold cannot produce valid behavior."""
        values = (
            self.target_distance,
            self.distance_tolerance,
            self.lateral_tolerance,
            self.angle_tolerance_deg,
            self.stable_time,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("All approach thresholds must be finite")
        if self.target_distance <= 0.0:
            raise ValueError("target_distance must be greater than zero")
        if self.distance_tolerance < 0.0:
            raise ValueError("distance_tolerance must not be negative")
        if self.lateral_tolerance < 0.0:
            raise ValueError("lateral_tolerance must not be negative")
        if self.angle_tolerance_deg < 0.0:
            raise ValueError("angle_tolerance_deg must not be negative")
        if self.stable_time < 0.0:
            raise ValueError("stable_time must not be negative")

    @property
    def angle_tolerance_rad(self) -> float:
        """Return the configured angle tolerance in radians."""
        return radians(self.angle_tolerance_deg)


@dataclass(frozen=True)
class TagObservation:
    """A validated, unfiltered TF observation for one tag."""

    tag_id: int
    x: float
    y: float
    z: float
    quaternion: Tuple[float, float, float, float]
    stamp_nanoseconds: int

    @property
    def straight_distance(self) -> float:
        """Return the Euclidean distance from the camera to the tag."""
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


@dataclass(frozen=True)
class RelativeMeasurement:
    """Filtered translation and derived approach metrics."""

    x: float
    y: float
    z: float
    distance: float
    lateral_error: float
    straight_distance: float
    angle: float


def is_valid_translation(x: float, y: float, z: float) -> bool:
    """Return whether a translation is finite and lies in front of the camera."""
    return all(isfinite(value) for value in (x, y, z)) and z > 0.0


def normalize_quaternion(
    quaternion: Sequence[float],
) -> Optional[Tuple[float, float, float, float]]:
    """Return a normalized finite quaternion, or ``None`` for invalid input."""
    if len(quaternion) != 4 or not all(isfinite(value) for value in quaternion):
        return None
    norm = sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        return None
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def compute_measurement(x: float, y: float, z: float) -> RelativeMeasurement:
    """Calculate distances and horizontal angle from a filtered translation."""
    if not is_valid_translation(x, y, z):
        raise ValueError("Translation must be finite and z must be greater than zero")
    return RelativeMeasurement(
        x=x,
        y=y,
        z=z,
        distance=z,
        lateral_error=x,
        straight_distance=sqrt(x * x + y * y + z * z),
        angle=atan2(x, z),
    )


def _above_boundary(value: float, boundary: float) -> bool:
    """Compare above a boundary without excluding its rounded float equivalent."""
    return value > boundary and not isclose(
        value, boundary, rel_tol=1.0e-9, abs_tol=1.0e-12
    )


def _below_boundary(value: float, boundary: float) -> bool:
    """Compare below a boundary without excluding its rounded float equivalent."""
    return value < boundary and not isclose(
        value, boundary, rel_tol=1.0e-9, abs_tol=1.0e-12
    )


class MedianTranslationFilter:
    """Median filter for distinct, timestamped x/y/z observations."""

    def __init__(self, window_size: int) -> None:
        if window_size < 1:
            raise ValueError("filter_window must be at least 1")
        self._samples: Deque[Tuple[float, float, float]] = deque(maxlen=window_size)
        self._last_stamp_nanoseconds: Optional[int] = None

    def reset(self) -> None:
        """Discard all samples, such as after loss or a selected-tag change."""
        self._samples.clear()
        self._last_stamp_nanoseconds = None

    def add(
        self, x: float, y: float, z: float, stamp_nanoseconds: int
    ) -> RelativeMeasurement:
        """Add a new timestamped sample and return the current filtered result.

        A timer may read the same buffered TF more than once.  Such duplicate stamps
        are not added again because they would bias the median.
        """
        if not is_valid_translation(x, y, z):
            raise ValueError("Translation must be finite and z must be greater than zero")
        if stamp_nanoseconds != self._last_stamp_nanoseconds:
            self._samples.append((x, y, z))
            self._last_stamp_nanoseconds = stamp_nanoseconds
        if not self._samples:
            raise RuntimeError("Median filter has no samples")
        filtered_x = float(median(sample[0] for sample in self._samples))
        filtered_y = float(median(sample[1] for sample in self._samples))
        filtered_z = float(median(sample[2] for sample in self._samples))
        return compute_measurement(filtered_x, filtered_y, filtered_z)


def select_observation(
    observations: Iterable[TagObservation],
    allowed_tag_ids: Sequence[int],
    selection_mode: str,
) -> Optional[TagObservation]:
    """Select a visible tag by configured priority or Euclidean distance."""
    by_id = {observation.tag_id: observation for observation in observations}
    ordered = [by_id[tag_id] for tag_id in allowed_tag_ids if tag_id in by_id]
    if not ordered:
        return None
    if selection_mode == "priority":
        return ordered[0]
    if selection_mode == "nearest":
        # ``min`` preserves list order for equal keys, giving a deterministic tie.
        return min(ordered, key=lambda observation: observation.straight_distance)
    raise ValueError("selection_mode must be 'priority' or 'nearest'")


class ApproachStateMachine:
    """Stateful alignment evaluator with continuous-time stabilization."""

    def __init__(self, thresholds: ApproachThresholds) -> None:
        thresholds.validate()
        self._thresholds = thresholds
        self._stable_since: Optional[float] = None
        self._active_tag_id: Optional[int] = None

    def reset(self) -> None:
        """Reset selected-tag and stabilization history."""
        self._stable_since = None
        self._active_tag_id = None

    def update(
        self,
        measurement: Optional[RelativeMeasurement],
        now_seconds: float,
        tag_id: Optional[int],
    ) -> ApproachState:
        """Evaluate one measurement according to the documented priority order."""
        if measurement is None or tag_id is None:
            self.reset()
            return ApproachState.TAG_LOST
        if not isfinite(now_seconds):
            raise ValueError("now_seconds must be finite")

        if tag_id != self._active_tag_id:
            self._stable_since = None
            self._active_tag_id = tag_id

        angle_tolerance = self._thresholds.angle_tolerance_rad
        if _below_boundary(measurement.angle, -angle_tolerance):
            return self._not_stable(ApproachState.TURN_LEFT)
        if _above_boundary(measurement.angle, angle_tolerance):
            return self._not_stable(ApproachState.TURN_RIGHT)
        if _above_boundary(
            measurement.distance,
            self._thresholds.target_distance + self._thresholds.distance_tolerance
        ):
            return self._not_stable(ApproachState.APPROACH)
        if _below_boundary(
            measurement.distance,
            self._thresholds.target_distance - self._thresholds.distance_tolerance
        ):
            return self._not_stable(ApproachState.TOO_CLOSE)
        if _below_boundary(
            measurement.lateral_error, -self._thresholds.lateral_tolerance
        ):
            return self._not_stable(ApproachState.FINE_ALIGN_LEFT)
        if _above_boundary(
            measurement.lateral_error, self._thresholds.lateral_tolerance
        ):
            return self._not_stable(ApproachState.FINE_ALIGN_RIGHT)

        if self._stable_since is None or now_seconds < self._stable_since:
            self._stable_since = now_seconds
        if now_seconds - self._stable_since >= self._thresholds.stable_time:
            return ApproachState.ALIGNED
        return ApproachState.STABILIZING

    def _not_stable(self, state: ApproachState) -> ApproachState:
        """Clear stabilization history and return a non-aligned state."""
        self._stable_since = None
        return state
