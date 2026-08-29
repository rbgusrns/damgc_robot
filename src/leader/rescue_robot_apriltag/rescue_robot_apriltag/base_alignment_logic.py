"""ROS-independent alignment decisions for base-frame AprilTag samples."""

from dataclasses import dataclass
from math import isclose, isfinite, radians
from typing import Optional

from rescue_robot_apriltag.approach_logic import ApproachState


@dataclass(frozen=True)
class BaseAlignmentThresholds:
    """Provisional thresholds for base-frame software validation."""

    target_forward: float
    forward_tolerance: float
    lateral_tolerance: float
    bearing_tolerance_deg: float
    stable_time: float
    sample_timeout: float

    def validate(self) -> None:
        """Raise ``ValueError`` when the thresholds cannot define valid states."""
        values = (
            self.target_forward,
            self.forward_tolerance,
            self.lateral_tolerance,
            self.bearing_tolerance_deg,
            self.stable_time,
            self.sample_timeout,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("All base alignment thresholds must be finite")
        if self.target_forward <= 0.0:
            raise ValueError("target_forward must be greater than zero")
        if self.forward_tolerance < 0.0:
            raise ValueError("forward_tolerance must not be negative")
        if self.target_forward <= self.forward_tolerance:
            raise ValueError("target_forward must be greater than forward_tolerance")
        if self.lateral_tolerance < 0.0:
            raise ValueError("lateral_tolerance must not be negative")
        if self.bearing_tolerance_deg < 0.0:
            raise ValueError("bearing_tolerance_deg must not be negative")
        if self.stable_time < 0.0:
            raise ValueError("stable_time must not be negative")
        if self.sample_timeout < 0.0:
            raise ValueError("sample_timeout must not be negative")

    @property
    def bearing_tolerance_rad(self) -> float:
        """Return the configured bearing tolerance in radians."""
        return radians(self.bearing_tolerance_deg)


@dataclass(frozen=True)
class BaseAlignmentMeasurement:
    """One timestamped planar measurement expressed in ``base_link``."""

    forward_distance: float
    lateral_error: float
    bearing: float
    stamp_seconds: float


def _is_above(value: float, boundary: float) -> bool:
    """Test an upper boundary while accepting rounded boundary equivalents."""
    return value > boundary and not isclose(
        value, boundary, rel_tol=1.0e-9, abs_tol=1.0e-12
    )


def _is_below(value: float, boundary: float) -> bool:
    """Test a lower boundary while accepting rounded boundary equivalents."""
    return value < boundary and not isclose(
        value, boundary, rel_tol=1.0e-9, abs_tol=1.0e-12
    )


class BaseAlignmentStateMachine:
    """Evaluate one coherent base-frame sample and track continuous stability."""

    def __init__(self, thresholds: BaseAlignmentThresholds) -> None:
        thresholds.validate()
        self._thresholds = thresholds
        self._stable_since: Optional[float] = None
        self._active_tag_id: Optional[int] = None

    def reset(self) -> None:
        """Clear tag identity and stability history."""
        self._stable_since = None
        self._active_tag_id = None

    def update(
        self,
        measurement: Optional[BaseAlignmentMeasurement],
        now_seconds: float,
        tag_id: Optional[int],
    ) -> ApproachState:
        """Return the highest-priority base alignment state for this sample."""
        if not isfinite(now_seconds):
            raise ValueError("now_seconds must be finite")
        if not self._is_valid_sample(measurement, now_seconds, tag_id):
            self.reset()
            return ApproachState.TAG_LOST
        assert measurement is not None
        assert tag_id is not None

        if tag_id != self._active_tag_id:
            self._stable_since = None
            self._active_tag_id = tag_id

        bearing_tolerance = self._thresholds.bearing_tolerance_rad
        if _is_above(measurement.bearing, bearing_tolerance):
            return self._leave_stable_region(ApproachState.TURN_LEFT)
        if _is_below(measurement.bearing, -bearing_tolerance):
            return self._leave_stable_region(ApproachState.TURN_RIGHT)
        if _is_above(
            measurement.forward_distance,
            self._thresholds.target_forward
            + self._thresholds.forward_tolerance,
        ):
            return self._leave_stable_region(ApproachState.APPROACH)
        if _is_below(
            measurement.forward_distance,
            self._thresholds.target_forward
            - self._thresholds.forward_tolerance,
        ):
            return self._leave_stable_region(ApproachState.TOO_CLOSE)
        if _is_above(
            measurement.lateral_error, self._thresholds.lateral_tolerance
        ):
            return self._leave_stable_region(ApproachState.FINE_ALIGN_LEFT)
        if _is_below(
            measurement.lateral_error, -self._thresholds.lateral_tolerance
        ):
            return self._leave_stable_region(ApproachState.FINE_ALIGN_RIGHT)

        if self._stable_since is None or now_seconds < self._stable_since:
            self._stable_since = now_seconds
        if now_seconds - self._stable_since >= self._thresholds.stable_time:
            return ApproachState.ALIGNED
        return ApproachState.STABILIZING

    def _is_valid_sample(
        self,
        measurement: Optional[BaseAlignmentMeasurement],
        now_seconds: float,
        tag_id: Optional[int],
    ) -> bool:
        """Return whether identity, metrics, and source timestamp are valid."""
        if measurement is None or tag_id is None or tag_id < 0:
            return False
        values = (
            measurement.forward_distance,
            measurement.lateral_error,
            measurement.bearing,
            measurement.stamp_seconds,
        )
        if not all(isfinite(value) for value in values):
            return False
        if measurement.forward_distance <= 0.0 or measurement.stamp_seconds <= 0.0:
            return False
        age = now_seconds - measurement.stamp_seconds
        return 0.0 <= age <= self._thresholds.sample_timeout + 1.0e-9

    def _leave_stable_region(self, state: ApproachState) -> ApproachState:
        """Reset continuous stability and return a non-aligned state."""
        self._stable_since = None
        return state
