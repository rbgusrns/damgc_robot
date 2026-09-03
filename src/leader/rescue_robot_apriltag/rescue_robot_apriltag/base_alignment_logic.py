"""ROS-independent alignment decisions for base-frame AprilTag samples."""

from dataclasses import dataclass
from math import atan2, hypot, isclose, isfinite, radians
from typing import Optional

from rescue_robot_apriltag.approach_logic import ApproachState


@dataclass(frozen=True)
class BaseAlignmentThresholds:
    """Thresholds for two-stage tag-normal alignment."""

    pre_align_position_tolerance: float
    pre_align_heading_tolerance_deg: float
    final_position_tolerance: float
    final_yaw_tolerance_deg: float
    stable_time: float
    sample_timeout: float

    def validate(self) -> None:
        """Raise ``ValueError`` when the thresholds cannot define valid states."""
        values = (
            self.pre_align_position_tolerance,
            self.pre_align_heading_tolerance_deg,
            self.final_position_tolerance,
            self.final_yaw_tolerance_deg,
            self.stable_time,
            self.sample_timeout,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("All base alignment thresholds must be finite")
        if self.pre_align_position_tolerance <= 0.0:
            raise ValueError("pre_align_position_tolerance must be positive")
        if self.pre_align_heading_tolerance_deg < 0.0:
            raise ValueError("pre_align_heading_tolerance_deg must not be negative")
        if self.final_position_tolerance <= 0.0:
            raise ValueError("final_position_tolerance must be positive")
        if self.final_yaw_tolerance_deg < 0.0:
            raise ValueError("final_yaw_tolerance_deg must not be negative")
        if self.stable_time < 0.0:
            raise ValueError("stable_time must not be negative")
        if self.sample_timeout < 0.0:
            raise ValueError("sample_timeout must not be negative")

    @property
    def pre_align_heading_tolerance_rad(self) -> float:
        """Return the pre-align heading tolerance in radians."""
        return radians(self.pre_align_heading_tolerance_deg)

    @property
    def final_yaw_tolerance_rad(self) -> float:
        """Return the final tag-normal yaw tolerance in radians."""
        return radians(self.final_yaw_tolerance_deg)


@dataclass(frozen=True)
class BaseAlignmentMeasurement:
    """One timestamped pair of tag-normal target poses in ``base_link``."""

    prealign_x: float
    prealign_y: float
    final_x: float
    final_y: float
    final_yaw_error: float
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
        self._final_phase = False

    def reset(self) -> None:
        """Clear tag identity and stability history."""
        self._stable_since = None
        self._active_tag_id = None
        self._final_phase = False

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
            self._final_phase = False

        if not self._final_phase:
            prealign_error = hypot(measurement.prealign_x, measurement.prealign_y)
            if prealign_error <= self._thresholds.pre_align_position_tolerance:
                self._final_phase = True
            elif measurement.prealign_x <= 0.0:
                # Never reverse to recover a missed pre-align point.  Continue only
                # when the final point is still safely in front of the robot.
                if measurement.final_x > 0.0:
                    self._final_phase = True
                else:
                    return self._leave_stable_region(ApproachState.TOO_CLOSE)
            else:
                bearing = atan2(measurement.prealign_y, measurement.prealign_x)
                heading_tolerance = (
                    self._thresholds.pre_align_heading_tolerance_rad
                )
                if _is_above(bearing, heading_tolerance):
                    return self._leave_stable_region(ApproachState.TURN_LEFT)
                if _is_below(bearing, -heading_tolerance):
                    return self._leave_stable_region(ApproachState.TURN_RIGHT)
                return self._leave_stable_region(ApproachState.APPROACH)

        yaw_tolerance = self._thresholds.final_yaw_tolerance_rad
        if _is_above(measurement.final_yaw_error, yaw_tolerance):
            return self._leave_stable_region(ApproachState.FINE_ALIGN_LEFT)
        if _is_below(measurement.final_yaw_error, -yaw_tolerance):
            return self._leave_stable_region(ApproachState.FINE_ALIGN_RIGHT)

        final_error = hypot(measurement.final_x, measurement.final_y)
        if _is_above(final_error, self._thresholds.final_position_tolerance):
            if measurement.final_x <= 0.0:
                return self._leave_stable_region(ApproachState.TOO_CLOSE)
            return self._leave_stable_region(ApproachState.FINAL_APPROACH)

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
            measurement.prealign_x,
            measurement.prealign_y,
            measurement.final_x,
            measurement.final_y,
            measurement.final_yaw_error,
            measurement.stamp_seconds,
        )
        if not all(isfinite(value) for value in values):
            return False
        if measurement.stamp_seconds <= 0.0:
            return False
        age = now_seconds - measurement.stamp_seconds
        return 0.0 <= age <= self._thresholds.sample_timeout + 1.0e-9

    def _leave_stable_region(self, state: ApproachState) -> ApproachState:
        """Reset continuous stability and return a non-aligned state."""
        self._stable_since = None
        return state
