"""Validation, TF application, and metrics for Leader base-frame tag poses."""

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin
from statistics import median
from typing import Deque, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose

from rescue_robot_apriltag.approach_logic import normalize_quaternion


@dataclass(frozen=True)
class BaseMetrics:
    """Planar Leader-tag measurements expressed in ``base_link``."""

    forward_distance: float
    lateral_error: float
    bearing: float


@dataclass(frozen=True)
class TargetGeometry:
    """Planar tag-normal targets expressed in the current ``base_link``."""

    normal_x: float
    normal_y: float
    target_yaw: float
    prealign_x: float
    prealign_y: float
    final_x: float
    final_y: float
    final_position_error: float
    final_yaw_error: float


def normalize_angle(angle: float) -> float:
    """Wrap one finite angle to ``[-pi, pi]``."""
    if not isfinite(angle):
        raise ValueError("Angle must be finite")
    return atan2(sin(angle), cos(angle))


def rotate_tag_z_to_base_xy(
    quaternion: Sequence[float], projection_epsilon: float = 1.0e-6
) -> Tuple[float, float]:
    """Return the normalized base-XY projection of the tag's inward +Z axis.

    ``apriltag_ros`` PnP uses the AprilTag convention in which tag +Z points
    through the printed face into the tag.  The supplied quaternion already
    represents the tag frame in ``base_link``; extracting Euler yaw from it
    would use the wrong pair of axes for a generally tilted tag.
    """
    normalized = normalize_quaternion(quaternion)
    if normalized is None:
        raise ValueError("Tag quaternion must be finite and non-zero")
    if not isfinite(projection_epsilon) or projection_epsilon <= 0.0:
        raise ValueError("projection_epsilon must be finite and positive")
    qx, qy, qz, qw = normalized
    # Third column of the quaternion rotation matrix: R(q) * [0, 0, 1].
    normal_x = 2.0 * (qx * qz + qw * qy)
    normal_y = 2.0 * (qy * qz - qw * qx)
    projection_norm = hypot(normal_x, normal_y)
    if not isfinite(projection_norm) or projection_norm <= projection_epsilon:
        raise ValueError("Tag normal has no usable base-XY projection")
    return normal_x / projection_norm, normal_y / projection_norm


class PlanarNormalMedianFilter:
    """Median-filter unique timestamped unit normal vectors."""

    def __init__(self, window_size: int) -> None:
        if window_size < 1:
            raise ValueError("filter_window must be at least 1")
        self._samples: Deque[Tuple[float, float]] = deque(maxlen=window_size)
        self._last_stamp_nanoseconds: Optional[int] = None

    def reset(self) -> None:
        """Discard normal samples and duplicate-timestamp history."""
        self._samples.clear()
        self._last_stamp_nanoseconds = None

    def add(
        self, normal_x: float, normal_y: float, stamp_nanoseconds: int
    ) -> Tuple[float, float]:
        """Insert one direction and return its normalized component medians."""
        norm = hypot(normal_x, normal_y)
        if not all(isfinite(value) for value in (normal_x, normal_y, norm)):
            raise ValueError("Normal components must be finite")
        if norm <= 1.0e-6:
            raise ValueError("Normal projection must be non-zero")
        if stamp_nanoseconds != self._last_stamp_nanoseconds:
            self._samples.append((normal_x / norm, normal_y / norm))
            self._last_stamp_nanoseconds = stamp_nanoseconds
        filtered_x = float(median(sample[0] for sample in self._samples))
        filtered_y = float(median(sample[1] for sample in self._samples))
        filtered_norm = hypot(filtered_x, filtered_y)
        if filtered_norm <= 1.0e-6:
            raise ValueError("Filtered tag normal is ambiguous")
        return filtered_x / filtered_norm, filtered_y / filtered_norm


def compute_target_geometry(
    tag_x: float,
    tag_y: float,
    normal_x: float,
    normal_y: float,
    pre_align_distance: float,
    final_target_distance: float,
) -> TargetGeometry:
    """Build robot target poses on the printed/front side of an AprilTag."""
    values = (
        tag_x,
        tag_y,
        normal_x,
        normal_y,
        pre_align_distance,
        final_target_distance,
    )
    if not all(isfinite(value) for value in values):
        raise ValueError("Target geometry inputs must be finite")
    if not pre_align_distance > final_target_distance > 0.0:
        raise ValueError(
            "pre_align_distance must be greater than final_target_distance > 0"
        )
    normal_norm = hypot(normal_x, normal_y)
    tag_range = hypot(tag_x, tag_y)
    if normal_norm <= 1.0e-6 or tag_range <= 1.0e-6:
        raise ValueError("Tag position and projected normal must be non-zero")
    inward_x = normal_x / normal_norm
    inward_y = normal_y / normal_norm
    # A front-facing detection has inward +Z pointing generally from robot to tag.
    if inward_x * tag_x + inward_y * tag_y <= 0.0:
        raise ValueError("Tag +Z does not point away from the robot into the tag")

    prealign_x = tag_x - pre_align_distance * inward_x
    prealign_y = tag_y - pre_align_distance * inward_y
    final_x = tag_x - final_target_distance * inward_x
    final_y = tag_y - final_target_distance * inward_y
    target_yaw = atan2(inward_y, inward_x)
    final_yaw_error = normalize_angle(target_yaw)
    return TargetGeometry(
        normal_x=inward_x,
        normal_y=inward_y,
        target_yaw=target_yaw,
        prealign_x=prealign_x,
        prealign_y=prealign_y,
        final_x=final_x,
        final_y=final_y,
        final_position_error=hypot(final_x, final_y),
        final_yaw_error=final_yaw_error,
    )


def is_fresh_timestamp(
    stamp_sec: int,
    stamp_nanosec: int,
    now_nanoseconds: int,
    timeout_seconds: float,
) -> bool:
    """Return whether a non-zero pose stamp is current under the camera timeout."""
    if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000:
        return False
    if (
        now_nanoseconds < 0
        or not isfinite(timeout_seconds)
        or timeout_seconds < 0.0
    ):
        return False
    stamp_nanoseconds = stamp_sec * 1_000_000_000 + stamp_nanosec
    if stamp_nanoseconds <= 0 or stamp_nanoseconds > now_nanoseconds:
        return False
    age_nanoseconds = now_nanoseconds - stamp_nanoseconds
    return age_nanoseconds <= int(timeout_seconds * 1_000_000_000)


def compute_base_metrics(x: float, y: float) -> BaseMetrics:
    """Compute forward, lateral, and bearing values from a base-frame point."""
    if not all(isfinite(value) for value in (x, y)):
        raise ValueError("Base-frame x and y must be finite")
    return BaseMetrics(
        forward_distance=x,
        lateral_error=y,
        bearing=atan2(y, x),
    )


def transform_pose_preserving_stamp(
    pose: Optional[PoseStamped],
    transform: TransformStamped,
    target_frame: str,
) -> Optional[PoseStamped]:
    """Apply a validated TF transform while retaining the input pose timestamp."""
    if pose is None or not pose.header.frame_id or not target_frame:
        return None

    position = pose.pose.position
    if not all(isfinite(value) for value in (position.x, position.y, position.z)):
        return None
    input_quaternion = normalize_quaternion(
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
    )
    if input_quaternion is None:
        return None

    translation = transform.transform.translation
    if not all(
        isfinite(value) for value in (translation.x, translation.y, translation.z)
    ):
        return None
    transform_quaternion = normalize_quaternion(
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
    )
    if transform_quaternion is None:
        return None

    normalized_pose = Pose()
    normalized_pose.position = position
    (
        normalized_pose.orientation.x,
        normalized_pose.orientation.y,
        normalized_pose.orientation.z,
        normalized_pose.orientation.w,
    ) = input_quaternion

    normalized_transform = TransformStamped()
    normalized_transform.transform.translation = translation
    (
        normalized_transform.transform.rotation.x,
        normalized_transform.transform.rotation.y,
        normalized_transform.transform.rotation.z,
        normalized_transform.transform.rotation.w,
    ) = transform_quaternion

    transformed_pose = do_transform_pose(normalized_pose, normalized_transform)
    transformed_position = transformed_pose.position
    if not all(
        isfinite(value)
        for value in (
            transformed_position.x,
            transformed_position.y,
            transformed_position.z,
        )
    ):
        return None
    output_quaternion = normalize_quaternion(
        (
            transformed_pose.orientation.x,
            transformed_pose.orientation.y,
            transformed_pose.orientation.z,
            transformed_pose.orientation.w,
        )
    )
    if output_quaternion is None:
        return None

    result = PoseStamped()
    result.header.frame_id = target_frame
    result.header.stamp = pose.header.stamp
    result.pose.position = transformed_position
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = output_quaternion
    return result
