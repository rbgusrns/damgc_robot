"""Validation, TF application, and metrics for Follower base-frame tag poses."""

from dataclasses import dataclass
from math import atan2, isfinite
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose

from follower_supply_perception.approach_logic import normalize_quaternion


@dataclass(frozen=True)
class BaseMetrics:
    """Planar Follower-tag measurements expressed in ``base_link``."""

    forward_distance: float
    lateral_error: float
    bearing: float


def is_fresh_timestamp(
    stamp_sec: int,
    stamp_nanosec: int,
    now_nanoseconds: int,
    timeout_seconds: float,
) -> bool:
    """Return whether a non-zero source pose stamp is current."""
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
    """Compute forward, lateral, and bearing from a base-frame point."""
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
    """Apply a validated TF transform while retaining the source timestamp."""
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
