"""ROS-independent camera projection helpers for survivor positions."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class CameraIntrinsics:
    """Rectified pinhole intrinsics and their image dimensions."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class CameraPoint:
    """One metric point in a camera optical frame."""

    x: float
    y: float
    z: float


def get_rectified_intrinsics(
    projection: Sequence[float],
    camera_width: int,
    camera_height: int,
    image_width: int,
    image_height: int,
) -> Optional[CameraIntrinsics]:
    """Validate CameraInfo.P for a rectified image and return intrinsics."""
    if (
        len(projection) != 12
        or camera_width <= 0
        or camera_height <= 0
        or image_width <= 0
        or image_height <= 0
        or camera_width != image_width
        or camera_height != image_height
    ):
        return None
    fx = float(projection[0])
    fy = float(projection[5])
    cx = float(projection[2])
    cy = float(projection[6])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        return None
    if fx <= 0.0 or fy <= 0.0:
        return None
    return CameraIntrinsics(fx, fy, cx, cy, image_width, image_height)


def roi_center_pixel(
    roi: Tuple[int, int, int, int],
) -> Optional[Tuple[float, float]]:
    """Return the center of half-open ROI bounds in pixel coordinates."""
    left, top, right, bottom = roi
    if right <= left or bottom <= top:
        return None
    return (left + right - 1) / 2.0, (top + bottom - 1) / 2.0


def deproject_pixel_to_camera_xyz(
    u: float,
    v: float,
    z: float,
    intrinsics: CameraIntrinsics,
) -> Optional[CameraPoint]:
    """Deproject one rectified pixel and median depth into optical XYZ."""
    if not all(math.isfinite(value) for value in (u, v, z)) or z <= 0.0:
        return None
    if not 0.0 <= u < intrinsics.width or not 0.0 <= v < intrinsics.height:
        return None
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    return CameraPoint(x, y, z)
