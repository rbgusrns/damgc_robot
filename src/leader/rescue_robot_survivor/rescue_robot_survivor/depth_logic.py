"""ROS-independent RGB-aligned depth distance helpers."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    """Distance estimate and the number of valid source pixels."""

    distance_m: Optional[float]
    valid_pixel_count: int


def calculate_depth_roi(
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    width_ratio: float,
    height_ratio: float,
    image_width: int,
    image_height: int,
) -> Optional[Tuple[int, int, int, int]]:
    """Return a clamped center ROI as half-open pixel bounds."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if not 0.0 < width_ratio <= 1.0 or not 0.0 < height_ratio <= 1.0:
        raise ValueError("ROI ratios must be greater than 0 and at most 1")

    left = max(0, min(image_width, int(round(x1))))
    top = max(0, min(image_height, int(round(y1))))
    right = max(0, min(image_width, int(round(x2))))
    bottom = max(0, min(image_height, int(round(y2))))
    if right <= left or bottom <= top:
        return None

    roi_width = max(1, int(round((right - left) * width_ratio)))
    roi_height = max(1, int(round((bottom - top) * height_ratio)))
    center_x = (left + right) / 2.0
    center_y = (top + bottom) / 2.0
    roi_left = int(round(center_x - roi_width / 2.0))
    roi_top = int(round(center_y - roi_height / 2.0))
    roi_right = roi_left + roi_width
    roi_bottom = roi_top + roi_height

    roi_left = max(0, min(image_width, roi_left))
    roi_top = max(0, min(image_height, roi_top))
    roi_right = max(0, min(image_width, roi_right))
    roi_bottom = max(0, min(image_height, roi_bottom))
    if roi_right <= roi_left or roi_bottom <= roi_top:
        return None
    return roi_left, roi_top, roi_right, roi_bottom


def convert_depth_to_meters(
    depth_roi: np.ndarray, encoding: str, depth_scale_m_per_unit: float
) -> Optional[np.ndarray]:
    """Convert supported ROS depth encodings to float meters."""
    if depth_scale_m_per_unit <= 0.0:
        raise ValueError("depth_scale_m_per_unit must be positive")
    if depth_roi.ndim != 2:
        raise ValueError("depth ROI must be a single-channel array")
    normalized = encoding.upper()
    values = depth_roi.astype(np.float32, copy=False)
    if normalized in ("16UC1", "MONO16"):
        return values * depth_scale_m_per_unit
    if normalized == "32FC1":
        return values
    return None


def extract_valid_depth_values(
    depth_roi: np.ndarray,
    encoding: str,
    depth_scale_m_per_unit: float,
    min_depth_m: float,
    max_depth_m: float,
) -> Optional[np.ndarray]:
    """Convert and retain finite depth values inside the configured range."""
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth range is invalid")
    depth_m = convert_depth_to_meters(
        depth_roi, encoding, depth_scale_m_per_unit
    )
    if depth_m is None:
        return None
    valid = depth_m[np.isfinite(depth_m)]
    return valid[(valid >= min_depth_m) & (valid <= max_depth_m)]


def median_depth(
    valid_depth_m: np.ndarray, min_valid_depth_pixels: int
) -> DepthEstimate:
    """Return the median distance if enough valid samples are available."""
    if min_valid_depth_pixels <= 0:
        raise ValueError("min_valid_depth_pixels must be positive")
    if valid_depth_m.size < min_valid_depth_pixels:
        return DepthEstimate(None, int(valid_depth_m.size))
    distance_m = float(np.median(valid_depth_m))
    if not math.isfinite(distance_m):
        return DepthEstimate(None, int(valid_depth_m.size))
    return DepthEstimate(distance_m, int(valid_depth_m.size))


def estimate_person_distance(
    depth_image: np.ndarray,
    encoding: str,
    bbox: Tuple[int, int, int, int],
    width_ratio: float,
    height_ratio: float,
    depth_scale_m_per_unit: float,
    min_depth_m: float,
    max_depth_m: float,
    min_valid_depth_pixels: int,
) -> Tuple[DepthEstimate, Optional[Tuple[int, int, int, int]]]:
    """Estimate one person's distance from a center ROI."""
    if depth_image.ndim != 2:
        raise ValueError("depth image must be single-channel")
    roi = calculate_depth_roi(
        *bbox,
        width_ratio,
        height_ratio,
        depth_image.shape[1],
        depth_image.shape[0],
    )
    if roi is None:
        return DepthEstimate(None, 0), None
    left, top, right, bottom = roi
    valid = extract_valid_depth_values(
        depth_image[top:bottom, left:right],
        encoding,
        depth_scale_m_per_unit,
        min_depth_m,
        max_depth_m,
    )
    if valid is None:
        return DepthEstimate(None, 0), roi
    return median_depth(valid, min_valid_depth_pixels), roi
