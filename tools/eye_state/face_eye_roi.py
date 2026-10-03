#!/usr/bin/env python3
"""YuNet face detection and landmark-based left/right eye ROI extraction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


BBox = Tuple[int, int, int, int]
Point = Tuple[float, float]


@dataclass(frozen=True)
class ExtractorConfig:
    """Geometric and quality thresholds for eye ROI extraction."""

    face_confidence_threshold: float = 0.60
    roi_side_inter_eye_ratio: float = 0.85
    min_eye_width: int = 24
    min_eye_height: int = 24
    severe_clipping_fraction: float = 0.25
    input_size: int = 320

    def __post_init__(self) -> None:
        if not 0.0 <= self.face_confidence_threshold <= 1.0:
            raise ValueError("face_confidence_threshold must be in [0, 1]")
        if self.roi_side_inter_eye_ratio <= 0.0:
            raise ValueError("roi_side_inter_eye_ratio must be positive")
        if self.min_eye_width < 1 or self.min_eye_height < 1:
            raise ValueError("minimum eye ROI dimensions must be positive")
        if not 0.0 <= self.severe_clipping_fraction < 1.0:
            raise ValueError("severe_clipping_fraction must be in [0, 1)")
        if self.input_size < 1:
            raise ValueError("input_size must be positive")


@dataclass
class EyeROI:
    """One subject-relative eye and its crop status."""

    side: str
    center: Optional[Point]
    bbox: Optional[BBox]
    image: Optional[np.ndarray]
    valid: bool
    reason: str = ""
    clipped_fraction: float = 0.0


@dataclass
class FaceEyeROI:
    """A detected face and the eye crops derived from its landmarks."""

    bbox: BBox
    confidence: float
    right_eye: EyeROI
    left_eye: EyeROI
    landmarks: Tuple[Optional[Point], ...]


def _invalid_eye(side: str, center: Optional[Point], reason: str) -> EyeROI:
    return EyeROI(side, center, None, None, False, reason)


def _as_point(x: float, y: float, width: int, height: int) -> Optional[Point]:
    if not np.isfinite(x) or not np.isfinite(y):
        return None
    if x < 0 or y < 0 or x >= width or y >= height:
        return None
    return float(x), float(y)


def _extract_eye(
    frame: np.ndarray,
    side: str,
    center: Optional[Point],
    side_px: int,
    config: ExtractorConfig,
) -> EyeROI:
    height, width = frame.shape[:2]
    if center is None:
        return _invalid_eye(side, None, "invalid_landmark")

    cx, cy = center
    half = side_px / 2.0
    requested_x1 = int(np.floor(cx - half))
    requested_y1 = int(np.floor(cy - half))
    requested_x2 = requested_x1 + side_px
    requested_y2 = requested_y1 + side_px
    x1, y1 = max(0, requested_x1), max(0, requested_y1)
    x2, y2 = min(width, requested_x2), min(height, requested_y2)
    crop_width, crop_height = x2 - x1, y2 - y1
    bbox = (x1, y1, max(0, crop_width), max(0, crop_height))

    if crop_width <= 0 or crop_height <= 0:
        return EyeROI(side, center, bbox, None, False, "empty_roi", 1.0)

    area = float(side_px * side_px)
    clipped_fraction = 1.0 - (crop_width * crop_height) / area
    if clipped_fraction > config.severe_clipping_fraction:
        return EyeROI(
            side, center, bbox, None, False, "severe_frame_clipping",
            clipped_fraction,
        )
    if crop_width < config.min_eye_width or crop_height < config.min_eye_height:
        return EyeROI(
            side, center, bbox, None, False, "roi_below_minimum_size",
            clipped_fraction,
        )

    crop = frame[y1:y2, x1:x2].copy()
    # Restore the requested square with replicated edge pixels for minor clipping.
    top = y1 - requested_y1
    bottom = requested_y2 - y2
    left = x1 - requested_x1
    right = requested_x2 - x2
    if top or bottom or left or right:
        crop = cv2.copyMakeBorder(
            crop, top, bottom, left, right, cv2.BORDER_REPLICATE
        )
    return EyeROI(side, center, bbox, crop, True, "", clipped_fraction)


def faces_to_eye_rois(
    frame: np.ndarray,
    detections: Optional[np.ndarray],
    config: ExtractorConfig = ExtractorConfig(),
) -> List[FaceEyeROI]:
    """Convert YuNet rows to per-face eye crops without running the detector.

    YuNet row fields are x, y, width, height, right-eye x/y, left-eye x/y,
    nose x/y, right/left mouth corners x/y, and face confidence.
    """
    if not isinstance(frame, np.ndarray) or frame.ndim not in (2, 3):
        raise ValueError("frame must be a non-empty OpenCV image array")
    if frame.size == 0 or frame.shape[0] <= 0 or frame.shape[1] <= 0:
        raise ValueError("frame must be a non-empty OpenCV image array")
    if frame.ndim == 3 and frame.shape[2] not in (1, 3, 4):
        raise ValueError("frame must have 1, 3, or 4 channels")

    if detections is None:
        return []
    rows = np.asarray(detections)
    if rows.size == 0:
        return []
    if rows.ndim != 2 or rows.shape[1] < 15:
        raise ValueError("YuNet detections must be an N x 15 array")

    height, width = frame.shape[:2]
    results: List[FaceEyeROI] = []
    for row in rows:
        values = np.asarray(row[:15], dtype=np.float64)
        x, y, face_w, face_h = values[:4]
        confidence = float(values[14])
        if (not np.all(np.isfinite(values[:4]))
                or not np.isfinite(confidence) or face_w <= 0 or face_h <= 0):
            invalid = _invalid_eye("right", None, "invalid_face_or_landmark")
            invalid_left = _invalid_eye("left", None, "invalid_face_or_landmark")
            results.append(FaceEyeROI((0, 0, 0, 0), confidence, invalid,
                                      invalid_left, tuple([None] * 5)))
            continue

        fx1, fy1 = max(0, int(np.floor(x))), max(0, int(np.floor(y)))
        fx2 = min(width, int(np.ceil(x + face_w)))
        fy2 = min(height, int(np.ceil(y + face_h)))
        face_bbox = (fx1, fy1, max(0, fx2 - fx1), max(0, fy2 - fy1))

        right_center = _as_point(values[4], values[5], width, height)
        left_center = _as_point(values[6], values[7], width, height)
        other_landmarks = tuple(
            _as_point(values[i], values[i + 1], width, height)
            for i in (8, 10, 12)
        )
        landmarks: Tuple[Optional[Point], ...] = (
            right_center, left_center, *other_landmarks
        )

        reason = ""
        inter_eye = 0.0
        if confidence < config.face_confidence_threshold:
            reason = "face_confidence_below_threshold"
        elif right_center is None or left_center is None:
            reason = "invalid_eye_landmark"
        else:
            inter_eye = float(np.linalg.norm(
                np.asarray(right_center) - np.asarray(left_center)
            ))
            if inter_eye <= 1.0 or inter_eye > max(face_w, face_h) * 1.5:
                reason = "invalid_inter_eye_distance"

        if reason:
            right_eye = _invalid_eye("right", right_center, reason)
            left_eye = _invalid_eye("left", left_center, reason)
        else:
            side_px = max(1, int(round(
                inter_eye * config.roi_side_inter_eye_ratio
            )))
            right_eye = _extract_eye(
                frame, "right", right_center, side_px, config
            )
            left_eye = _extract_eye(frame, "left", left_center, side_px, config)
        results.append(FaceEyeROI(
            face_bbox, confidence, right_eye, left_eye, landmarks
        ))
    return results


class FaceEyeROIExtractor:
    """OpenCV YuNet detector with landmark-based eye ROI extraction."""

    def __init__(
        self,
        model_path: Path | str,
        config: ExtractorConfig = ExtractorConfig(),
        nms_threshold: float = 0.30,
        top_k: int = 5000,
    ) -> None:
        self.config = config
        model_path = Path(model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"YuNet model file not found: {model_path}")
        factory = getattr(cv2, "FaceDetectorYN", None)
        create = getattr(factory, "create", None) if factory else None
        if create is None:
            create = getattr(cv2, "FaceDetectorYN_create", None)
        if create is None:
            raise RuntimeError(
                f"OpenCV {cv2.__version__} has no FaceDetectorYN API; "
                "OpenCV 4.5.4 or newer with this API is required"
            )
        try:
            self._detector = create(
                str(model_path), "", (config.input_size, config.input_size),
                config.face_confidence_threshold, nms_threshold, top_k,
            )
        except Exception as exc:
            raise RuntimeError(
                f"could not load YuNet model {model_path}: {exc}"
            ) from exc

    def extract(self, frame: np.ndarray) -> List[FaceEyeROI]:
        """Detect all faces in one BGR/RGB frame and extract eye crops."""
        if not isinstance(frame, np.ndarray) or frame.ndim not in (2, 3):
            raise ValueError("frame must be a non-empty OpenCV image array")
        if frame.size == 0:
            raise ValueError("frame must be a non-empty OpenCV image array")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("YuNet input must be a 3-channel BGR or RGB frame")
        height, width = frame.shape[:2]
        self._detector.setInputSize((width, height))
        _, detections = self._detector.detect(frame)
        return faces_to_eye_rois(frame, detections, self.config)
