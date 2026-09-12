"""ROS-independent person detection display helpers."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Mapping, Optional, Tuple

import cv2
import numpy as np


PERSON_CLASS_ID = 0


@dataclass(frozen=True)
class Detection:
    """One model detection in image-pixel coordinates."""

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int


@dataclass(frozen=True)
class PersonDetection:
    """A validated person box ready for deterministic display."""

    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float

    @property
    def center_x(self) -> float:
        """Return the horizontal box center in pixels."""
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        """Return the vertical box center in pixels."""
        return (self.y1 + self.y2) / 2.0


def prepare_person_detections(
    detections: Iterable[Detection],
    confidence_threshold: float,
    image_width: int,
    image_height: int,
) -> List[PersonDetection]:
    """Filter, clamp, and sort person detections from left to right."""
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold must be between 0.0 and 1.0")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    people = []
    for detection in detections:
        values = (
            detection.x1,
            detection.y1,
            detection.x2,
            detection.y2,
            detection.confidence,
        )
        if detection.class_id != PERSON_CLASS_ID or not all(
            math.isfinite(value) for value in values
        ):
            continue
        if detection.confidence < confidence_threshold:
            continue

        x1 = max(0, min(image_width - 1, int(round(detection.x1))))
        y1 = max(0, min(image_height - 1, int(round(detection.y1))))
        x2 = max(0, min(image_width - 1, int(round(detection.x2))))
        y2 = max(0, min(image_height - 1, int(round(detection.y2))))
        if x2 <= x1 or y2 <= y1:
            continue
        people.append(PersonDetection(x1, y1, x2, y2, detection.confidence))

    return sorted(
        people,
        key=lambda person: (
            person.center_x,
            person.center_y,
            person.x1,
            person.y1,
        ),
    )


def draw_person_detections(
    image: np.ndarray,
    people: Iterable[PersonDetection],
    distances: Optional[Mapping[int, Optional[float]]] = None,
    rois: Optional[Mapping[int, Tuple[int, int, int, int]]] = None,
    show_depth_roi: bool = False,
) -> np.ndarray:
    """Draw boxes and frame-local person numbers on an image in place."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected a three-channel BGR image")

    height, width = image.shape[:2]
    color = (0, 255, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 2

    for number, person in enumerate(people, start=1):
        distance = distances.get(number) if distances is not None else None
        distance_label = f"{distance:.2f} m" if distance is not None else "N/A"
        label = f"person{number} {person.confidence:.2f} | {distance_label}"
        cv2.rectangle(
            image,
            (person.x1, person.y1),
            (person.x2, person.y2),
            color,
            thickness,
        )
        (text_width, text_height), baseline = cv2.getTextSize(
            label, font, font_scale, thickness
        )
        text_x = max(0, min(person.x1, max(0, width - text_width - 1)))
        text_y = person.y1 - 6
        if text_y - text_height < 0:
            text_y = min(height - baseline - 1, person.y1 + text_height + 6)
        cv2.putText(
            image,
            label,
            (text_x, max(text_height, text_y)),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
        if show_depth_roi and rois is not None and number in rois:
            roi = rois[number]
            cv2.rectangle(image, (roi[0], roi[1]), (roi[2] - 1, roi[3] - 1), (255, 0, 0), 1)
    return image
