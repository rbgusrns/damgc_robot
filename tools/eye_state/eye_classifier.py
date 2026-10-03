#!/usr/bin/env python3
"""Reusable YOLO11n-cls loading and per-eye inference helpers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Sequence
import sys

import cv2
import numpy as np


CLASSES = ("open", "closed")


@dataclass(frozen=True)
class EyePrediction:
    """One eye's predicted class, confidence, and class probabilities."""

    state: str
    predicted_class: str
    confidence: float
    probabilities: Dict[str, float]


def normalize_model_names(names) -> Dict[int, str]:
    """Normalize checkpoint class names without assuming index ordering."""
    if isinstance(names, Mapping):
        normalized = {int(index): str(name).strip().lower()
                      for index, name in names.items()}
    elif isinstance(names, (list, tuple)):
        normalized = {index: str(name).strip().lower()
                      for index, name in enumerate(names)}
    else:
        raise ValueError(f"unsupported model.names value: {type(names).__name__}")
    if sorted(normalized) != list(range(len(normalized))):
        raise ValueError(f"model class indices must be contiguous from 0: {normalized}")
    if len(normalized) != 2 or set(normalized.values()) != set(CLASSES):
        raise ValueError(
            f"model classes must be exactly {sorted(CLASSES)}; got {normalized}"
        )
    return normalized


def prediction_from_probabilities(
    names: Mapping[int, str],
    probabilities: Sequence[float],
    min_confidence: float,
) -> EyePrediction:
    """Map probabilities through checkpoint names and apply a confidence gate."""
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError("min_confidence must be in [0, 1]")
    if len(probabilities) != len(names) or len(probabilities) != 2:
        raise ValueError(
            f"expected two probabilities matching model.names, got {len(probabilities)}"
        )
    probs = [float(value) for value in probabilities]
    if not np.all(np.isfinite(probs)) or any(value < 0.0 or value > 1.0 for value in probs):
        raise ValueError(f"invalid class probabilities: {probs}")
    class_id = max(range(len(probs)), key=probs.__getitem__)
    predicted_class = names[class_id]
    confidence = probs[class_id]
    state = predicted_class.upper() if confidence >= min_confidence else "LOW_CONFIDENCE"
    probabilities_by_name = {
        names[index].upper(): probs[index] for index in range(len(probs))
    }
    return EyePrediction(state, predicted_class, confidence, probabilities_by_name)


def prepare_eye_crop(crop: np.ndarray, preprocessing_mode: str) -> np.ndarray:
    """Prepare a BGR crop for Ultralytics' standard classifier pipeline.

    Ultralytics receives BGR arrays and performs its normal BGR-to-RGB conversion.
    In gray mode this helper converts to grayscale and replicates that plane into
    three channels before passing it through the same Ultralytics transforms.
    """
    if preprocessing_mode not in ("rgb", "gray"):
        raise ValueError("preprocessing_mode must be 'rgb' or 'gray'")
    if not isinstance(crop, np.ndarray) or crop.size == 0:
        raise ValueError("eye crop must be a non-empty image array")
    if crop.ndim != 3 or crop.shape[2] != 3:
        raise ValueError("eye crop must be a 3-channel BGR image")
    if preprocessing_mode == "rgb":
        return np.ascontiguousarray(crop)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    return np.ascontiguousarray(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))


class EyeStateClassifier:
    """Load a classifier once and classify multiple eye crops per frame."""

    def __init__(
        self,
        model_path: Path | str,
        device: str = "auto",
        imgsz: int = 224,
        preprocessing_mode: str = "gray",
        min_confidence: float = 0.60,
    ) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        self.imgsz = int(imgsz)
        self.preprocessing_mode = preprocessing_mode
        self.min_confidence = float(min_confidence)
        if not self.model_path.is_file():
            raise FileNotFoundError(f"classifier checkpoint not found: {self.model_path}")
        if self.imgsz < 1:
            raise ValueError("imgsz must be positive")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if preprocessing_mode not in ("rgb", "gray"):
            raise ValueError("preprocessing_mode must be 'rgb' or 'gray'")
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "PyTorch/Ultralytics unavailable; use the verified "
                "damgc-survivor-yolo:humble runtime"
            ) from exc

        if device == "auto":
            if torch.cuda.is_available():
                self.device = "0"
            else:
                self.device = "cpu"
                print("WARNING: CUDA unavailable; eye inference uses CPU.",
                      file=sys.stderr)
        elif device == "cpu":
            self.device = "cpu"
        elif torch.cuda.is_available():
            self.device = device
        else:
            raise RuntimeError(
                f"requested device {device!r}, but CUDA is unavailable"
            )

        self.model = YOLO(str(self.model_path))
        if self.model.task != "classify":
            raise ValueError(
                f"unsupported model task {self.model.task!r}; expected 'classify'"
            )
        self.names = normalize_model_names(self.model.names)

    def predict(self, crops: Sequence[np.ndarray]) -> List[EyePrediction]:
        """Classify a batch of valid BGR eye crops with the loaded model."""
        if not crops:
            return []
        prepared = [
            prepare_eye_crop(crop, self.preprocessing_mode) for crop in crops
        ]
        results = self.model.predict(
            source=prepared,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        if len(results) != len(prepared):
            raise RuntimeError(
                f"classifier returned {len(results)} results for "
                f"{len(prepared)} eye crops"
            )
        predictions: List[EyePrediction] = []
        for result in results:
            if result.probs is None:
                raise RuntimeError("classification result has no probabilities")
            probs = result.probs.data.detach().cpu().tolist()
            predictions.append(prediction_from_probabilities(
                self.names, probs, self.min_confidence
            ))
        return predictions


def classify_valid_eye_rois(faces, classifier: EyeStateClassifier):
    """Infer only valid ROIs and return predictions keyed by face and side."""
    slots = []
    crops = []
    for face_index, face in enumerate(faces):
        for eye in (face.left_eye, face.right_eye):
            if eye.valid and eye.image is not None:
                slots.append((face_index, eye.side))
                crops.append(eye.image)
    predictions = classifier.predict(crops) if crops else []
    if len(predictions) != len(slots):
        raise RuntimeError(
            f"classifier returned {len(predictions)} predictions for "
            f"{len(slots)} valid eye ROIs"
        )
    return dict(zip(slots, predictions))
