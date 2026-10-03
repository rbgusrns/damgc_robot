#!/usr/bin/env python3
"""Classify one eye image with a fine-tuned OPEN/CLOSED model."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

from eye_classifier import EyeStateClassifier
from validate_dataset import SUPPORTED_EXTENSIONS


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--device", default="auto", help="auto, 0, or cpu")
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--preprocessing-mode", choices=("rgb", "gray"),
                        default="rgb")
    args = parser.parse_args()

    model_path = args.model.expanduser().resolve()
    image_path = args.image.expanduser().resolve()
    if not model_path.is_file():
        print(f"ERROR: model file not found: {model_path}", file=sys.stderr)
        return 2
    if not image_path.is_file():
        print(f"ERROR: image file not found: {image_path}", file=sys.stderr)
        return 2
    if image_path.suffix.lower() not in SUPPORTED_EXTENSIONS:
        print(f"ERROR: unsupported image extension: {image_path.suffix}", file=sys.stderr)
        return 2
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        print(f"ERROR: unreadable image {image_path}", file=sys.stderr)
        return 2

    try:
        classifier = EyeStateClassifier(
            model_path=model_path,
            device=args.device,
            imgsz=args.imgsz,
            preprocessing_mode=args.preprocessing_mode,
            min_confidence=0.0,
        )
        prediction = classifier.predict([image])[0]
        print(f"prediction: {prediction.predicted_class.upper()}")
        print(f"confidence: {prediction.confidence:.4f}")
        for class_name, probability in prediction.probabilities.items():
            print(f"{class_name}: {probability:.4f}")
    except Exception as exc:
        print(f"ERROR: inference failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
