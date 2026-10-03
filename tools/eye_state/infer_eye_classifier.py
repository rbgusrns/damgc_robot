#!/usr/bin/env python3
"""Classify one eye image with a fine-tuned OPEN/CLOSED model."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from validate_dataset import CLASSES, SUPPORTED_EXTENSIONS


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--device", default="auto", help="auto, 0, or cpu")
    parser.add_argument("--imgsz", type=int, default=224)
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
    try:
        from PIL import Image
        with Image.open(image_path) as image:
            image.verify()
    except Exception as exc:
        print(f"ERROR: unreadable image {image_path}: {exc}", file=sys.stderr)
        return 2

    try:
        import torch
        from ultralytics import YOLO
        if args.device == "auto":
            device = "0" if torch.cuda.is_available() else "cpu"
            if device == "cpu":
                print("WARNING: CUDA is unavailable; inference will run on CPU.")
        elif args.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError(f"requested device {args.device!r}, but CUDA is unavailable")
        else:
            device = args.device
        model = YOLO(str(model_path))
        if model.task != "classify":
            raise ValueError(f"unsupported model task {model.task!r}; expected 'classify'")
        names = {int(key): str(value) for key, value in model.names.items()}
        if len(names) != 2 or set(names.values()) != set(CLASSES):
            raise ValueError(f"model classes must be exactly {CLASSES}; found {names}")
        result = model.predict(str(image_path), imgsz=args.imgsz, device=device, verbose=False)[0]
        probabilities = result.probs.data.detach().cpu().tolist()
        prediction_id = max(range(len(probabilities)), key=probabilities.__getitem__)
        print(f"prediction: {names[prediction_id].upper()}")
        print(f"confidence: {probabilities[prediction_id]:.4f}")
        for class_id, name in names.items():
            print(f"{name.upper()}: {probabilities[class_id]:.4f}")
    except Exception as exc:
        print(f"ERROR: inference failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
