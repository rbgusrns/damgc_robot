#!/usr/bin/env python3
"""Evaluate a fine-tuned classifier on the independent test split."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from validate_dataset import CLASSES, SUPPORTED_EXTENSIONS, validate


def class_map(model) -> dict[int, str]:
    names = model.names
    names = {int(key): str(value) for key, value in names.items()}
    if set(names.values()) != set(CLASSES) or len(names) != 2:
        raise ValueError(f"model classes must be exactly {CLASSES}; found {names}")
    return names


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--device", default="auto", help="auto, 0, or cpu")
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--output", type=Path, default=Path("test_predictions.csv"))
    parser.add_argument("--all-output", type=Path, help="Optional CSV with predictions for every test sample")
    args = parser.parse_args()

    model_path = args.model.expanduser().resolve()
    data = args.data.expanduser().resolve()
    if not model_path.is_file():
        print(f"ERROR: model file not found: {model_path}", file=sys.stderr)
        return 2
    if not validate(data):
        return 2
    try:
        import torch
        from ultralytics import YOLO
        if args.device == "auto":
            device = "0" if torch.cuda.is_available() else "cpu"
            if device == "cpu":
                print("WARNING: CUDA is unavailable; evaluation will run on CPU.")
        elif args.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError(f"requested device {args.device!r}, but CUDA is unavailable")
        else:
            device = args.device
        model = YOLO(str(model_path))
        if model.task != "classify":
            raise ValueError(f"unsupported model task {model.task!r}; expected 'classify'")
        names = class_map(model)
    except Exception as exc:
        print(f"ERROR: could not load a compatible classifier: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    samples = [(path, label) for label in CLASSES for path in sorted((data / "test" / label).rglob("*"))
               if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS]
    confusion = {truth: {pred: 0 for pred in CLASSES} for truth in CLASSES}
    rows: list[dict[str, str | float]] = []
    try:
        results = model.predict(
            source=[str(path) for path, _ in samples],
            imgsz=args.imgsz,
            device=device,
            stream=True,
            verbose=False,
        )
        for (path, truth), result in zip(samples, results, strict=True):
            probabilities = result.probs.data.detach().cpu().tolist()
            pred_index = max(range(len(probabilities)), key=probabilities.__getitem__)
            prediction = names[pred_index]
            confidence = float(probabilities[pred_index])
            confusion[truth][prediction] += 1
            rows.append({"path": str(path), "true": truth, "predicted": prediction, "confidence": confidence})
    except Exception as exc:
        print(f"ERROR: evaluation inference failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    total = len(rows)
    correct = sum(confusion[label][label] for label in CLASSES)
    print(f"Total samples: {total}")
    print(f"Overall accuracy: {correct / total:.4f}")
    for label in CLASSES:
        tp = confusion[label][label]
        predicted = sum(confusion[truth][label] for truth in CLASSES)
        support = sum(confusion[label].values())
        precision = tp / predicted if predicted else 0.0
        recall = tp / support if support else 0.0
        print(f"{label.upper()}: precision={precision:.4f}, recall={recall:.4f}, samples={support}")
    print("Confusion matrix (rows=true, columns=predicted; order=open, closed):")
    print(f"             OPEN  CLOSED")
    for truth in CLASSES:
        print(f"{truth.upper():>8} {confusion[truth]['open']:>6} {confusion[truth]['closed']:>7}")

    output = args.output.expanduser().resolve()
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("path", "true", "predicted", "confidence"))
            writer.writeheader()
            writer.writerows(row for row in rows if row["true"] != row["predicted"])
    except OSError as exc:
        print(f"ERROR: could not write misclassification report {output}: {exc}", file=sys.stderr)
        return 2
    print(f"Misclassified samples CSV (header only if none): {output}")
    if args.all_output is not None:
        all_output = args.all_output.expanduser().resolve()
        try:
            all_output.parent.mkdir(parents=True, exist_ok=True)
            with all_output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=("path", "true", "predicted", "confidence"))
                writer.writeheader()
                writer.writerows(rows)
        except OSError as exc:
            print(f"ERROR: could not write all-predictions report {all_output}: {exc}", file=sys.stderr)
            return 2
        least_confident = min(rows, key=lambda row: row["confidence"])
        print(f"All test predictions CSV: {all_output}")
        print(f"Lowest-confidence sample: {least_confident['path']} (true={least_confident['true']}, predicted={least_confident['predicted']}, confidence={least_confident['confidence']:.4f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
