#!/usr/bin/env python3
"""Run face/eye ROI extraction and independent OPEN/CLOSED inference."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from face_eye_roi import ExtractorConfig, FaceEyeROIExtractor
from eye_classifier import EyeStateClassifier, classify_valid_eye_rois
from fetch_yunet_model import DEFAULT_PATH as DEFAULT_YUNET_PATH


DEFAULT_CLASSIFIER = Path(
    "/home/maze/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt"
)


def _label(eye, prediction):
    if not eye.valid:
        return f"{eye.side[0].upper()}: INVALID ({eye.reason})"
    if prediction is None:
        return f"{eye.side[0].upper()}: ERROR"
    return f"{eye.side[0].upper()}: {prediction.state} {prediction.confidence:.3f}"


def _annotate(frame, faces, predictions_by_slot):
    output = frame.copy()
    for index, face in enumerate(faces):
        x, y, width, height = face.bbox
        cv2.rectangle(output, (x, y), (x + width, y + height), (0, 220, 0), 2)
        cv2.putText(output, f"Face {index} {face.confidence:.2f}",
                    (x, max(18, y - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 220, 0), 1, cv2.LINE_AA)
        for eye, color in ((face.left_eye, (255, 0, 255)),
                           (face.right_eye, (0, 255, 255))):
            prediction = None
            if eye.valid:
                prediction = predictions_by_slot.get((index, eye.side))
            if eye.bbox is not None:
                ex, ey, ew, eh = eye.bbox
                cv2.rectangle(output, (ex, ey), (ex + ew, ey + eh), color, 2)
            label = _label(eye, prediction)
            text_y = min(output.shape[0] - 4,
                         max(15, (eye.bbox[1] if eye.bbox else y) - 4))
            label_x = eye.bbox[0] if eye.bbox else x
            cv2.putText(output, label, (label_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
    if not faces:
        cv2.putText(output, "NO_FACE", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2, cv2.LINE_AA)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_CLASSIFIER)
    parser.add_argument("--yunet-model", type=Path, default=DEFAULT_YUNET_PATH)
    parser.add_argument("--output-dir", type=Path, default=Path("eye_roi_debug"))
    parser.add_argument("--device", default="auto",
                        help="auto, 0, or cpu (default: auto)")
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--preprocessing-mode", choices=("rgb", "gray"),
                        default="gray",
                        help="gray matches the public training images; compare with rgb")
    parser.add_argument("--min-confidence", type=float, default=0.60)
    parser.add_argument("--face-conf", type=float, default=0.60)
    parser.add_argument("--roi-ratio", type=float, default=0.85)
    parser.add_argument("--min-eye-width", type=int, default=24)
    parser.add_argument("--min-eye-height", type=int, default=24)
    parser.add_argument("--severe-clipping-fraction", type=float, default=0.25)
    parser.add_argument("--save-crops", action="store_true")
    args = parser.parse_args()

    image_path = args.image.expanduser().resolve()
    if not image_path.is_file():
        print(f"ERROR: input image not found: {image_path}", file=sys.stderr)
        return 2
    frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0:
        print(f"ERROR: could not read image: {image_path}", file=sys.stderr)
        return 2
    output_dir = args.output_dir.expanduser().resolve()
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        config = ExtractorConfig(
            face_confidence_threshold=args.face_conf,
            roi_side_inter_eye_ratio=args.roi_ratio,
            min_eye_width=args.min_eye_width,
            min_eye_height=args.min_eye_height,
            severe_clipping_fraction=args.severe_clipping_fraction,
        )
        # Each model is initialized once for this process, outside frame inference.
        extractor = FaceEyeROIExtractor(args.yunet_model, config)
        classifier = EyeStateClassifier(
            args.model, args.device, args.imgsz, args.preprocessing_mode,
            args.min_confidence,
        )

        total_start = time.perf_counter()
        face_start = time.perf_counter()
        faces = extractor.extract(frame)
        face_latency_ms = (time.perf_counter() - face_start) * 1000.0

        classifier_start = time.perf_counter()
        predictions_by_slot = classify_valid_eye_rois(faces, classifier)
        classifier_latency_ms = (time.perf_counter() - classifier_start) * 1000.0
        total_latency_ms = (time.perf_counter() - total_start) * 1000.0
        crop_count = len(predictions_by_slot)

        print(f"classifier_model: {classifier.model_path}")
        print(f"model_names: {classifier.names}")
        print(f"device: {classifier.device}")
        print(f"preprocessing_mode: {args.preprocessing_mode}")
        if not faces:
            print("NO_FACE")
        for face_index, face in enumerate(faces):
            print(f"Face {face_index}: bbox={face.bbox} face_confidence={face.confidence:.4f}")
            for eye in (face.left_eye, face.right_eye):
                key = (face_index, eye.side)
                prediction = predictions_by_slot.get(key)
                if prediction is None:
                    print(f"  {eye.side.upper()}: INVALID ({eye.reason})")
                    continue
                print(f"  {eye.side.upper()}: {prediction.state} "
                      f"confidence={prediction.confidence:.4f} "
                      f"probabilities={prediction.probabilities}")
                if args.save_crops:
                    crop_path = output_dir / f"face{face_index}_{eye.side}_eye.png"
                    if not cv2.imwrite(str(crop_path), eye.image):
                        raise OSError(f"could not write eye crop: {crop_path}")

        annotated = _annotate(frame, faces, predictions_by_slot)
        debug_path = output_dir / f"{image_path.stem}_eye_state.jpg"
        if not cv2.imwrite(str(debug_path), annotated):
            raise OSError(f"could not write debug image: {debug_path}")
        print(f"face_detection_and_roi_ms: {face_latency_ms:.2f}")
        print(f"classifier_batch_ms: {classifier_latency_ms:.2f} "
              f"(crops={crop_count})")
        print(f"total_inference_ms: {total_latency_ms:.2f}")
        print(f"debug_image: {debug_path}")
    except Exception as exc:
        print(f"ERROR: Eye State inference failed: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
