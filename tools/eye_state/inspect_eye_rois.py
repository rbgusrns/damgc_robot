#!/usr/bin/env python3
"""Save a visual diagnostic of YuNet faces, landmarks, and eye crops."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

from face_eye_roi import ExtractorConfig, FaceEyeROIExtractor
from fetch_yunet_model import DEFAULT_PATH


def _draw(frame, faces):
    output = frame.copy()
    for index, face in enumerate(faces):
        x, y, width, height = face.bbox
        cv2.rectangle(output, (x, y), (x + width, y + height), (0, 220, 0), 2)
        cv2.putText(
            output, f"Face {index} {face.confidence:.2f}", (x, max(18, y - 7)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA,
        )
        for point in face.landmarks:
            if point is not None:
                cv2.circle(output, tuple(round(v) for v in point), 3,
                           (255, 180, 0), -1, cv2.LINE_AA)
        for eye, color in ((face.left_eye, (255, 0, 255)),
                           (face.right_eye, (0, 255, 255))):
            if eye.bbox is not None:
                ex, ey, ew, eh = eye.bbox
                cv2.rectangle(output, (ex, ey), (ex + ew, ey + eh), color, 2)
            label = f"{eye.side.upper()}: " + (
                f"VALID {eye.bbox[2]}x{eye.bbox[3]}"
                if eye.valid and eye.bbox else f"INVALID ({eye.reason})"
            )
            text_y = max(16, (eye.bbox[1] if eye.bbox else y) - 4)
            cv2.putText(output, label, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, color, 1, cv2.LINE_AA)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_PATH)
    parser.add_argument("--output-dir", type=Path, default=Path("eye_roi_debug"))
    parser.add_argument("--face-conf", type=float, default=0.60)
    parser.add_argument("--roi-ratio", type=float, default=0.85,
                        help="square ROI side divided by inter-eye distance")
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
        extractor = FaceEyeROIExtractor(args.model, config)
        faces = extractor.extract(frame)
        annotated = _draw(frame, faces)
        debug_path = output_dir / f"{image_path.stem}_eye_rois.jpg"
        if not cv2.imwrite(str(debug_path), annotated):
            raise OSError(f"could not write debug image: {debug_path}")
        print(f"faces_detected: {len(faces)}")
        print(f"debug_image: {debug_path}")
        for index, face in enumerate(faces):
            print(f"Face {index}: bbox={face.bbox}, confidence={face.confidence:.4f}")
            for eye in (face.left_eye, face.right_eye):
                print(f"  {eye.side}: {'VALID' if eye.valid else 'INVALID'}"
                      f" bbox={eye.bbox} clipped={eye.clipped_fraction:.3f}"
                      f" reason={eye.reason or '-'}")
                if args.save_crops and eye.valid and eye.image is not None:
                    crop_path = output_dir / f"face{index}_{eye.side}_eye.png"
                    if not cv2.imwrite(str(crop_path), eye.image):
                        raise OSError(f"could not write eye crop: {crop_path}")
                    print(f"  crop: {crop_path}")
    except Exception as exc:
        print(f"ERROR: ROI inspection failed: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
