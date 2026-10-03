#!/usr/bin/env python3
"""Fine-tune YOLO11n-cls on the standalone open/closed eye dataset."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from validate_dataset import validate


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path, help="Dataset root with train/val/test")
    parser.add_argument("--model", default="yolo11n-cls.pt", help="Pretrained classification weights")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="auto", help="auto, 0, or cpu")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--project", type=Path, default=Path("/outputs/eye_state"))
    parser.add_argument("--name", default="baseline")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", action="store_true", help="Resume an interrupted run from its last checkpoint")
    args = parser.parse_args()

    data = args.data.expanduser().resolve()
    if not validate(data):
        return 2
    if args.epochs < 1 or args.imgsz < 32 or args.batch < 1 or args.workers < 0:
        parser.error("epochs/imgsz/batch must be positive (imgsz >= 32), workers must be non-negative")

    try:
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        print(f"ERROR: training dependencies could not be imported: {exc}", file=sys.stderr)
        print("Run this tool inside damgc-survivor-yolo:humble; do not install over the host/Survivor runtime.", file=sys.stderr)
        return 2

    if args.device == "auto":
        if torch.cuda.is_available():
            device = "0"
        else:
            device = "cpu"
            print("WARNING: CUDA is unavailable; training will run on CPU and may be very slow.")
    elif args.device != "cpu" and not torch.cuda.is_available():
        print(f"ERROR: requested device {args.device!r}, but CUDA is unavailable.", file=sys.stderr)
        return 2
    else:
        device = args.device

    project = args.project.expanduser().resolve()
    try:
        project.mkdir(parents=True, exist_ok=True)
        available = shutil.disk_usage(project).free
        probe = project / ".eye_state_write_check"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink()
    except OSError as exc:
        print(f"ERROR: output directory is not writable: {project} ({exc})", file=sys.stderr)
        return 2

    print(f"Device: {device}; CUDA available: {torch.cuda.is_available()}")
    if device != "cpu" and torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"Available output disk: {available / (1024 ** 3):.2f} GiB")
    print("Use a conservative batch size and do not train while VSLAM/Survivor workloads are active.")

    try:
        model = YOLO(args.model)
        if model.task != "classify":
            print(f"ERROR: unsupported model task {model.task!r}; expected 'classify'.", file=sys.stderr)
            return 2
        if args.resume:
            model.train(resume=True)
        else:
            model.train(
                data=str(data),
                epochs=args.epochs,
                imgsz=args.imgsz,
                batch=args.batch,
                device=device,
                workers=args.workers,
                project=str(project),
                name=args.name,
                seed=args.seed,
                deterministic=True,
                plots=True,
            )
    except RuntimeError as exc:
        if "out of memory" in str(exc).lower():
            print("ERROR: GPU out of memory. Stop other GPU workloads and retry with a smaller --batch (for example 4 or 2).", file=sys.stderr)
        else:
            print(f"ERROR: training failed: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"ERROR: training failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    trainer = getattr(model, "trainer", None)
    best = Path(trainer.best) if trainer is not None and getattr(trainer, "best", None) else project / args.name / "weights" / "best.pt"
    print(f"Best model: {best}")
    return 0 if best.is_file() else 2


if __name__ == "__main__":
    sys.exit(main())
