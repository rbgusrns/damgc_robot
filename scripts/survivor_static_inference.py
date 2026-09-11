#!/usr/bin/env python3
"""Run one YOLO11n GPU inference for the survivor runtime smoke test."""

import sys
from pathlib import Path

import requests
from ultralytics import YOLO


def main() -> None:
    target = Path(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] else Path(
        "/root/.cache/ultralytics/bus.jpg"
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists():
        response = requests.get("https://ultralytics.com/images/bus.jpg", timeout=30)
        response.raise_for_status()
        target.write_bytes(response.content)

    model = YOLO("yolo11n.pt")
    results = model.predict(source=str(target), device="0", verbose=False)
    count = 0 if not results or results[0].boxes is None else len(results[0].boxes)
    print(f"static image: {target}")
    print(f"detections: {count}")
    print("GPU inference: OK")


if __name__ == "__main__":
    main()
