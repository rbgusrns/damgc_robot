#!/usr/bin/env python3
"""Fail-fast checks for an Ultralytics image classification dataset."""

from __future__ import annotations

import argparse
import hashlib
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    from PIL import Image
except ImportError as exc:  # pragma: no cover - environment dependent
    raise SystemExit("ERROR: Pillow is required (it is included in the project Docker image).") from exc


SPLITS = ("train", "val", "test")
CLASSES = ("open", "closed")
SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def validate(root: Path, check_duplicates: bool = True) -> bool:
    errors: list[str] = []
    warnings: list[str] = []
    counts: dict[str, Counter[str]] = {}
    dimensions: dict[str, list[tuple[int, int]]] = defaultdict(list)
    hashes: dict[str, list[tuple[str, Path]]] = defaultdict(list)

    if not root.exists() or not root.is_dir():
        print(f"ERROR: dataset root does not exist or is not a directory: {root}")
        return False

    for split in SPLITS:
        split_dir = root / split
        if not split_dir.is_dir():
            errors.append(f"missing split directory: {split_dir}")
            continue

        child_dirs = {p.name for p in split_dir.iterdir() if p.is_dir()}
        root_files = [p for p in split_dir.iterdir() if p.is_file()]
        if root_files:
            errors.append(f"files found directly under split directory {split}: " + ", ".join(map(str, root_files)))
        unexpected = sorted(child_dirs - set(CLASSES))
        if unexpected:
            errors.append(f"unexpected class directories in {split}: {', '.join(unexpected)}")
        counts[split] = Counter()
        for label in CLASSES:
            class_dir = split_dir / label
            if not class_dir.is_dir():
                errors.append(f"missing class directory: {class_dir}")
                counts[split][label] = 0
                continue

            files = sorted(p for p in class_dir.rglob("*") if p.is_file())
            valid_images: list[Path] = []
            for path in files:
                if path.suffix.lower() not in SUPPORTED_EXTENSIONS:
                    errors.append(f"unsupported file extension: {path}")
                    continue
                try:
                    with Image.open(path) as image:
                        image.verify()
                    with Image.open(path) as image:
                        width, height = image.size
                        if width < 1 or height < 1:
                            raise ValueError("zero-sized image")
                    valid_images.append(path)
                    dimensions[label].append((width, height))
                    if check_duplicates:
                        hasher = hashlib.sha256()
                        with path.open("rb") as image_file:
                            for chunk in iter(lambda: image_file.read(1024 * 1024), b""):
                                hasher.update(chunk)
                        digest = hasher.hexdigest()
                        hashes[digest].append((split, path))
                except Exception as exc:
                    errors.append(f"unreadable/corrupt image: {path} ({exc})")

            counts[split][label] = len(valid_images)
            if not valid_images:
                errors.append(f"no readable images in {split}/{label}")

        split_total = sum(counts[split].values())
        print(f"{split}: total={split_total}, open={counts[split]['open']}, closed={counts[split]['closed']}")
        if split_total and all(counts[split][name] for name in CLASSES):
            high = max(counts[split][name] for name in CLASSES)
            low = min(counts[split][name] for name in CLASSES)
            if high / low > 3.0:
                warnings.append(f"{split} class imbalance exceeds 3:1 ({high}:{low})")
        if split_total == 0:
            errors.append(f"empty split: {split}")

    if check_duplicates:
        for entries in hashes.values():
            split_names = {split for split, _ in entries}
            if len(split_names) > 1:
                errors.append("identical image content appears across splits: " + ", ".join(str(path) for _, path in entries))

    print("Image dimensions (width x height):")
    for label in CLASSES:
        values = dimensions[label]
        if values:
            widths = [item[0] for item in values]
            heights = [item[1] for item in values]
            print(
                f"  {label}: n={len(values)}, width min/median/max="
                f"{min(widths)}/{statistics.median(widths):g}/{max(widths)}, "
                f"height min/median/max={min(heights)}/{statistics.median(heights):g}/{max(heights)}"
            )

    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}")
    if errors:
        print(f"Dataset validation FAILED ({len(errors)} error(s), {len(warnings)} warning(s)).")
        return False
    print(f"Dataset validation PASS ({len(warnings)} warning(s)).")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path, help="Dataset root with train/val/test")
    parser.add_argument("--skip-duplicate-check", action="store_true", help="Skip SHA-256 split duplicate scan")
    args = parser.parse_args()
    return 0 if validate(args.data.expanduser().resolve(), not args.skip_duplicate_check) else 2


if __name__ == "__main__":
    sys.exit(main())
