#!/usr/bin/env python3
"""Map a Roboflow multiclass export to the standalone eye dataset layout."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from datetime import date
from pathlib import Path

from validate_dataset import SUPPORTED_EXTENSIONS, validate


SPLIT_NAMES = {"train": ("train",), "val": ("valid", "val"), "test": ("test",)}
CLASS_NAMES = {"open": ("Open_Eyes", "open"), "closed": ("Closed_Eyes", "closed")}
PROJECT_URL = "https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification"


def _choose_directory(parent: Path, names: tuple[str, ...], description: str) -> Path:
    matches = [parent / name for name in names if (parent / name).is_dir()]
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one {description} directory under {parent} "
            f"(accepted names: {', '.join(names)}); found {len(matches)}"
        )
    return matches[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path, help="Extracted Roboflow multiclass export root")
    parser.add_argument("--output", required=True, type=Path, help="New train/val/test dataset root")
    parser.add_argument("--version", type=int, default=1, help="Roboflow dataset version (default: 1)")
    parser.add_argument("--download-date", required=True, help="Date the export was downloaded, YYYY-MM-DD")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    try:
        date.fromisoformat(args.download_date)
        if args.version < 1:
            raise ValueError("version must be positive")
        if not source.is_dir():
            raise ValueError(f"source directory does not exist: {source}")
        if output.exists():
            raise ValueError(f"output already exists; refusing to overwrite: {output}")
        if output.is_relative_to(source) or source.is_relative_to(output):
            raise ValueError("source and output directories must not overlap")

        selected: dict[str, dict[str, list[tuple[Path, Path]]]] = {}
        for target_split, aliases in SPLIT_NAMES.items():
            split_dir = _choose_directory(source, aliases, target_split)
            selected[target_split] = {}
            chosen_classes: set[str] = set()
            for target_class, class_aliases in CLASS_NAMES.items():
                class_dir = _choose_directory(split_dir, class_aliases, target_class)
                chosen_classes.add(class_dir.name)
                paths = sorted(path for path in class_dir.rglob("*") if path.is_file())
                if not paths:
                    raise ValueError(f"no images in {class_dir}")
                for path in paths:
                    if path.is_symlink() or path.suffix.lower() not in SUPPORTED_EXTENSIONS:
                        raise ValueError(f"unsupported file or symlink in class directory: {path}")
                selected[target_split][target_class] = [(path, path.relative_to(class_dir)) for path in paths]
            extras = {path.name for path in split_dir.iterdir() if path.is_dir()} - chosen_classes
            if extras:
                raise ValueError(f"unexpected class directories in {split_dir}: {sorted(extras)}")

        output.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=".eye_state_prepare_", dir=output.parent))
        try:
            counts: dict[str, dict[str, int]] = {}
            for split, classes in selected.items():
                counts[split] = {}
                for label, images in classes.items():
                    counts[split][label] = len(images)
                    for image, relative in images:
                        destination = staging / split / label / relative
                        destination.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(image, destination)
            manifest = {
                "dataset": "Eye state Classification",
                "author": "Mujahid Raja",
                "source": PROJECT_URL,
                "version": args.version,
                "version_url": f"{PROJECT_URL}/dataset/{args.version}",
                "license": "CC BY 4.0",
                "attribution": "Mujahid Raja, Eye state Classification, Roboflow Universe",
                "download_date": args.download_date,
                "original_image_count": sum(len(images) for classes in selected.values() for images in classes.values()),
                "source_split_preserved": True,
                "transformation": {
                    "split_mapping": {"valid": "val"},
                    "image_handling": "Copied without further resize or augmentation",
                },
                "class_mapping": {"Open_Eyes": "open", "Closed_Eyes": "closed"},
                "counts": counts,
            }
            (staging / "dataset_source.json").write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            if not validate(staging):
                raise ValueError("prepared dataset failed validation; inspect source split overlap or image errors")
            if output.exists():
                raise ValueError(f"output appeared during preparation; refusing to overwrite: {output}")
            staging.rename(output)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
    except (OSError, ValueError) as exc:
        print(f"ERROR: dataset preparation failed: {exc}", file=sys.stderr)
        return 2

    print(f"Prepared dataset: {output}")
    print(f"Source record: {output / 'dataset_source.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
