#!/usr/bin/env python3
"""Download the pinned, MIT-licensed OpenCV Zoo YuNet model."""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tempfile
import urllib.request
from pathlib import Path


REVISION = "c97242ce7f2a554e288b50eabd9f5df957e78801"
MODEL_NAME = "face_detection_yunet_2022mar.onnx"
MODEL_URL = (
    "https://media.githubusercontent.com/media/opencv/opencv_zoo/"
    f"{REVISION}/models/face_detection_yunet/{MODEL_NAME}"
)
MODEL_SHA256 = "50ef07f702a31741ca46a4c0d947773b64143b9362780237bf0d427d6c79bab7"
DEFAULT_PATH = Path.home() / ".cache/damgc-eye-state" / MODEL_NAME


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_PATH)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.is_file():
            actual = sha256(output)
            if actual == MODEL_SHA256:
                print(f"YuNet model already verified: {output}")
                return 0
            raise RuntimeError(
                f"existing model has unexpected SHA-256 ({actual}): {output}; "
                "move it aside before fetching the pinned model"
            )
        with tempfile.NamedTemporaryFile(
            prefix=f".{MODEL_NAME}.", dir=output.parent, delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
        try:
            request = urllib.request.Request(
                MODEL_URL, headers={"User-Agent": "damgc-eye-state/1.0"}
            )
            with urllib.request.urlopen(request, timeout=45) as response:
                with temporary_path.open("wb") as target:
                    while True:
                        block = response.read(1024 * 1024)
                        if not block:
                            break
                        target.write(block)
            actual = sha256(temporary_path)
            if actual != MODEL_SHA256:
                raise RuntimeError(
                    f"download checksum mismatch: expected {MODEL_SHA256}, "
                    f"received {actual}"
                )
            os.replace(temporary_path, output)
        finally:
            temporary_path.unlink(missing_ok=True)
    except Exception as exc:
        print(f"ERROR: YuNet model acquisition failed: {exc}", file=sys.stderr)
        return 2
    print(f"Downloaded and verified YuNet model: {output}")
    print(f"SHA-256: {MODEL_SHA256}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
