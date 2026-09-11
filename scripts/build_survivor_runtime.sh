#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${SURVIVOR_IMAGE:-damgc-survivor-yolo:humble}"
DOCKERFILE="${REPO_ROOT}/docker/survivor_runtime.Dockerfile"

docker build \
  --file "${DOCKERFILE}" \
  --tag "${IMAGE}" \
  "${REPO_ROOT}"

echo "Built ${IMAGE}"
