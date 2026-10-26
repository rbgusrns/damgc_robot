#!/usr/bin/env bash
# One mission: fresh map, arrival-only person checks, return to start.
set -Eeuo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export MAPPING_CHECKPOINT_DETECTION=1
exec bash "${REPO_ROOT}/scripts/run_map_only_exploration.sh" "$@"
