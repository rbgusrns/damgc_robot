#!/usr/bin/env bash
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export MAPPING_SOLO_DELIVERY=1
exec bash "$root/scripts/run_survivor_exploration.sh" "$@"
