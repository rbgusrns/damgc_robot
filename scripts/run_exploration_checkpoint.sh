#!/usr/bin/env bash
# Fresh solo exploration, return facing the box, save mission, stop before grasp.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$root/scripts/run_staged_solo_delivery.py" --checkpoint-only
