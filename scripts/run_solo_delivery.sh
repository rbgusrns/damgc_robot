#!/usr/bin/env bash
# Preserve the proven mapping process; delivery components start after return only.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if (( $# )); then
  printf 'Usage: %s (fresh physical start only)\n' "$0" >&2
  exit 2
fi
exec python3 "$root/scripts/run_staged_solo_delivery.py"
