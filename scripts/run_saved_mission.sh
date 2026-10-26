#!/usr/bin/env bash
# Restore the checkpoint pose/map/people and wait for cooperative preparation.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
snapshot="$(realpath -- "${1:-$root/data/maps/latest_mission}")"
PYTHONPATH="$root/scripts${PYTHONPATH:+:$PYTHONPATH}" python3 -c 'import sys; from mission_checkpoint import load_checkpoint; m,_,_=load_checkpoint(sys.argv[1]); print("Place robot at saved return anchor:",m["anchor"])' "$snapshot"
export MAPPING_SNAPSHOT="$snapshot" MAPPING_RESTORE_SURVIVORS=1
export MAPPING_AUTOMATIC_EXPLORATION=0 MAPPING_INITIAL_SCAN=0 MAPPING_CHECKPOINT_DETECTION=0 MAPPING_SOLO_DELIVERY=0
export MAPPING_RECORD_BAG=0 MAPPING_SOURCE_MODE=STOP MAPPING_COOPERATIVE_ON_STARTUP=0 MAPPING_MANUAL_ONLY=0
exec bash "$root/scripts/run_saved_mapping.sh"
