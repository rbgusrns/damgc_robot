#!/usr/bin/env bash
set -Eeuo pipefail
export MAPPING_MODE=2D
export MAPPING_INITIAL_SCAN=0
export VSLAM_ONLY=0
exec /home/maze/damgc_robot/scripts/run_vslam_mapping.sh "$@"
