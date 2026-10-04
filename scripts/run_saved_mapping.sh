#!/usr/bin/env bash
set -Eeuo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MAPPING_SNAPSHOT="${MAPPING_SNAPSHOT:-${REPO_ROOT}/data/maps/latest}"
export MAPPING_MODE=3D MAPPING_INITIAL_SCAN=0 VSLAM_ONLY=0
printf 'Resume saved map: place the robot at the saved physical anchor with the same heading.\n'
exec "${REPO_ROOT}/scripts/run_vslam_mapping.sh"
