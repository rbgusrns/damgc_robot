#!/usr/bin/env bash
# One command: stop perception, reset mapping, verify inputs, scan and explore.
set -Eeuo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ "${1:-}" == "--retry" ]]; then
  # A scan may already have rotated the robot. Preserve the original square
  # and return pose while resetting the odometry/map coordinate frame.
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export MAPPING_START_POSE_FILE="/tmp/damgc-exploration-start-${UID}.json"
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  python3 "${REPO_ROOT}/scripts/capture_exploration_anchor.py" --output "${MAPPING_START_POSE_FILE}"
elif [[ $# -gt 0 ]]; then
  printf 'Usage: %s [--retry]\n' "$0" >&2
  exit 1
else
  if [[ -n "${MAPPING_START_POSE_FILE:-}" && ! -s "${MAPPING_START_POSE_FILE}" ]]; then
    printf 'The supplied start pose file is missing.\n' >&2
    exit 1
  fi
fi

# Stop previous mission/perception processes before resetting their map frame.
python3 - <<'PY'
import os
from pathlib import Path
import signal
import time

targets = []
for proc in Path('/proc').glob('[0-9]*'):
    try:
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        args = [v.decode(errors='replace') for v in argv if v]
        if any(a.endswith(('/frontier_explorer.py', '/initial_map_scan.py',
                            '/mapping_autorun.py', '/survivor_pipeline.launch.py'))
               or a == 'survivor_pipeline.launch.py' for a in args):
            targets.append(int(proc.name))
    except (OSError, ValueError):
        pass
for pid in targets:
    try: os.kill(pid, signal.SIGINT)
    except ProcessLookupError: pass
time.sleep(2)
for pid in targets:
    try: os.kill(pid, signal.SIGTERM)
    except ProcessLookupError: pass
PY
while IFS= read -r container_name; do
  case "${container_name}" in
    damgc-survivor-detector-*|damgc-rgbd-test) docker stop --timeout 5 "${container_name}" ;;
  esac
done < <(docker ps --format '{{.Names}}')

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export MAPPING_CAMERA_FPS="${MAPPING_CAMERA_FPS:-30}"
export MAPPING_AUTOMATIC_EXPLORATION=1
export MAPPING_RECORD_BAG="${MAPPING_RECORD_BAG:-0}"
export FRESH_INITIAL_SCAN=1
exec bash "${REPO_ROOT}/scripts/restart_fresh_mapping.sh"
