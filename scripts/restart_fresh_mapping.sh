#!/usr/bin/env bash
# Restart the managed stack: new nvblox map, VSLAM session and EKF state.
set -Eeuo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${XDG_RUNTIME_DIR:-/tmp}/damgc-vslam-mapping-${UID}/launcher.pid"
if [[ -f "${PID_FILE}" ]]; then
  previous_pid="$(cat "${PID_FILE}")"
  if [[ "${previous_pid}" =~ ^[0-9]+$ ]] && kill -0 "${previous_pid}" 2>/dev/null; then
    if ! tr '\0' ' ' <"/proc/${previous_pid}/cmdline" | rg -q 'run_vslam_mapping.sh'; then
      printf 'Launcher PID does not match the mapping script; refusing to signal it.\n' >&2
      exit 1
    fi
    printf 'Stopping the previous mapping session and finalizing its recording...\n'
    previous_group="$(ps -o pgid= -p "${previous_pid}" | tr -d ' ')"
    if [[ "${previous_group}" == "${previous_pid}" ]]; then
      # Signal the foreground teleop too: bash defers its trap while waiting
      # for that child, so signalling only the shell can leave mapping alive.
      kill -INT -- "-${previous_pid}"
    else
      kill -TERM "${previous_pid}"
      pkill -INT -P "${previous_pid}" || true
    fi
    stop_deadline=$((SECONDS + 180))
    while kill -0 "${previous_pid}" 2>/dev/null; do
      if (( SECONDS >= stop_deadline )); then
        printf 'Previous mapping session has not stopped; new session not started.\n' >&2
        exit 1
      fi
      sleep 1
    done
  fi
fi
printf 'Fresh session: previous maps/anchors are not loaded; initial source is STOP.\n'
exec env MAPPING_CAMERA_FPS="${MAPPING_CAMERA_FPS:-30}" MAPPING_SNAPSHOT= MAPPING_MODE=3D MAPPING_INITIAL_SCAN="${FRESH_INITIAL_SCAN:-1}" \
  MAPPING_SOURCE_MODE=STOP MAPPING_COOPERATIVE_ON_STARTUP=0 MAPPING_MANUAL_ONLY=0 MAPPING_SKIP_ANALYSIS=1 \
  bash "${REPO_ROOT}/scripts/run_vslam_mapping.sh"
