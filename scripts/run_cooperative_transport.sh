#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROLE="${1:-}"

usage() {
  printf 'Usage: %s leader|follower\n' "${0##*/}"
  printf '\nEnvironment overrides:\n'
  printf '  ROS_DOMAIN_ID=42             Shared DDS domain\n'
  printf '  COOP_PEER_IP=192.168.0.7     Optional peer ping check\n'
  printf '  COOP_USE_STM32_BRIDGE=1      Set 0 for network-only testing\n'
  printf '  COOP_I2C_DEVICE=/dev/i2c-7   STM32 I2C device\n'
  printf '  COOP_I2C_ADDRESS=66          STM32 7-bit address\n'
  printf '  COOP_I2C_WRITE_ENABLED=1     Set 0 for receive-only testing\n'
  printf '  COOP_DISCOVERY_TIMEOUT=30    Leader heartbeat wait, seconds\n'
}

if [[ "${ROLE}" != "leader" && "${ROLE}" != "follower" ]]; then
  usage >&2
  exit 2
fi

normalize_bool() {
  case "$1" in
    1|true|TRUE|yes|YES) printf 'true' ;;
    0|false|FALSE|no|NO) printf 'false' ;;
    *)
      printf 'Expected a boolean value, got: %s\n' "$1" >&2
      return 2
      ;;
  esac
}

source_with_nounset_disabled() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble setup was not found.\n' >&2
  exit 1
fi
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  printf 'Workspace is not built. Run colcon build first.\n' >&2
  exit 1
fi

source_with_nounset_disabled /opt/ros/humble/setup.bash
source_with_nounset_disabled "${REPO_ROOT}/install/setup.bash"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros2_dds_env.sh"

if ! [[ "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID > 232 )); then
  printf 'ROS_DOMAIN_ID must be an integer from 0 to 232.\n' >&2
  exit 2
fi

use_bridge="$(normalize_bool "${COOP_USE_STM32_BRIDGE:-1}")"
i2c_write_enabled="$(normalize_bool "${COOP_I2C_WRITE_ENABLED:-1}")"
i2c_device="${COOP_I2C_DEVICE:-/dev/i2c-7}"
i2c_address="${COOP_I2C_ADDRESS:-66}"
discovery_timeout="${COOP_DISCOVERY_TIMEOUT:-30}"

if ! [[ "${i2c_address}" =~ ^[0-9]+$ ]] || (( i2c_address < 0 || i2c_address > 127 )); then
  printf 'COOP_I2C_ADDRESS must be an integer from 0 to 127.\n' >&2
  exit 2
fi
if ! [[ "${discovery_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'COOP_DISCOVERY_TIMEOUT must be a positive integer.\n' >&2
  exit 2
fi

if [[ -n "${COOP_PEER_IP:-}" ]]; then
  printf 'Checking peer %s...\n' "${COOP_PEER_IP}"
  if ! ping -c 1 -W 2 -- "${COOP_PEER_IP}" >/dev/null; then
    printf 'Peer ping failed: %s\n' "${COOP_PEER_IP}" >&2
    exit 1
  fi
fi

common_launch_args=(
  "use_stm32_bridge:=${use_bridge}"
  "i2c_device:=${i2c_device}"
  "i2c_address:=${i2c_address}"
  "i2c_write_enabled:=${i2c_write_enabled}"
)

printf 'Role: %s, DDS domain: %s, STM32 bridge: %s, I2C write: %s\n' \
  "${ROLE}" "${ROS_DOMAIN_ID}" "${use_bridge}" "${i2c_write_enabled}"

if [[ "${ROLE}" == "follower" ]]; then
  printf '\nFollower starts fail-closed. After Leader reports COOPERATING, enable motion from another terminal:\n'
  printf '  ros2 service call /follower/velocity_guard/enable std_srvs/srv/SetBool "{data: true}"\n\n'
  exec ros2 launch follower_supply_perception \
    follower_cooperation_drive.launch.py \
    guard_enabled_on_startup:=false \
    "${common_launch_args[@]}"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/cooperative_transport_${RUN_ID}"
mkdir -p "${LOG_DIR}"
leader_launch_pid=""

cleanup() {
  trap - EXIT INT TERM
  timeout 2s ros2 service call /cooperation/enable std_srvs/srv/SetBool \
    '{data: false}' >/dev/null 2>&1 || true
  if [[ -n "${leader_launch_pid}" ]] && kill -0 "${leader_launch_pid}" 2>/dev/null; then
    kill -INT -- "-${leader_launch_pid}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "${leader_launch_pid}" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "${leader_launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${leader_launch_pid}" 2>/dev/null || true
    fi
    wait "${leader_launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch rescue_robot_bringup leader_cooperation_drive.launch.py \
  "${common_launch_args[@]}" >"${LOG_DIR}/leader_cooperation.log" 2>&1 &
leader_launch_pid=$!

service_deadline=$((SECONDS + discovery_timeout))
until ros2 service list 2>/dev/null | grep -Fxq '/cooperation/enable'; do
  if ! kill -0 "${leader_launch_pid}" 2>/dev/null; then
    printf 'Leader launch exited. Check %s/leader_cooperation.log\n' "${LOG_DIR}" >&2
    exit 1
  fi
  if (( SECONDS >= service_deadline )); then
    printf 'Timed out waiting for /cooperation/enable. Check %s\n' "${LOG_DIR}" >&2
    exit 1
  fi
  sleep 1
done

printf 'Waiting up to %ss for Follower heartbeat...\n' "${discovery_timeout}"
if ! timeout "${discovery_timeout}s" \
  ros2 topic echo /follower/status std_msgs/msg/String --once >/dev/null 2>&1; then
  printf 'Follower heartbeat was not discovered. Check DDS settings and start the Follower first.\n' >&2
  exit 1
fi

enable_result="$(ros2 service call /cooperation/enable std_srvs/srv/SetBool \
  '{data: true}' 2>&1)"
printf '%s\n' "${enable_result}"
if [[ "${enable_result}" != *"success=True"* && \
      "${enable_result}" != *"success=true"* && \
      "${enable_result}" != *"success: true"* ]]; then
  printf 'Failed to enable cooperative forwarding.\n' >&2
  exit 1
fi

printf '\nFollower heartbeat detected; cooperative forwarding is enabled.\n'
printf 'The Follower still requires its velocity guard to be enabled explicitly.\n'
printf 'Starting the existing Leader keyboard controller. Log: %s\n' "${LOG_DIR}"
ros2 run rescue_robot_bringup arrow_key_teleop.py --ros-args \
  -p linear_speed:=0.08 \
  -p angular_speed:=0.25
