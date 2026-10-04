#!/usr/bin/env bash
# Software/drive-only peer. Never starts grasp, lift or Dynamixel.
set -Eeuo pipefail
role="${1:-}"
if [[ "$role" != leader && "$role" != follower ]]; then
  printf 'Usage: %s leader|follower\n' "${0##*/}" >&2
  exit 2
fi
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${repo_root}/${COOP_INSTALL_DIR:-install}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
args=("role:=${role}")
if [[ "$role" == follower ]]; then
  args+=("follower_drive:=${COOP_FOLLOWER_DRIVE:-true}"
         "use_stm32_bridge:=${COOP_USE_STM32_BRIDGE:-true}"
         "i2c_device:=${COOP_I2C_DEVICE:-/dev/i2c-7}"
         "i2c_address:=${COOP_I2C_ADDRESS:-66}")
fi
cd "${repo_root}"
exec ros2 launch cooperative_transport manual_transport.launch.py "${args[@]}"
