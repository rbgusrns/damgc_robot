#!/usr/bin/env bash
# Desktop keyboard for existing grasped robots; reuses motor/drive processes.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
set +u
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
peer_pid=''
cleanup() {
  if [[ -n "$peer_pid" ]]; then
    kill -INT "$peer_pid" 2>/dev/null || true
    wait "$peer_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 run cooperative_manual manual_straight --ros-args -r __ns:=/leader \
  -p role:=leader -p motion_enabled:=true -p odom_topic:=/leader/odom/raw \
  -p gripper_enabled:=true -p close_raw:=477 -p gripper_settle:=9.0 \
  > /tmp/leader_manual_straight.log 2>&1 &
peer_pid=$!
printf 'Leader manual straight transport\nFollower must be prepared and enabled first.\ne = enable; hold UP/DOWN = move; Space = stop both; q = exit.\n'
ros2 run cooperative_manual manual_straight_keys
