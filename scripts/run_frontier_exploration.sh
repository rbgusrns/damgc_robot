#!/usr/bin/env bash

set -Eeuo pipefail

REPO_ROOT="/home/maze/damgc_robot"
CONTAINER_NAME="${ISAAC_CONTAINER_NAME:-isaac_ros_dev-$(uname -m)-container}"
RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}/damgc-vslam-mapping-${UID}"
RUN_ID_FILE="${RUNTIME_ROOT}/active_run_id"

if [[ ! -s "${RUN_ID_FILE}" ]]; then
  printf 'No active mapping run. Start run_vslam_mapping.sh first.\n' >&2
  exit 1
fi

RUN_ID="$(<"${RUN_ID_FILE}")"
LOG_DIR="${REPO_ROOT}/log/vslam_mapping_${RUN_ID}"
mkdir -p "${LOG_DIR}/ros_logs"

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

stop_robot() {
  timeout 3 ros2 param set /leader/command_selector source_mode STOP \
    >>"${LOG_DIR}/frontier_exploration.log" 2>&1 || true
}
trap stop_robot EXIT INT TERM

{
  printf 'run_id=%s\nstart_time=%s\n' "${RUN_ID}" "$(date --iso-8601=seconds)"
  printf 'Reading startup-configured Nav2 speed limits (0.00..0.10 m/s, angular ceiling 0.20 rad/s).\n'
  ros2 param get /controller_server FollowPath.min_vel_x
  ros2 param get /controller_server FollowPath.max_vel_x
  ros2 param get /controller_server FollowPath.max_speed_xy
  ros2 param get /controller_server FollowPath.max_vel_theta
  ros2 param get /controller_server FollowPath.min_speed_theta
  printf 'Capturing active Nav2 and drivetrain parameters/topics.\n'
  printf 'nodes and topic connections\n' >"${LOG_DIR}/diagnostic_snapshot.txt"
  ros2 node list >>"${LOG_DIR}/diagnostic_snapshot.txt" 2>&1 || true
  for topic in \
    /nav2/cmd_vel /leader/cmd_vel /leader/odom/raw \
    /global_costmap/costmap /global_costmap/costmap_updates \
    /local_costmap/costmap /local_costmap/costmap_updates \
    /nvblox_node/static_map_slice /evaluation; do
    {
      printf '\n--- %s ---\n' "${topic}"
      ros2 topic info -v "${topic}"
    } >>"${LOG_DIR}/diagnostic_snapshot.txt" 2>&1 || true
  done
  for node in \
    /controller_server /planner_server /bt_navigator \
    /global_costmap/global_costmap /local_costmap/local_costmap \
    /nvblox_node /robot_self_filter /leader/command_selector \
    /leader/stm32_bridge; do
    safe_name="${node//\//_}"
    ros2 param dump "${node}" >"${LOG_DIR}/parameters${safe_name}.yaml" 2>&1 || true
  done
  printf 'Selecting NAV2 command source.\n'
  ros2 param set /leader/command_selector source_mode NAV2
  printf 'Launching frontier explorer inside %s.\n' "${CONTAINER_NAME}"
  docker exec -i -u admin \
    -e ROS_LOG_DIR="/workspaces/isaac_ros-dev/log/vslam_mapping_${RUN_ID}/ros_logs" \
    "${CONTAINER_NAME}" bash -lc '
      set +u
      source /opt/ros/humble/setup.bash
      source /workspaces/isaac_ros-dev/install_docker/setup.bash
      set -u
      ros2 launch rescue_robot_bringup frontier_exploration.launch.py
    '
} 2>&1 | tee -a "${LOG_DIR}/frontier_exploration.log"
