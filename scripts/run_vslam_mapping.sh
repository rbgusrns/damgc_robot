#!/usr/bin/env bash

set -Eeuo pipefail

REPO_ROOT="/home/maze/damgc_robot"
CONTAINER_NAME="${ISAAC_CONTAINER_NAME:-isaac_ros_dev-$(uname -m)-container}"
MAPPING_IMAGE="${ISAAC_MAPPING_IMAGE:-damgc-vslam-mapping:humble}"
MAPPING_DOCKERFILE="${REPO_ROOT}/docker/vslam_mapping.Dockerfile"
RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}/damgc-vslam-mapping-${UID}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/vslam_mapping_${RUN_ID}"
LAUNCHER_PID_FILE="${RUNTIME_ROOT}/launcher.pid"
CONTAINER_VSLAM_PID_FILE="/tmp/damgc_vslam_mapping_vslam.pid"
CONTAINER_RVIZ_PID_FILE="/tmp/damgc_vslam_mapping_rviz.pid"

HOST_PIDS=()
CONTAINER_STARTED_BY_US=0
CONTAINER_USER=""
CLEANING_UP=0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

mkdir -p "${RUNTIME_ROOT}" "${LOG_DIR}"

is_container_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" == "true" ]]
}

source_with_nounset_disabled() {
  local setup_file="$1"

  # ROS 2 Humble setup scripts probe optional variables without always using
  # ${name:-}. Temporarily disable nounset while sourcing them.
  set +u
  source "${setup_file}"
  set -u
}

stop_container_process() {
  local pid_file="$1"
  local signal="$2"
  local container_pid

  container_pid="$(docker exec "${CONTAINER_NAME}" sh -c "cat '${pid_file}' 2>/dev/null" 2>/dev/null || true)"
  if [[ "${container_pid}" =~ ^[0-9]+$ ]]; then
    docker exec "${CONTAINER_NAME}" kill "-${signal}" "${container_pid}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  if (( CLEANING_UP )); then
    return
  fi
  CLEANING_UP=1
  trap - EXIT INT TERM
  set +e

  printf '\nStopping mapping stack...\n'

  for pid in "${HOST_PIDS[@]}"; do
    # Each host launch runs in its own session, so signal its complete process
    # group rather than leaving camera/bridge child nodes behind.
    kill -INT -- "-${pid}" >/dev/null 2>&1 || true
  done

  if is_container_running; then
    if (( CONTAINER_STARTED_BY_US )); then
      docker stop --time 5 "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    else
      stop_container_process "${CONTAINER_RVIZ_PID_FILE}" INT
      stop_container_process "${CONTAINER_VSLAM_PID_FILE}" INT
    fi
  fi

  sleep 1
  for pid in "${HOST_PIDS[@]}"; do
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  done

  if is_container_running && (( ! CONTAINER_STARTED_BY_US )); then
    stop_container_process "${CONTAINER_RVIZ_PID_FILE}" TERM
    stop_container_process "${CONTAINER_VSLAM_PID_FILE}" TERM
    docker exec "${CONTAINER_NAME}" rm -f \
      "${CONTAINER_RVIZ_PID_FILE}" "${CONTAINER_VSLAM_PID_FILE}" \
      >/dev/null 2>&1 || true
  fi

  rm -f "${LAUNCHER_PID_FILE}"
  printf 'Stopped. Logs: %s\n' "${LOG_DIR}"
}

trap cleanup EXIT INT TERM

if [[ -f "${LAUNCHER_PID_FILE}" ]]; then
  existing_pid="$(<"${LAUNCHER_PID_FILE}")"
  if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    printf 'Mapping stack is already managed by PID %s.\n' "${existing_pid}" >&2
    exit 1
  fi
  rm -f "${LAUNCHER_PID_FILE}"
fi
printf '%s\n' "$$" > "${LAUNCHER_PID_FILE}"

if [[ ! -t 0 ]]; then
  printf 'Run this script from an interactive terminal for arrow-key input.\n' >&2
  exit 1
fi

for required_path in \
  "/opt/ros/humble/setup.bash" \
  "/home/maze/stm32_bridge_install/setup.bash" \
  "${REPO_ROOT}/install/setup.bash" \
  "${MAPPING_DOCKERFILE}"; do
  if [[ ! -e "${required_path}" ]]; then
    printf 'Required file is missing: %s\n' "${required_path}" >&2
    exit 1
  fi
done

if ! command -v setsid >/dev/null 2>&1; then
  printf 'Required command is missing: setsid\n' >&2
  exit 1
fi

source_with_nounset_disabled /opt/ros/humble/setup.bash

printf 'Logs: %s\n' "${LOG_DIR}"

if ! docker image inspect "${MAPPING_IMAGE}" >/dev/null 2>&1; then
  printf '[0/5] Building the persistent VSLAM mapping image (one time)...\n'
  docker build \
    --file "${MAPPING_DOCKERFILE}" \
    --tag "${MAPPING_IMAGE}" \
    "${REPO_ROOT}"
fi

printf '[1/5] Starting STM32 bridge...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  source /home/maze/stm32_bridge_install/setup.bash
  exec ros2 launch stm32_bridge stm32_bridge.launch.py \\
    port:=/dev/ttyTHS1 baudrate:=460800 namespace:=leader
" >"${LOG_DIR}/stm32_bridge.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[2/5] Starting RealSense...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  exec ros2 launch realsense2_camera rs_launch.py \\
    camera_namespace:=leader camera_name:=camera \\
    enable_color:=true enable_depth:=true \\
    enable_infra:=true enable_infra1:=true enable_infra2:=true \\
    enable_gyro:=false enable_accel:=false \\
    publish_tf:=true tf_publish_rate:=30.0
" >"${LOG_DIR}/realsense.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[3/5] Preparing Isaac ROS container...\n'
if ! is_container_running; then
  if [[ -z "${DISPLAY:-}" ]] || ! command -v gnome-terminal >/dev/null 2>&1; then
    printf 'A graphical terminal is required to start the Isaac ROS container.\n' >&2
    exit 1
  fi
  CONTAINER_STARTED_BY_US=1
  gnome-terminal \
    --title="Isaac ROS container (managed by mapping launcher)" \
    -- bash -lc "
      export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
      docker run -it --rm \\
        --privileged \\
        --network host \\
        --ipc host \\
        --pid host \\
        --runtime nvidia \\
        --name '${CONTAINER_NAME}' \\
        --workdir /workspaces/isaac_ros-dev \\
        --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \\
        -e DISPLAY='${DISPLAY}' \\
        -e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all \\
        -e NVIDIA_DRIVER_CAPABILITIES=all \\
        -e ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' \\
        -e USER='${USER}' \\
        -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \\
        -e HOST_USER_UID='$(id -u)' \\
        -e HOST_USER_GID='$(id -g)' \\
        -v /tmp/.X11-unix:/tmp/.X11-unix \\
        -v /tmp/:/tmp/ \\
        -v '${HOME}/.Xauthority:/home/admin/.Xauthority:rw' \\
        -v '${REPO_ROOT}:/workspaces/isaac_ros-dev' \\
        -v /etc/localtime:/etc/localtime:ro \\
        -v /usr/bin/tegrastats:/usr/bin/tegrastats \\
        -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \\
        -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api \\
        -v /usr/share/vpi3:/usr/share/vpi3 \\
        -v /dev/input:/dev/input \\
        '${MAPPING_IMAGE}' /bin/bash
    "
fi

container_deadline=$((SECONDS + 300))
until is_container_running; do
  if (( SECONDS >= container_deadline )); then
    printf 'Timed out waiting for container %s.\n' "${CONTAINER_NAME}" >&2
    exit 1
  fi
  sleep 1
done

container_user_deadline=$((SECONDS + 30))
while [[ -z "${CONTAINER_USER}" ]]; do
  CONTAINER_USER="$(
    docker exec "${CONTAINER_NAME}" getent passwd "$(id -u)" 2>/dev/null \
      | cut -d: -f1 \
      || true
  )"
  if [[ -n "${CONTAINER_USER}" ]]; then
    break
  fi
  if ! is_container_running || (( SECONDS >= container_user_deadline )); then
    printf 'Container user initialization did not finish for %s.\n' "${CONTAINER_NAME}" >&2
    exit 1
  fi
  sleep 1
done
printf '  container user: %s\n' "${CONTAINER_USER}"

if ! docker exec -u "${CONTAINER_USER}" "${CONTAINER_NAME}" bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  ros2 pkg prefix rescue_robot_bringup >/dev/null
  ros2 pkg prefix isaac_ros_visual_slam >/dev/null
  ros2 pkg prefix nvblox_ros >/dev/null
  ros2 pkg prefix robot_localization >/dev/null
'; then
  printf 'The container is missing a required ROS package or install_docker overlay.\n' >&2
  exit 1
fi

container_log_dir="/workspaces/isaac_ros-dev/log/vslam_mapping_${RUN_ID}"
docker exec -d -u "${CONTAINER_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  "${CONTAINER_NAME}" bash -lc '
    log_path="$1"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:${LD_LIBRARY_PATH}"
    echo "$$" > /tmp/damgc_vslam_mapping_vslam.pid
    exec ros2 launch rescue_robot_bringup visual_slam_nvblox_realsense.launch.py >>"${log_path}" 2>&1
  ' _ "${container_log_dir}/vslam_nvblox.log"

printf '[4/5] Starting RViz...\n'
docker exec -d -u "${CONTAINER_USER}" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  "${CONTAINER_NAME}" bash -lc '
    log_path="$1"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    echo "$$" > /tmp/damgc_vslam_mapping_rviz.pid
    exec rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz >>"${log_path}" 2>&1
  ' _ "${container_log_dir}/rviz.log"

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  until ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; do
    if (( SECONDS >= deadline )); then
      printf 'Timed out waiting for topic %s. Check %s\n' "${topic}" "${LOG_DIR}" >&2
      return 1
    fi
    sleep 1
  done
  printf '  ready: %s\n' "${topic}"
}

printf 'Waiting for the mapping data path...\n'
wait_for_topic "/leader/odom/raw" 30
wait_for_topic "/leader/camera/infra1/image_rect_raw" 45
wait_for_topic "/visual_slam/tracking/odometry" 120

printf '[5/5] Starting arrow-key control. Space stops; Ctrl-C shuts everything down.\n'
source_with_nounset_disabled "${REPO_ROOT}/install/setup.bash"
ros2 run rescue_robot_bringup arrow_key_teleop.py --ros-args \
  -p linear_speed:=0.08 \
  -p angular_speed:=0.25
