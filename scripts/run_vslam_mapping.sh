#!/usr/bin/env bash

set -Eeuo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${ISAAC_CONTAINER_NAME:-isaac_ros_dev-$(uname -m)-container}"
MAPPING_IMAGE="${ISAAC_MAPPING_IMAGE:-damgc-vslam-mapping:humble}"
MAPPING_DOCKERFILE="${REPO_ROOT}/docker/vslam_mapping.Dockerfile"
RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}/damgc-vslam-mapping-${UID}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
MAPPING_MODE="${MAPPING_MODE:-3D}"
MAPPING_SNAPSHOT="${MAPPING_SNAPSHOT:-}"
CONTAINER_SNAPSHOT=""
RUN_PREFIX="vslam_mapping"
if [[ "${MAPPING_MODE}" == "2D" ]]; then RUN_PREFIX="mapping_2d"; fi
LOG_DIR="${REPO_ROOT}/log/${RUN_PREFIX}_${RUN_ID}"
LAUNCHER_PID_FILE="${RUNTIME_ROOT}/launcher.pid"
CONTAINER_VSLAM_PID_FILE="/tmp/damgc_vslam_mapping_vslam.pid"
CONTAINER_RVIZ_PID_FILE="/tmp/damgc_vslam_mapping_rviz.pid"
CONTAINER_BAG_PID_FILE="/tmp/damgc_vslam_mapping_bag.pid"
CONTAINER_AUTO_PID_FILE="/tmp/damgc_vslam_mapping_auto.pid"
BAG_DIR="${REPO_ROOT}/data/${RUN_PREFIX}_${RUN_ID}"
CONTAINER_BAG_DIR="/workspaces/isaac_ros-dev/data/${RUN_PREFIX}_${RUN_ID}"

if [[ "${MAPPING_MODE}" != "2D" && "${MAPPING_MODE}" != "3D" ]]; then
  printf "MAPPING_MODE must be 2D or 3D.\n" >&2
  exit 1
fi

if [[ -n "${MAPPING_SNAPSHOT}" ]]; then
  if [[ "${MAPPING_MODE}" != "3D" || "${VSLAM_ONLY:-0}" == "1" ]]; then
    printf 'Saved map resume requires 3D nvblox mapping.\n' >&2
    exit 1
  fi
  MAPPING_SNAPSHOT="$(realpath "${MAPPING_SNAPSHOT}")"
  if [[ "${MAPPING_SNAPSHOT}" != "${REPO_ROOT}/"* || ! -s "${MAPPING_SNAPSHOT}/manifest.json" || ! -s "${MAPPING_SNAPSHOT}/map.nvblx" ]]; then
    printf 'Snapshot must contain manifest.json and map.nvblx inside the repository.\n' >&2
    exit 1
  fi
  CONTAINER_SNAPSHOT="/workspaces/isaac_ros-dev/${MAPPING_SNAPSHOT#"${REPO_ROOT}/"}"
  MAPPING_INITIAL_SCAN=0
fi

HOST_PIDS=()
CONTAINER_STARTED_BY_US=0
CONTAINER_USER=""
CLEANING_UP=0
LAUNCHER_OWNED=0
BAG_STARTED=0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
VSLAM_HEADLESS="${VSLAM_HEADLESS:-0}"
VSLAM_ONLY="${VSLAM_ONLY:-0}"
SELF_FILTER_ENABLED="${SELF_FILTER_ENABLED:-1}"
MAPPING_SOURCE_MODE="${MAPPING_SOURCE_MODE:-STOP}"
MAPPING_INITIAL_SCAN="${MAPPING_INITIAL_SCAN:-0}"
MAPPING_AUTOMATIC_EXPLORATION="${MAPPING_AUTOMATIC_EXPLORATION:-0}"
MAPPING_RECORD_BAG="${MAPPING_RECORD_BAG:-1}"
if [[ "${MAPPING_RECORD_BAG}" != "0" && "${MAPPING_RECORD_BAG}" != "1" ]]; then
  printf 'MAPPING_RECORD_BAG must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${MAPPING_AUTOMATIC_EXPLORATION}" != "0" && "${MAPPING_AUTOMATIC_EXPLORATION}" != "1" ]]; then
  printf 'MAPPING_AUTOMATIC_EXPLORATION must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${MAPPING_AUTOMATIC_EXPLORATION}" == "1" && ( "${MAPPING_INITIAL_SCAN}" != "1" || "${MAPPING_MODE}" != "3D" || "${VSLAM_ONLY}" == "1" ) ]]; then
  printf 'Automatic exploration requires fresh nvblox mapping and initial scan.\n' >&2
  exit 1
fi
MAPPING_CAMERA_FPS="${MAPPING_CAMERA_FPS:-30}"
if [[ "${MAPPING_CAMERA_FPS}" != "15" && "${MAPPING_CAMERA_FPS}" != "30" ]]; then
  printf "MAPPING_CAMERA_FPS must be 15 or 30.\n" >&2
  exit 1
fi
MAPPING_COOPERATIVE_ON_STARTUP="${MAPPING_COOPERATIVE_ON_STARTUP:-0}"
MAPPING_MANUAL_ONLY="${MAPPING_MANUAL_ONLY:-0}"
NAV2_GOAL_SELECTION_ARG="true"
if [[ "${MAPPING_COOPERATIVE_ON_STARTUP}" == "1" || "${MAPPING_MANUAL_ONLY}" == "1" ]]; then NAV2_GOAL_SELECTION_ARG="false"; fi
HOST_XAUTHORITY="${XAUTHORITY:-/run/user/${UID}/gdm/Xauthority}"
DEPTH_CLIP_DISTANCE_M="${DEPTH_CLIP_DISTANCE_M:-4.0}"
STM32_I2C_DEVICE="${STM32_I2C_DEVICE:-/dev/i2c-7}"
STM32_I2C_ADDRESS="${STM32_I2C_ADDRESS:-66}"
STM32_I2C_POLL_HZ="${STM32_I2C_POLL_HZ:-500.0}"
STM32_I2C_WRITE_ENABLED="${STM32_I2C_WRITE_ENABLED:-1}"

if [[ "${VSLAM_HEADLESS}" != "0" && "${VSLAM_HEADLESS}" != "1" ]] || \
  [[ "${SELF_FILTER_ENABLED}" != "0" && "${SELF_FILTER_ENABLED}" != "1" ]] || \
  [[ "${VSLAM_ONLY}" != "0" && "${VSLAM_ONLY}" != "1" ]]; then
  printf 'VSLAM_HEADLESS, SELF_FILTER_ENABLED, and VSLAM_ONLY must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${MAPPING_INITIAL_SCAN}" != "0" && "${MAPPING_INITIAL_SCAN}" != "1" ]]; then
  printf 'MAPPING_INITIAL_SCAN must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${VSLAM_ONLY}" == "1" && "${VSLAM_HEADLESS}" != "1" ]]; then
  printf 'VSLAM_ONLY=1 also requires VSLAM_HEADLESS=1.\n' >&2
  exit 1
fi
if [[ "${STM32_I2C_WRITE_ENABLED}" != "0" && \
      "${STM32_I2C_WRITE_ENABLED}" != "1" ]]; then
  printf 'STM32_I2C_WRITE_ENABLED must be 0 or 1.\n' >&2
  exit 1
fi
if [[ "${MAPPING_SOURCE_MODE}" != "STOP" && \
  "${MAPPING_SOURCE_MODE}" != "TELEOP" && \
  "${MAPPING_SOURCE_MODE}" != "APPROACH" && \
  "${MAPPING_SOURCE_MODE}" != "NAV2" ]]; then
  printf 'MAPPING_SOURCE_MODE must be STOP, TELEOP, APPROACH, or NAV2.\n' >&2
  exit 1
fi
if [[ "${MAPPING_COOPERATIVE_ON_STARTUP}" != "0" && "${MAPPING_COOPERATIVE_ON_STARTUP}" != "1" ]]; then
  printf "MAPPING_COOPERATIVE_ON_STARTUP must be 0 or 1.\n" >&2
  exit 1
fi
if [[ "${MAPPING_MANUAL_ONLY}" != "0" && "${MAPPING_MANUAL_ONLY}" != "1" ]]; then
  printf "MAPPING_MANUAL_ONLY must be 0 or 1.\n" >&2
  exit 1
fi
STM32_I2C_WRITE_ARG="false"
if [[ "${STM32_I2C_WRITE_ENABLED}" == "1" ]]; then
  STM32_I2C_WRITE_ARG="true"
fi

mkdir -p "${RUNTIME_ROOT}" "${LOG_DIR}"
# Acquire ownership before installing cleanup traps or touching shared PID files.
exec 9>"${RUNTIME_ROOT}/launcher.lock"
if ! flock -n 9; then
  printf 'A mapping session is already running; no processes were changed.\n' >&2
  exit 1
fi

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
    docker exec "${CONTAINER_NAME}" python3 /workspaces/isaac_ros-dev/scripts/mapping_processes.py \
      signal "${container_pid}" "${signal}" >/dev/null 2>&1 || true
  fi
}

container_process_running() {
  local pid_file="$1"
  local container_pid

  container_pid="$(docker exec "${CONTAINER_NAME}" sh -c "cat '${pid_file}' 2>/dev/null" 2>/dev/null || true)"
  [[ "${container_pid}" =~ ^[0-9]+$ ]] && \
    docker exec "${CONTAINER_NAME}" python3 /workspaces/isaac_ros-dev/scripts/mapping_processes.py \
      alive "${container_pid}" >/dev/null 2>&1
}

finalize_bag() {
  local deadline

  if (( ! BAG_STARTED )) || ! is_container_running; then
    return
  fi

  printf 'Finalizing rosbag...\n'
  stop_container_process "${CONTAINER_BAG_PID_FILE}" INT
  deadline=$((SECONDS + 20))
  while container_process_running "${CONTAINER_BAG_PID_FILE}"; do
    if (( SECONDS >= deadline )); then
      printf 'Rosbag did not stop within 20 seconds; sending TERM.\n' >&2
      stop_container_process "${CONTAINER_BAG_PID_FILE}" TERM
      deadline=$((SECONDS + 5))
      while container_process_running "${CONTAINER_BAG_PID_FILE}" && (( SECONDS < deadline )); do
        sleep 1
      done
      break
    fi
    sleep 1
  done

  if [[ -f "${BAG_DIR}/metadata.yaml" ]]; then
    if [[ "${MAPPING_SKIP_ANALYSIS:-0}" == "1" ]]; then
      printf 'Bag saved; deferred trajectory analysis: %s\n' "${BAG_DIR}"
      return
    fi
    printf 'Analyzing recorded trajectories...\n'
    docker exec -u "${CONTAINER_USER}" \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
      "${CONTAINER_NAME}" setsid --wait bash -lc '
        source /opt/ros/humble/setup.bash
        source /workspaces/isaac_ros-dev/install_docker/setup.bash
        python3 /workspaces/isaac_ros-dev/scripts/analyze_vslam_bag.py "$1"
      ' _ "${CONTAINER_BAG_DIR}" >"${LOG_DIR}/bag_analysis.log" 2>&1 || {
        printf 'Bag analysis failed; see %s\n' "${LOG_DIR}/bag_analysis.log" >&2
        return
      }
    printf 'Bag: %s\n' "${BAG_DIR}"
    printf 'Analysis: %s/analysis.md\n' "${BAG_DIR}"
  else
    printf 'Rosbag metadata was not created; see %s/rosbag.log\n' "${LOG_DIR}" >&2
  fi
}

cleanup() {
  if (( CLEANING_UP || ! LAUNCHER_OWNED )); then
    return
  fi
  CLEANING_UP=1
  trap - EXIT INT TERM
  set +e

  printf '\nStopping mapping stack...\n'

  # The coordinator owns exploration actions. Stop it before stopping sensors.
  if is_container_running; then
    stop_container_process "${CONTAINER_AUTO_PID_FILE}" INT
  fi

  if [[ -n "${CHECKPOINT_CONTAINER_NAME:-}" ]]; then
    docker stop --time 3 "${CHECKPOINT_CONTAINER_NAME}" >/dev/null 2>&1 || true
  fi

  # Let rosbag receive a clean SIGINT and write metadata before its topics or
  # container disappear.
  if (( BAG_STARTED )) && is_container_running; then
    printf 'Capturing a 5-second stationary tail for drift measurement...\n'
    sleep 5
  fi
  finalize_bag

  for pid in "${HOST_PIDS[@]}"; do
    # Each host launch runs in its own session, so signal its complete process
    # group rather than leaving camera/bridge child nodes behind.
    python3 "${REPO_ROOT}/scripts/mapping_processes.py" signal "${pid}" INT >/dev/null 2>&1 || true
  done

  if is_container_running; then
    if (( CONTAINER_STARTED_BY_US )); then
      docker stop --time 5 "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    else
      stop_container_process "${CONTAINER_RVIZ_PID_FILE}" INT
      stop_container_process "${CONTAINER_VSLAM_PID_FILE}" INT
    fi
  fi

  sleep 3
  for pid in "${HOST_PIDS[@]}"; do
    python3 "${REPO_ROOT}/scripts/mapping_processes.py" signal "${pid}" TERM >/dev/null 2>&1 || true
    python3 "${REPO_ROOT}/scripts/mapping_processes.py" signal "${pid}" KILL >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
  done

  if is_container_running && (( ! CONTAINER_STARTED_BY_US )); then
    stop_container_process "${CONTAINER_RVIZ_PID_FILE}" TERM
    stop_container_process "${CONTAINER_VSLAM_PID_FILE}" TERM
    stop_container_process "${CONTAINER_AUTO_PID_FILE}" TERM
    sleep 1
    stop_container_process "${CONTAINER_RVIZ_PID_FILE}" KILL
    stop_container_process "${CONTAINER_VSLAM_PID_FILE}" KILL
    stop_container_process "${CONTAINER_AUTO_PID_FILE}" KILL
    docker exec "${CONTAINER_NAME}" rm -f \
      "${CONTAINER_RVIZ_PID_FILE}" "${CONTAINER_VSLAM_PID_FILE}" "${CONTAINER_BAG_PID_FILE}" "${CONTAINER_AUTO_PID_FILE}" \
      >/dev/null 2>&1 || true
  fi

  rm -f "${LAUNCHER_PID_FILE}"
  printf 'Stopped. Logs: %s\n' "${LOG_DIR}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ -f "${LAUNCHER_PID_FILE}" ]]; then
  existing_pid="$(<"${LAUNCHER_PID_FILE}")"
  if [[ "${existing_pid}" =~ ^[0-9]+$ ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    printf 'Mapping stack is already managed by PID %s.\n' "${existing_pid}" >&2
    exit 1
  fi
  rm -f "${LAUNCHER_PID_FILE}"
fi
python3 "${REPO_ROOT}/scripts/mapping_processes.py" conflicts
printf '%s\n' "$$" > "${LAUNCHER_PID_FILE}"
LAUNCHER_OWNED=1

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

if [[ "${VSLAM_HEADLESS}" == "0" && ! -r "${HOST_XAUTHORITY}" ]]; then
  printf 'Cannot read X11 authority file: %s\n' "${HOST_XAUTHORITY}" >&2
  exit 1
fi

source_with_nounset_disabled /opt/ros/humble/setup.bash

printf 'Logs: %s\n' "${LOG_DIR}"

if ! docker image inspect "${MAPPING_IMAGE}" >/dev/null 2>&1; then
  printf '[0/7] Building the persistent VSLAM mapping image (one time)...\n'
  docker build \
    --file "${MAPPING_DOCKERFILE}" \
    --tag "${MAPPING_IMAGE}" \
    "${REPO_ROOT}"
fi

printf '[1/7] Starting STM32 bridge...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  source /home/maze/stm32_bridge_install/setup.bash
  exec ros2 launch stm32_bridge stm32_bridge.launch.py \\
    transport:=i2c \\
    i2c_device:='${STM32_I2C_DEVICE}' \\
    i2c_address:='${STM32_I2C_ADDRESS}' \\
    i2c_poll_hz:='${STM32_I2C_POLL_HZ}' \\
    i2c_write_enabled:='${STM32_I2C_WRITE_ARG}' \\
    namespace:=leader
" >"${LOG_DIR}/stm32_bridge.log" 2>&1 &
HOST_PIDS+=("$!")

if [[ "${MAPPING_MODE}" == "2D" ]]; then
printf '[2/7] Starting depth-only RealSense...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  source /opt/ros/humble/setup.bash
  exec ros2 launch realsense2_camera rs_launch.py \\
    camera_namespace:=leader camera_name:=camera \\
    enable_color:=false enable_depth:=true \\
    enable_infra:=false enable_infra1:=false enable_infra2:=false \\
    enable_sync:=false align_depth.enable:=false \\
    depth_module.depth_profile:=424x240x15 \\
    clip_distance:='${DEPTH_CLIP_DISTANCE_M}' \\
    enable_gyro:=false enable_accel:=false publish_tf:=true tf_publish_rate:=0.0
" >"${LOG_DIR}/realsense.log" 2>&1 &
HOST_PIDS+=("$!")
else
printf '[2/7] Starting RealSense...\n'
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  # single shared D435 source for VSLAM and future survivor pipeline.
  exec ros2 launch realsense2_camera rs_launch.py \\
    camera_namespace:=leader camera_name:=camera \\
    enable_color:=true enable_depth:=true \\
    enable_infra:=true enable_infra1:=true enable_infra2:=true \\
    depth_module.depth_profile:=848x480x'${MAPPING_CAMERA_FPS}' \\
    depth_module.infra_profile:=848x480x'${MAPPING_CAMERA_FPS}' \\
    rgb_camera.color_profile:=640x480x'${MAPPING_CAMERA_FPS}' \\
    enable_sync:=true align_depth.enable:=true \\
    clip_distance:='${DEPTH_CLIP_DISTANCE_M}' \\
    enable_gyro:=false enable_accel:=false \\
    publish_tf:=true tf_publish_rate:=30.0
" >"${LOG_DIR}/realsense.log" 2>&1 &
HOST_PIDS+=("$!")

fi

printf '[3/7] Starting Leader command selector in %s mode...\n' "${MAPPING_SOURCE_MODE}"
setsid bash -lc "
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
  export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
  export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}'
  export FASTDDS_BUILTIN_TRANSPORTS='${FASTDDS_BUILTIN_TRANSPORTS}'
  source /opt/ros/humble/setup.bash
  source '${REPO_ROOT}/install/setup.bash'
  exec ros2 launch leader_command_selector command_selector.launch.py \\
    source_mode:='${MAPPING_SOURCE_MODE}' \
    enable_nav2_goal_selection:='${NAV2_GOAL_SELECTION_ARG}'
" >"${LOG_DIR}/leader_command_selector.log" 2>&1 &
HOST_PIDS+=("$!")

printf '[4/7] Preparing Isaac ROS container...\n'
if ! is_container_running && docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  existing_image="$(docker inspect -f '{{.Image}}' "${CONTAINER_NAME}")"
  requested_image="$(docker image inspect -f '{{.Id}}' "${MAPPING_IMAGE}")"
  existing_workspace="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/workspaces/isaac_ros-dev"}}{{.Source}}{{end}}{{end}}' "${CONTAINER_NAME}")"
  if [[ "${existing_image}" != "${requested_image}" || "${existing_workspace}" != "${REPO_ROOT}" ]]; then
    printf 'Stopped container does not match the mapping image/workspace; refusing to reuse it.\n' >&2
    exit 1
  fi
  printf '  starting the existing stopped mapping container...\n'
  docker start "${CONTAINER_NAME}" >/dev/null
  CONTAINER_STARTED_BY_US=1
fi
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
        -e XAUTHORITY=/tmp/host.Xauthority \\
        -e QT_X11_NO_MITSHM=1 \\
        -e NVIDIA_VISIBLE_DEVICES=nvidia.com/gpu=all,nvidia.com/pva=all \\
        -e NVIDIA_DRIVER_CAPABILITIES=all \\
        -e ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' \\
        -e USER='${USER}' \\
        -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \\
        -e HOST_USER_UID='$(id -u)' \\
        -e HOST_USER_GID='$(id -g)' \\
        -v /tmp/.X11-unix:/tmp/.X11-unix \\
        -v /tmp/:/tmp/ \\
        -v '${HOST_XAUTHORITY}:/tmp/host.Xauthority:ro' \\
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

if ! docker exec -u "${CONTAINER_USER}" -e DAMGC_MAPPING_MODE="${MAPPING_MODE}" "${CONTAINER_NAME}" setsid --wait bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  ros2 pkg prefix rescue_robot_bringup >/dev/null
  if [[ "${DAMGC_MAPPING_MODE}" == "2D" ]]; then
    ros2 pkg prefix slam_toolbox >/dev/null
  else
    ros2 pkg prefix isaac_ros_visual_slam >/dev/null
    ros2 pkg prefix nvblox_ros >/dev/null
  fi
'; then
  printf 'The container is missing a required ROS package or install_docker overlay.\n' >&2
  exit 1
fi
if [[ "${VSLAM_ONLY}" != "1" ]] && ! docker exec -u "${CONTAINER_USER}" "${CONTAINER_NAME}" setsid --wait bash -lc '
  source /opt/ros/humble/setup.bash
  ros2 pkg prefix nav2_planner >/dev/null
  ros2 pkg prefix nav2_controller >/dev/null
  ros2 pkg prefix nav2_bt_navigator >/dev/null
  ros2 pkg prefix nav2_lifecycle_manager >/dev/null
'; then
  printf 'The container is missing a required Nav2 package.\n' >&2
  exit 1
fi

container_log_dir="/workspaces/isaac_ros-dev/log/${RUN_PREFIX}_${RUN_ID}"
docker exec -d -u "${CONTAINER_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  -e DAMGC_SOLO_DELIVERY="${MAPPING_SOLO_DELIVERY:-0}" \
  -e DAMGC_MAPPING_MANUAL_ONLY="${MAPPING_MANUAL_ONLY}" \
  -e DAMGC_COOPERATIVE_ON_STARTUP="${MAPPING_COOPERATIVE_ON_STARTUP}" \
  -e DAMGC_MAPPING_RUN_ID="${RUN_ID}" \
  -e DAMGC_MAPPING_SNAPSHOT="${CONTAINER_SNAPSHOT}" \
  -e DAMGC_MAPPING_MODE="${MAPPING_MODE}" \
  -e DAMGC_VSLAM_HEADLESS="${VSLAM_HEADLESS}" \
  -e DAMGC_VSLAM_ONLY="${VSLAM_ONLY}" \
  -e DAMGC_SELF_FILTER_ENABLED="${SELF_FILTER_ENABLED}" \
  -e DAMGC_MAPPING_INITIAL_SCAN="${MAPPING_INITIAL_SCAN}" \
  "${CONTAINER_NAME}" setsid --wait bash -lc '
    log_path="$1"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:${LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:${LD_LIBRARY_PATH}"
    printf "%s" "${DAMGC_MAPPING_RUN_ID}" > /workspaces/isaac_ros-dev/data/.mapping_session_id
    echo "$$" > /tmp/damgc_vslam_mapping_vslam.pid
    if [[ "${DAMGC_MAPPING_MODE}" == "2D" ]]; then
      exec ros2 launch rescue_robot_bringup mapping_2d.launch.py >>"${log_path}" 2>&1
    fi
    if [[ "${DAMGC_VSLAM_ONLY}" == "1" ]]; then
      exec ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py \
        publish_odom_to_base_tf:=true >>"${log_path}" 2>&1
    fi
    exec ros2 launch rescue_robot_bringup nvblox_vslam_realsense.launch.py \
      filter_enabled:="${DAMGC_SELF_FILTER_ENABLED}" \
      initial_scan:="${DAMGC_MAPPING_INITIAL_SCAN}" >>"${log_path}" 2>&1
  ' _ "${container_log_dir}/vslam_nvblox.log"

if [[ "${VSLAM_HEADLESS}" == "1" ]]; then
  if [[ "${VSLAM_ONLY}" == "1" ]]; then
    printf '[5/7] VSLAM-only headless mode: nvblox and RViz are disabled...\n'
  else
    printf '[5/7] Headless mode: skipping RViz...\n'
  fi
else
  printf '[5/7] Starting RViz...\n'
  rviz_config="${MAPPING_RVIZ_CONFIG:-nvblox_2d_view.rviz}"
  if [[ "${MAPPING_MODE}" == "2D" ]]; then rviz_config="mapping_2d.rviz"; fi
  docker exec -d -u "${CONTAINER_USER}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    "${CONTAINER_NAME}" setsid --wait bash -lc '
      log_path="$1"
      source /opt/ros/humble/setup.bash
      source /workspaces/isaac_ros-dev/install_docker/setup.bash
      echo "$$" > /tmp/damgc_vslam_mapping_rviz.pid
      exec rviz2 -d "$2" --ros-args \
        -r /lifecycle_manager_navigation/is_active:=/lifecycle_manager_nvblox_nav2/is_active \
        -r /lifecycle_manager_navigation/manage_nodes:=/lifecycle_manager_nvblox_nav2/manage_nodes \
        >>"${log_path}" 2>&1
    ' _ "${container_log_dir}/rviz.log" "/workspaces/isaac_ros-dev/rviz/${rviz_config}"
fi

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
wait_for_topic "/leader/odometry/local" 30
recording_ready_topic="/visual_slam/tracking/odometry"
if [[ "${MAPPING_MODE}" == "2D" ]]; then
  recording_ready_topic="/scan"
  wait_for_topic "/leader/camera/depth/image_rect_raw" 45
  wait_for_topic "/scan" 60
  wait_for_topic "/map" 60
else
  wait_for_topic "/leader/camera/infra1/image_rect_raw" 45
  wait_for_topic "/visual_slam/tracking/odometry" 120
  wait_for_topic "/nvblox_node/static_map_slice" 60
fi

if [[ -n "${CONTAINER_SNAPSHOT}" ]]; then
  printf 'Restoring saved nvblox map and anchored starting pose...\n'
  docker exec -u "${CONTAINER_USER}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    "${CONTAINER_NAME}" setsid --wait bash -lc '
      source /opt/ros/humble/setup.bash
      source /workspaces/isaac_ros-dev/install_docker/setup.bash
      /usr/bin/python3 /workspaces/isaac_ros-dev/scripts/mapping_snapshot.py load "$1"
    ' _ "${CONTAINER_SNAPSHOT}" >"${LOG_DIR}/map_restore.json" 2>"${LOG_DIR}/map_restore_error.log"
fi

if [[ "${VSLAM_ONLY}" != "1" ]]; then
  printf 'Checking Nav2 lifecycle readiness...\n'
  for nav2_node in planner_server controller_server behavior_server bt_navigator; do
    # The lifecycle manager owns activation. During GPU-heavy startup, a
    # lifecycle service can take longer than the old one-shot 15 s query; do
    # not race the manager by manually activating nodes from this script.
    nav2_deadline=$((SECONDS + 120))
    nav2_state=""
    until [[ "${nav2_state}" == "active [3]" ]]; do
      nav2_state="$(timeout 10 ros2 lifecycle get "/${nav2_node}" 2>/dev/null || true)"
      if [[ "${nav2_state}" == "active [3]" ]]; then
        break
      fi
      # A failed lifecycle manager has stopped sequencing transitions. Recover
      # only that condition; never race a manager which is still bringing up.
      if grep -Fq 'Failed to bring up all requested nodes. Aborting bringup.' "${LOG_DIR}/vslam_nvblox.log"; then
        if [[ "${nav2_state}" == "unconfigured [1]" ]]; then
          timeout 15 ros2 lifecycle set "/${nav2_node}" configure >>"${LOG_DIR}/nav2_recovery.log" 2>&1 || true
        elif [[ "${nav2_state}" == "inactive [2]" ]]; then
          timeout 15 ros2 lifecycle set "/${nav2_node}" activate >>"${LOG_DIR}/nav2_recovery.log" 2>&1 || true
        fi
      fi
      if (( SECONDS >= nav2_deadline )); then
        break
      fi
      sleep 2
    done
    if [[ "${nav2_state}" != "active [3]" ]]; then
      printf 'Nav2 node %s is not active (%s); see %s/vslam_nvblox.log\n' \
        "${nav2_node}" "${nav2_state}" "${LOG_DIR}" >&2
      exit 1
    fi
    printf '  active: %s\n' "${nav2_node}"
  done
fi

if [[ "${MAPPING_RECORD_BAG}" == "1" ]]; then
printf '[6/7] Starting metrics rosbag...\n'
docker exec "${CONTAINER_NAME}" rm -f "${CONTAINER_BAG_PID_FILE}" >/dev/null 2>&1 || true
docker exec -d -u "${CONTAINER_USER}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
  "${CONTAINER_NAME}" setsid --wait bash -lc '
    bag_path="$1"
    log_path="$2"
    source /opt/ros/humble/setup.bash
    source /workspaces/isaac_ros-dev/install_docker/setup.bash
    echo "$$" > /tmp/damgc_vslam_mapping_bag.pid
    exec ros2 bag record --output "${bag_path}" \
      --include-hidden-topics \
      /mapping/projected_map \
      /mapping/planned_goal \
      /nav2/cmd_vel \
      /spin/_action/status \
      /navigate_to_pose/_action/status \
      /leader/command_selector/request \
      /leader/cmd_vel \
      /leader/mapping/control \
      /leader/mapping/mode \
      /leader/mapping/status \
      /leader/command_selector/status \
      /leader/system_state \
      /leader/stm32_rx/sequence_drops \
      /leader/stm32_rx/crc_errors \
      /leader/odom/raw \
      /leader/imu/data_calibrated \
      /leader/odometry/local \
      /leader/odometry/global \
      /nvblox_node/static_map_slice \
      /visual_slam/tracking/odometry \
      /visual_slam/status \
      /tf \
      /tf_static >>"${log_path}" 2>&1
  ' _ "${CONTAINER_BAG_DIR}" "${container_log_dir}/rosbag.log"
BAG_STARTED=1

bag_deadline=$((SECONDS + 60))
until container_process_running "${CONTAINER_BAG_PID_FILE}" && \
  grep -Fq 'Recording...' "${LOG_DIR}/rosbag.log" 2>/dev/null && \
  grep -Fq "Subscribed to topic '${recording_ready_topic}'" \
    "${LOG_DIR}/rosbag.log" 2>/dev/null; do
  if (( SECONDS >= bag_deadline )); then
    printf 'Rosbag failed to start. Check %s/rosbag.log\n' "${LOG_DIR}" >&2
    exit 1
  fi
  sleep 1
done
printf '  recording: %s\n' "${BAG_DIR}"
# /leader/cmd_vel is intentionally discovered after the teleop node starts.
# The sensor topics are already present, so the baseline delay also gives the
# recorder time to finish subscribing to their VSLAM/odometry streams.
printf '  capturing a 5-second stationary baseline...\n'
sleep 5
else
  printf '[6/7] Lightweight mapping: rosbag recording disabled.\n'
fi

if [[ "${MAPPING_AUTOMATIC_EXPLORATION}" == "1" ]]; then
  if [[ "${MAPPING_CHECKPOINT_DETECTION:-0}" == "1" ]]; then
    export CHECKPOINT_CONTAINER_NAME="damgc-survivor-detector-checkpoint-$$"
    export DAMGC_REPO_ROOT="${REPO_ROOT}"
    setsid bash -c '
      set +u
      source /opt/ros/humble/setup.bash
      source "$DAMGC_REPO_ROOT/install/setup.bash"
      exec ros2 launch "$DAMGC_REPO_ROOT/src/leader/rescue_robot_bringup/launch/checkpoint_survivor.launch.py"
    ' >"${LOG_DIR}/checkpoint_survivor.log" 2>&1 &
    HOST_PIDS+=("$!")
  fi
  if [[ "${MAPPING_SOLO_DELIVERY:-0}" == "1" ]]; then
    setsid bash -c '
      source /opt/ros/humble/setup.bash
      source "$DAMGC_REPO_ROOT/install/setup.bash"
      exec python3 "$DAMGC_REPO_ROOT/scripts/solo_grasp_host.py"
    ' >"${LOG_DIR}/solo_grasp.log" 2>&1 &
    HOST_PIDS+=("$!")
  fi
  printf '[7/7] Automatic mapping mission: verify live inputs, scan, explore and return. Ctrl-C stops.\n'
  docker exec -u "${CONTAINER_USER}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    -e MAPPING_POST_RETURN="${MAPPING_POST_RETURN:-0}" \
    -e MAPPING_SOLO_DELIVERY="${MAPPING_SOLO_DELIVERY:-0}" \
    -e MAPPING_CHECKPOINT_DETECTION="${MAPPING_CHECKPOINT_DETECTION:-0}" \
    -e MAPPING_START_POSE_FILE="${MAPPING_START_POSE_FILE:-}" \
    "${CONTAINER_NAME}" setsid --wait bash -lc '
      source /opt/ros/humble/setup.bash
      source /workspaces/isaac_ros-dev/install_docker/setup.bash
      echo "$$" > /tmp/damgc_vslam_mapping_auto.pid
      exec python3 /workspaces/isaac_ros-dev/scripts/mapping_autorun.py --mapping-log "$1"
    ' _ "${container_log_dir}/vslam_nvblox.log" > >(tee "${LOG_DIR}/automatic_exploration.log") 2>&1
  exit 0
fi

printf '[7/7] Starting arrow-key control. E/D changes speed; Space stops; Ctrl-C shuts everything down.\n'
source_with_nounset_disabled "${REPO_ROOT}/install/setup.bash"
if [[ "${MAPPING_INITIAL_SCAN}" == "1" ]]; then
  printf 'Requesting the one-time 360-degree mapping scan...\n'
  scan_service_deadline=$((SECONDS + 20))
  until ros2 service list 2>/dev/null | grep -Fxq "/leader/initial_map_scan/start"; do
    if (( SECONDS >= scan_service_deadline )); then
      printf 'Initial scan service did not appear; see %s/vslam_nvblox.log\n' \
        "${LOG_DIR}" >&2
      break
    fi
    sleep 1
  done
  if (( SECONDS < scan_service_deadline )); then
    ros2 service call /leader/initial_map_scan/start std_srvs/srv/Trigger "{}" \
      >"${LOG_DIR}/initial_map_scan_trigger.log" 2>&1 || {
        printf 'Initial scan request failed; see %s/initial_map_scan_trigger.log\n' \
          "${LOG_DIR}" >&2
      }
  fi
fi
ros2 run rescue_robot_bringup arrow_key_teleop.py --ros-args \
  -p command_topic:=/leader/teleop/cmd_vel \
  -p select_command_source:=true \
  -p linear_speed:=0.08 \
  -p angular_speed:=0.25
