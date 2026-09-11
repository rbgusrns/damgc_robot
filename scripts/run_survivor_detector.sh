#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${SURVIVOR_IMAGE:-damgc-survivor-yolo:humble}"
MODEL_CACHE="${SURVIVOR_MODEL_CACHE:-${HOME}/.cache/damgc-survivor-ultralytics}"
CONTAINER_NAME="${SURVIVOR_CONTAINER_NAME:-damgc-survivor-detector-$$}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
RMW_VALUE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
FASTDDS_VALUE="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

mkdir -p "${MODEL_CACHE}"

exec docker run --rm -it \
  --name "${CONTAINER_NAME}" \
  --runtime=nvidia \
  --network host \
  --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_VALUE}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_VALUE}" \
  -e YOLO_CONFIG_DIR=/tmp \
  -v "${MODEL_CACHE}:/root/.cache/ultralytics" \
  -v "${REPO_ROOT}:/workspaces/isaac_ros-dev:ro" \
  --workdir /root/.cache/ultralytics \
  "${IMAGE}" \
  "set +u; source /opt/ros/humble/setup.bash; source /opt/damgc_survivor_ws/install_survivor/setup.bash; set -u; ros2 launch rescue_robot_survivor person_detector.launch.py ${*}"
