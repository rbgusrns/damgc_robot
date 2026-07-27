#!/usr/bin/env bash
# Check the AprilTag approach node without starting or stopping any ROS process.

set -o pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-/home/kde/ros2_ws/install/setup.bash}"
NODE_NAME="/follower/apriltag_approach"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 2
fi
if [[ ! -r "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace setup not found: ${WORKSPACE_SETUP}" >&2
  exit 2
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

# ROS setup scripts may probe unset variables; enable nounset only afterwards.
set -u

topics=(
  "/follower/supply/detected"
  "/follower/supply/tag_id"
  "/follower/supply/relative_pose"
  "/follower/supply/distance"
  "/follower/supply/lateral_error"
  "/follower/supply/straight_distance"
  "/follower/supply/angle"
  "/follower/alignment/state"
)

declare -A expected_types=(
  ["/follower/supply/detected"]="std_msgs/msg/Bool"
  ["/follower/supply/tag_id"]="std_msgs/msg/Int32"
  ["/follower/supply/relative_pose"]="geometry_msgs/msg/PoseStamped"
  ["/follower/supply/distance"]="std_msgs/msg/Float64"
  ["/follower/supply/lateral_error"]="std_msgs/msg/Float64"
  ["/follower/supply/straight_distance"]="std_msgs/msg/Float64"
  ["/follower/supply/angle"]="std_msgs/msg/Float64"
  ["/follower/alignment/state"]="std_msgs/msg/String"
)

failures=0
node_verified=0

echo
echo "Topic type checks"
for topic in "${topics[@]}"; do
  actual_type=""
  for _attempt in 1 2 3; do
    actual_type="$(timeout 3s ros2 topic type "${topic}" 2>/dev/null || true)"
    [[ -n "${actual_type}" ]] && break
    sleep 0.5
  done
  if [[ "${actual_type}" == "${expected_types[${topic}]}" ]]; then
    echo "PASS ${topic} [${actual_type}]"
  else
    echo "FAIL ${topic} expected=${expected_types[${topic}]} actual=${actual_type:-missing}"
    failures=$((failures + 1))
  fi
done

echo
echo "Main parameters"
for parameter in target_tag_id source_frame target_distance; do
  if timeout 3s ros2 param get "${NODE_NAME}" "${parameter}"; then
    node_verified=1
  else
    echo "FAIL unable to read parameter ${parameter}"
    failures=$((failures + 1))
  fi
done

if ((node_verified)); then
  echo "PASS node ${NODE_NAME} (parameter service responded)"
else
  echo "FAIL node ${NODE_NAME} is not visible"
  failures=$((failures + 1))
fi

echo
echo "Short message samples (up to 3 seconds each)"
for topic in \
  "/follower/supply/detected" \
  "/follower/supply/tag_id" \
  "/follower/supply/distance" \
  "/follower/supply/angle" \
  "/follower/alignment/state"; do
  echo "--- ${topic}"
  if ! timeout 3s ros2 topic echo "${topic}" --once; then
    echo "NO SAMPLE: valid-tag-only topics do not publish while TAG_LOST"
  fi
done

if ((failures)); then
  echo
  echo "Interface check failed: ${failures} issue(s)."
  exit 1
fi

echo
echo "Interface check passed. Message values still require the manual physical tests."
