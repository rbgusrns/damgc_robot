#!/usr/bin/env bash
set -euo pipefail

expected_domain="${ROS_DOMAIN_ID:-42}"

if [[ "${ROS_LOCALHOST_ONLY:-}" != "0" ]]; then
  echo "ERROR: ROS_LOCALHOST_ONLY must be 0" >&2
  exit 1
fi
if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_fastrtps_cpp" ]]; then
  echo "ERROR: RMW_IMPLEMENTATION must be rmw_fastrtps_cpp" >&2
  exit 1
fi

echo "DDS environment OK (domain ${expected_domain})."
echo "Follower: ros2 multicast receive"
echo "Leader:   ros2 multicast send"
echo "Then test /test/orin_link as documented in docs/progress/DDS.md."
