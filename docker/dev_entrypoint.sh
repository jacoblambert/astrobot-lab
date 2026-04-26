#!/usr/bin/env bash
set -e
sudo ip link set lo multicast on || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "$ROS_LOG_DIR" || true

# Preserve intended middleware/domain before setup scripts.
ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-177}"
ORIG_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ORIG_CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///config/cyclonedds_config.xml}"

if [[ -f /etc/ros_setup.sh ]]; then
  # shellcheck disable=SC1091
  source /etc/ros_setup.sh
elif [[ -f /opt/ros/spaceros/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/spaceros/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Re-assert desired values after setup scripts.
export ROS_DOMAIN_ID="${ORIG_ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${ORIG_RMW_IMPLEMENTATION}"
export CYCLONEDDS_URI="${ORIG_CYCLONEDDS_URI}"

# Prevent stale daemon middleware mismatch between sessions.
ros2 daemon stop >/dev/null 2>&1 || true
pkill -f _ros2_daemon >/dev/null 2>&1 || true
rm -rf "${ROS_HOME:-$HOME/.ros}/ros2cli" >/dev/null 2>&1 || true

exec "$@"
