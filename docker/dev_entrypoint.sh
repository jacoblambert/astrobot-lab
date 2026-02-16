#!/usr/bin/env bash
set -e
sudo ip link set lo multicast on || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "$ROS_LOG_DIR" || true
source /opt/ros/spaceros/setup.bash
exec "$@"
