#!/usr/bin/env bash
set -e
sudo ip link set lo multicast on || true
source /opt/ros/humble/setup.bash
exec "$@"