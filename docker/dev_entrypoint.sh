#!/usr/bin/env bash
set -e
sudo ip link set lo multicast on || true
source /opt/spaceros/install/setup.bash
exec "$@"