#!/usr/bin/env bash
# Explicit opt-in simulator startup, after the recorder. No navigation/teleop.
set -eo pipefail
if (( $# > 2 )); then
  echo 'Usage: bash docker/run_selected_gui.sh [preflight|start] [loop16|turn15|turn45|straight_slow|combined10]' >&2
  echo 'Run docker logs as a separate command, on a new line or after &&.' >&2
  exit 2
fi
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mode="${1:-preflight}"
route="${2:-loop16}"
if [[ "$mode" != preflight && "$mode" != start ]]; then
  echo 'Usage: bash docker/run_selected_gui.sh [preflight|start] [loop16|turn15|turn45|straight_slow|combined10]' >&2
  exit 2
fi
if [[ "$route" != loop16 && "$route" != turn15 && "$route" != turn45 && "$route" != straight_slow && "$route" != combined10 ]]; then
  echo 'Only declared loop16, turn15, turn45, straight_slow or combined10 inspection protocols are supported.' >&2
  exit 2
fi
export SELECTED_EXECUTION_PROFILE="${SELECTED_EXECUTION_PROFILE:-default}"
if [[ "$SELECTED_EXECUTION_PROFILE" != default &&
      ! ( "$SELECTED_EXECUTION_PROFILE" == turn45_bounded_v1 && "$route" == turn45 ) ]]; then
  echo 'Undeclared GUI capture execution profile for this route.' >&2
  exit 2
fi
export DISPLAY="${DISPLAY:-:1}"
export INSPECT_ROUTE="$route"
export INSPECT_X11_HOSTNAME="$(hostname)"
: "${XAUTHORITY:?Set XAUTHORITY to the current X11 session cookie file}"
test -r "$XAUTHORITY"
archive_path=/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab
dev_container=astrobot-selected-dev
sim_container=astrobot-sim-lunaryard
if [[ "$(docker inspect "$sim_container" --format '{{.State.Running}}' 2>/dev/null || true)" == true ]]; then
  echo 'Simulator already running; refusing to replace an active world.' >&2
  exit 1
fi
xdpyinfo -display "$DISPLAY" >/dev/null
nvidia-smi
python3 docker/check_cuda_runtime.py
PYTHONPATH=tools python3 -c 'import sys; from archive_guard import check_capacity, ARCHIVE_ROOT, GIB; from costmap_scenarios import archive_limit; print(check_capacity(ARCHIVE_ROOT, archive_limit(sys.argv[1],sys.argv[2])+50*GIB))' "$route" "$SELECTED_EXECUTION_PROFILE"
host_device="$(stat -c %d "$archive_path")"
dev_device="$(docker exec "$dev_container" stat -c %d /archive)"
if [[ "$host_device" != "$dev_device" ]]; then
  echo 'Stale/mismatched development archive bind; refusing capture.' >&2
  exit 1
fi
docker exec "$dev_container" python3 /workspace/astrobot-lab/docker/check_cuda_runtime.py
docker exec "$dev_container" test -f /archive/revision-1545bcc16e884638ae30b137cacb17ea/config/config.json
compose=(docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f docker-compose.inspect.yml)
"${compose[@]}" config --quiet
# A disposable Xlib probe, not a simulator launch. No cookie copied to evidence.
"${compose[@]}" run --rm --no-deps --entrypoint timeout astrobot-sim \
  15 /isaac-sim/python.sh /workspace/astrobot-lab/docker/check_x11.py
echo "GUI preflight passed for $route; this is not simulator/GLIM/terrain acceptance."
if [[ "$mode" == start ]]; then
  # Recorder additionally enforces actual command ownership, request hashes,
  # geometry, native stamps, IMU and lease readiness before permitting movement.
  if ! nodes="$(docker exec "$dev_container" bash -lc 'source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash && ros2 node list')"; then
    echo 'Could not query ROS nodes; refusing simulator startup. Check the dev container and ROS setup.' >&2
    exit 1
  fi
  # Bash builtins only: host ripgrep/grep availability must not affect safety.
  recorder_count=0
  while IFS= read -r node_name; do
    if [[ "$node_name" == /terrain_shadow_capture ]]; then
      recorder_count=$((recorder_count + 1))
    fi
  done <<< "$nodes"
  if (( recorder_count != 1 )); then
    echo "Start exactly one matching guarded recorder first; found $recorder_count (see inspection guide)." >&2
    exit 1
  fi
  "${compose[@]}" up -d --no-deps astrobot-sim
fi
