#!/usr/bin/env bash
# Recorder-first opt-in acquisition. Does not start navigation or replace a world.
set -eo pipefail
if (( $# > 2 )); then
  echo 'Usage: bash docker/run_selected_headless.sh [preflight|start] [loop16|turn15|turn360|peek_reverse|peek_reverse20|straight_slow|combined10|terrain_step]' >&2
  exit 2
fi
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mode="${1:-preflight}"
route="${2:-loop16}"
if [[ "$mode" != preflight && "$mode" != start ]]; then
  echo 'Usage: bash docker/run_selected_headless.sh [preflight|start] [loop16|turn15|turn360|peek_reverse|peek_reverse20|straight_slow|combined10|terrain_step]' >&2
  exit 2
fi
if [[ "$route" != loop16 && "$route" != turn15 && "$route" != turn45 && "$route" != turn360 && "$route" != peek_reverse && "$route" != peek_reverse20 && "$route" != straight_slow && "$route" != combined10 && "$route" != terrain_step && "$route" != costmap1 && "$route" != costmap2 && "$route" != costmap_return4 && "$route" != costmap_square8 && "$route" != costmap_detour8 ]]; then
  echo 'Only declared inspection routes are supported.' >&2
  exit 2
fi
export SELECTED_ROUTE="$route"
export SELECTED_EXECUTION_PROFILE="${SELECTED_EXECUTION_PROFILE:-default}"
if [[ "$SELECTED_EXECUTION_PROFILE" != default &&
      ! ( "$SELECTED_EXECUTION_PROFILE" == loop16_slow_host_v1 && "$route" == loop16 ) &&
      ! ( "$SELECTED_EXECUTION_PROFILE" == turn15_gc_stress_v1 && "$route" == turn15 ) &&
      ! ( "$SELECTED_EXECUTION_PROFILE" == turn45_bounded_v1 && "$route" == turn45 ) ]]; then
  echo 'Undeclared capture execution profile for this route.' >&2
  exit 2
fi
archive_path=/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab
dev_container=astrobot-selected-dev
if [[ "$(docker inspect astrobot-sim-lunaryard --format '{{.State.Running}}' 2>/dev/null || true)" == true ]]; then
  echo 'Simulator already running; refusing to replace an active world.' >&2
  exit 1
fi
nvidia-smi
python3 docker/check_cuda_runtime.py
PYTHONPATH=tools python3 -c 'import sys; from archive_guard import check_capacity, ARCHIVE_ROOT, GIB; from costmap_scenarios import archive_limit; print(check_capacity(ARCHIVE_ROOT, archive_limit(sys.argv[1],sys.argv[2])+50*GIB))' "$route" "$SELECTED_EXECUTION_PROFILE"
if [[ "$(stat -c %d "$archive_path")" != "$(docker exec "$dev_container" stat -c %d /archive)" ]]; then
  echo 'Stale/mismatched development archive bind; refusing capture.' >&2
  exit 1
fi
docker exec "$dev_container" python3 /workspace/astrobot-lab/docker/check_cuda_runtime.py
compose=(docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f docker-compose.selected-headless.yml)
"${compose[@]}" config --quiet
echo "Headless preflight passed for $route; not simulator/SLAM/terrain acceptance."
if [[ "$mode" == start ]]; then
  if ! nodes="$(docker exec "$dev_container" bash -lc 'source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash && ros2 node list')"; then
    echo 'Could not query ROS nodes; refusing simulator startup.' >&2
    exit 1
  fi
  recorder_count=0
  while IFS= read -r node_name; do
    if [[ "$node_name" == /terrain_shadow_capture ]]; then
      recorder_count=$((recorder_count + 1))
    fi
  done <<< "$nodes"
  if (( recorder_count != 1 )); then
    echo "Start exactly one matching guarded recorder first; found $recorder_count." >&2
    exit 1
  fi
  "${compose[@]}" up -d --no-deps astrobot-sim
fi
