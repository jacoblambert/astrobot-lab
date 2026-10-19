#!/bin/bash
# Compatibility entry point for the timestamp-paired, shared-config replay.
# Old positional log-odds sweeps used a different mapper configuration and are
# deliberately rejected. Use replay_surface_fixture.py --params for explicit A/B YAML.
set -e
cd "$(dirname "$0")/.." || exit 2
if [ "$#" -ne 2 ]; then
  echo "Usage: tools/replay_voxel_experiment.sh BAG NEW_OUTPUT_DIRECTORY" >&2
  echo "For explicit configuration experiments use tools/replay_surface_fixture.py BAG --output DIR --params YAML." >&2
  exit 2
fi
if [ -z "${ROS_DISTRO:-}" ]; then source /etc/ros_setup.sh; fi
exec python3 tools/replay_surface_fixture.py "$1" --output "$2"
