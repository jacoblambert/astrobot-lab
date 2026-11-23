#!/usr/bin/env bash
# Run python with the pinned host tools environment (no ROS on the path).
#   research/tools/uv_python.sh -m pytest -q -p no:cacheprovider research/tools/test
#   research/tools/uv_python.sh research/tools/replay_live_local_terrain.py /archive/<mapper>
# TERRAIN_LIBRARY defaults to the workspace build (needed by the terrain tests).
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
export TERRAIN_LIBRARY=${TERRAIN_LIBRARY:-$repo/astrobot-lab/build/slam_mapping/liblayered_terrain.so}
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  PYTHONPATH="$repo/research/legacy/astrobot_core:$repo/research/tools" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
  uv run --offline --no-project --with-requirements "$repo/research/tools/requirements-test.txt" python "$@"
