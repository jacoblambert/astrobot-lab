#!/usr/bin/env bash
# Run python with the pinned host tools environment (no ROS on the path).
#   tools/uv_python.sh -m pytest -q -p no:cacheprovider tools/test
#   tools/uv_python.sh tools/replay_live_local_terrain.py /archive/<mapper>
# TERRAIN_LIBRARY defaults to the workspace build (needed by the terrain tests).
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
export TERRAIN_LIBRARY=${TERRAIN_LIBRARY:-$repo/astrobot-lab/build/slam_mapping/liblayered_terrain.so}
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH \
  PYTHONPATH="$repo/astrobot-lab/src/astrobot_core:$repo/tools" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
  uv run --offline --no-project --with-requirements "$repo/tools/requirements-test.txt" python "$@"
