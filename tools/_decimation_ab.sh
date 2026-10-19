#!/bin/bash
# Compatibility entry point; historical implementation is preserved verbatim.
# WARNING: legacy sweeps can overwrite bags, use workspace storage and stop ROS processes.
echo "WARNING: historical sweep; not the archive-safe v2 workflow. Read tools/legacy/README.md." >&2
exec bash "$(dirname "${BASH_SOURCE[0]}")/legacy/terrain_sweeps/_decimation_ab.sh" "$@"
