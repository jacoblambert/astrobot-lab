# Historical experiments — not the v2 execution path

`terrain_sweeps/` preserves six superseded voxel/BEV parameter sweeps verbatim.
Their original SHA256 values are in `terrain_sweeps/original_sha256.json`.
Old CLI paths forward arguments and exit status, but are not recommended runners.

| Scripts | Historical purpose | Current replacement |
| --- | --- | --- |
| `_planefit_ab.sh`, `_rgpf_ab.sh` | Ground-fit parameter experiments | Declared local reconstruction / surface diagnostics |
| `_decimation_ab.sh`, `_recall_ab.sh` | Sampling and recall tuning | Frozen native replay with unchanged independent reference |
| `_rock_height_sweep.sh`, `bev_sweep.sh` | Height / occupancy parameter sweeps | Explicit candidate comparisons, never unnamed parameter search |

These scripts assume an old sourced ROS container and absolute workspace paths.
Some delete/overwrite output bags, write large workloads to the workspace, kill
ROS processes, and use obsolete CLI arguments. **Do not run them under current
archive/safety rules.** Forwarders preserve historical entry points, not current
qualification or authorization. Tests mock `bash`; they never execute a sweep.

Active native capture, mapper libraries, GT evaluators and hash-verified replay
modules remain in their original locations. In particular, do not move runtime
source files referenced by archived manifests or weaken hash checks to permit it.

Current direction: [v2 plan](../../docs/implementation/implementation-plan-v2.md).
Commands and dependencies: [runbook](../../docs/implementation/agent-runbook.md).
