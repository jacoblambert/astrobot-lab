# GLIM config (astrobot / lunaryard)

## Backend: CPU is the standard (since 2026-06-03)

GLIM odometry + sub/global mapping run on the **CPU**, not the GPU. This is the
default and recommended configuration.

- `config.json` -> `config_odometry: config_odometry_cpu.json`
  (`libodometry_estimation_cpu.so`, GICP/iVox, `num_threads: 6`).
- `config_sub_mapping.json` / `config_global_mapping.json` use
  `registration_error_factor_type: VGICP` (CPU), not `VGICP_GPU`.
- `config_odometry_gpu.json` is kept as the **GPU alternative** (unused). To go
  back to GPU you must ALSO flip the two mapping configs' factor type back to
  `VGICP_GPU`.

### Why CPU

On the 8 GB laptop GPU, Isaac Sim's RTX renderer (~6 GB, the dominant consumer)
and GLIM's GPU GICP competed for GPU **compute**, starving registration. Under
the phase-4 rocks-on load this caused two failure modes ~2/3 of the time:
a silent GICP **freeze** (odom stuck at origin while the rover drove) and a
catastrophic **runaway divergence** (odom blowing up to ~1700 m).

Head-to-head on the rock-barrier scenario (seed 42), same exploration params:

| Backend | Clean true successes | Failure modes | Output rate |
|---------|----------------------|---------------|-------------|
| GPU     | 1 / 3                | 1 freeze-runaway, 1 drift false-success | ~9 Hz |
| **CPU** | **3 / 3**            | none (freeze/diverge/jump = 0)          | 9.1–9.2 Hz (real-time) |

The rover is slow, so CPU GICP keeps real-time easily (output rate matches the
~9.2 Hz LiDAR input). Moving GLIM off the GPU also frees the 8 GB ceiling for
larger / multi-robot scenes (Isaac-alone-on-GPU scales further).

### If we ever want GPU GLIM back

The GPU backend is faster per scan and would matter for very high point rates or
many robots. But returning to it on a **shared** GPU requires solving the
Isaac-vs-GLIM compute contention first — i.e. some form of **GPU serialization**:
NVIDIA MPS, CUDA stream priorities / time-slicing, or a dedicated GPU for GLIM.
Without that, the freeze/divergence returns. Don't switch back blindly.
