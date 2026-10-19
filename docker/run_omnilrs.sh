#!/usr/bin/env bash
set -euo pipefail

ENVIRONMENT_NAME="${OMNILRS_ENVIRONMENT:-lunaryard_40m}"
HEADLESS_RAW="${OMNILRS_HEADLESS:-true}"
ROCKS_RAW="${OMNILRS_ROCKS_ENABLED:-}"
ENVIRONMENT_SEED="${OMNILRS_ENVIRONMENT_SEED:-}"

case "${HEADLESS_RAW,,}" in
  true|1|yes|on)
    HEADLESS_OVERRIDE="True"
    ;;
  false|0|no|off)
    HEADLESS_OVERRIDE="False"
    ;;
  *)
    echo "Invalid OMNILRS_HEADLESS='${HEADLESS_RAW}'. Use true or false." >&2
    exit 2
    ;;
esac

ROCKS_OVERRIDE=""
if [[ -n "${ROCKS_RAW}" ]]; then
  case "${ROCKS_RAW,,}" in
    true|1|yes|on)
      ROCKS_OVERRIDE="True"
      ;;
    false|0|no|off)
      ROCKS_OVERRIDE="False"
      ;;
    *)
      echo "Invalid OMNILRS_ROCKS_ENABLED='${ROCKS_RAW}'. Use true or false." >&2
      exit 2
      ;;
  esac
fi

# Camera toggle: the camera (RGB+depth, two RTX-raytraced streams) is unused by SLAM/nav
# and a major GPU load. When disabled (default), stop the camera/depth ROS streams via a
# very high frameSkipCount. NOTE: we keep the _mono_depth_imu USD because it is the ONLY
# robot asset that carries the IMU (the camera-free vlp16 USD has no IMU, which GLIM needs).
CAMERA_RAW="${OMNILRS_CAMERA_ENABLED:-false}"
CAMERA_DISABLE=""
case "${CAMERA_RAW,,}" in
  true|1|yes|on)
    : ;;  # keep camera streams for GUI/UI-console runs
  *)
    CAMERA_DISABLE="1" ;;
esac

if [[ "${HEADLESS_OVERRIDE}" == "False" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    cat >&2 <<'EOF'
GUI mode requested but DISPLAY is empty.

Set DISPLAY from your local X11 session, for example:
  export DISPLAY=:0
  xhost +local:root

Or run headless with:
  export OMNILRS_HEADLESS=true
EOF
    exit 2
  fi

  if [[ ! -d /tmp/.X11-unix ]]; then
    cat >&2 <<'EOF'
GUI mode requested but /tmp/.X11-unix is not available in the container.

Make sure the compose service mounts the host X11 socket and that an X server is running.
EOF
    exit 2
  fi
  # DISPLAY on the host can work while container authorization fails. Isaac
  # otherwise continues without a window, falsely resembling a GUI success.
  timeout 15 /isaac-sim/python.sh /workspace/astrobot-lab/docker/check_x11.py
fi

ARGS=(
  environment="${ENVIRONMENT_NAME}"
  rendering.renderer.headless="${HEADLESS_OVERRIDE}"
  "++rendering.renderer.disable_viewport_updates=${OMNILRS_DISABLE_VIEWPORT_UPDATES:-false}"
  "++rendering.renderer.path_stat_cache_enabled=${OMNILRS_IMPORT_STAT_CACHE:-true}"
)

# The explicit-ray adapter consumes every physics acquisition tick. This is an
# opt-in experiment, not a change to historical 30 Hz rendering protocols.
if [[ "${OMNILRS_SELECTED_LIDAR:-false}" == "true" ]]; then
  if [[ "${OMNILRS_NATURAL_HOLDOUT:-false}" != "true" || -z "${OMNILRS_NATURAL_MOTION:-}" ]]; then
    echo "Selected LiDAR requires guarded natural motion." >&2
    exit 2
  fi
  ARGS+=("environment.rendering_dt=0.016666666666666666")
fi

if [[ -n "${ROCKS_OVERRIDE}" ]]; then
  ARGS+=("++environment.rocks_settings.enable=${ROCKS_OVERRIDE}")
fi

if [[ -n "${CAMERA_DISABLE}" ]]; then
  # astrobot_0 is robots_settings.parameters[0]; disable camera + depth ROS streams.
  ARGS+=("++environment.robots_settings.parameters.0.sensor_settings.camera_frame_skip_count=1000000")
  ARGS+=("++environment.robots_settings.parameters.0.sensor_settings.depth_frame_skip_count=1000000")
fi

if [[ -n "${ENVIRONMENT_SEED}" ]]; then
  if ! [[ "${ENVIRONMENT_SEED}" =~ ^[0-9]+$ ]] || [[ "${ENVIRONMENT_SEED}" -le 0 ]]; then
    echo "Invalid OMNILRS_ENVIRONMENT_SEED='${ENVIRONMENT_SEED}'. Use a positive integer." >&2
    exit 2
  fi
  ARGS+=("++environment.seed=${ENVIRONMENT_SEED}")
fi

# NVML/nvidia-smi can succeed while CUDA initialization fails (e.g. error 999).
# Fail before world construction, calibration or any possible motion. This probe
# creates only a disposable context; it never changes drivers or resets hardware.
/isaac-sim/python.sh /workspace/astrobot-lab/docker/check_cuda_runtime.py

echo "Starting OmniLRS environment='${ENVIRONMENT_NAME}' headless='${HEADLESS_OVERRIDE}' rocks='${ROCKS_OVERRIDE:-default}' seed='${ENVIRONMENT_SEED:-config}' camera='${CAMERA_RAW}' display='${DISPLAY:-}'"

exec /isaac-sim/python.sh /workspace/omnilrs/run.py "${ARGS[@]}"
