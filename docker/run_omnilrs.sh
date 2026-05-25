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
fi

ARGS=(
  environment="${ENVIRONMENT_NAME}"
  rendering.renderer.headless="${HEADLESS_OVERRIDE}"
)

if [[ -n "${ROCKS_OVERRIDE}" ]]; then
  ARGS+=("++environment.rocks_settings.enable=${ROCKS_OVERRIDE}")
fi

if [[ -n "${ENVIRONMENT_SEED}" ]]; then
  if ! [[ "${ENVIRONMENT_SEED}" =~ ^[0-9]+$ ]] || [[ "${ENVIRONMENT_SEED}" -le 0 ]]; then
    echo "Invalid OMNILRS_ENVIRONMENT_SEED='${ENVIRONMENT_SEED}'. Use a positive integer." >&2
    exit 2
  fi
  ARGS+=("++environment.seed=${ENVIRONMENT_SEED}")
fi

echo "Starting OmniLRS environment='${ENVIRONMENT_NAME}' headless='${HEADLESS_OVERRIDE}' rocks='${ROCKS_OVERRIDE:-default}' seed='${ENVIRONMENT_SEED:-config}' display='${DISPLAY:-}'"

exec /isaac-sim/python.sh /workspace/omnilrs/run.py "${ARGS[@]}"
