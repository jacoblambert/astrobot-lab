#!/usr/bin/env bash
set -euo pipefail

ENVIRONMENT_NAME="${OMNILRS_ENVIRONMENT:-lunaryard_40m}"
HEADLESS_RAW="${OMNILRS_HEADLESS:-true}"
ROCKS_RAW="${OMNILRS_ROCKS_ENABLED:-}"

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

echo "Starting OmniLRS environment='${ENVIRONMENT_NAME}' headless='${HEADLESS_OVERRIDE}' rocks='${ROCKS_OVERRIDE:-default}' display='${DISPLAY:-}'"

exec /isaac-sim/python.sh /workspace/omnilrs/run.py "${ARGS[@]}"
