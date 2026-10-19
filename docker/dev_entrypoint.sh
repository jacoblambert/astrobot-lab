#!/usr/bin/env bash
set -e
sudo ip link set lo multicast on || true
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "$ROS_LOG_DIR" || true

# Preserve intended middleware/domain before setup scripts.
ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-177}"
ORIG_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ORIG_CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///config/cyclonedds_config.xml}"

if [[ -f /etc/ros_setup.sh ]]; then
  # shellcheck disable=SC1091
  source /etc/ros_setup.sh
elif [[ -f /opt/ros/spaceros/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/spaceros/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Re-assert desired values after setup scripts.
export ROS_DOMAIN_ID="${ORIG_ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${ORIG_RMW_IMPLEMENTATION}"
export CYCLONEDDS_URI="${ORIG_CYCLONEDDS_URI}"

if [[ "${ASTROBOT_AUTO_BUILD:-false}" == "true" && -d /workspace/astrobot-lab/astrobot-lab/src ]]; then
  echo "[astrobot-dev] Building ROS overlay in /workspace/astrobot-lab/astrobot-lab"
  cd /workspace/astrobot-lab/astrobot-lab
  build_args=(--symlink-install)
  if [[ "${ASTROBOT_BUILD_SOURCE_GLIM:-false}" != "true" ]]; then
    # GLIM and glim_ros are built into the dev image from source. The checked-out
    # submodules are for fork development and should not rebuild on every
    # container start unless explicitly requested.
    build_args+=(--packages-ignore glim glim_ros)
  else
    build_args+=(--allow-overriding glim glim_ros)
    build_args+=(
      --cmake-args
      -DCMAKE_BUILD_TYPE=Release
      -DBUILD_WITH_CUDA=ON
      -DBUILD_WITH_VIEWER=OFF
      -DBUILD_WITH_OPENCV=ON
      -DBUILD_WITH_MARCH_NATIVE=OFF
      "-DCMAKE_PREFIX_PATH=/opt/ros/jazzy;/usr/local"
    )
  fi
  colcon build "${build_args[@]}"
  # shellcheck disable=SC1091
  source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
fi

# Prevent stale daemon middleware mismatch between sessions.
ros2 daemon stop >/dev/null 2>&1 || true
pkill -f _ros2_daemon >/dev/null 2>&1 || true
rm -rf "${ROS_HOME:-$HOME/.ros}/ros2cli" >/dev/null 2>&1 || true

exec "$@"
