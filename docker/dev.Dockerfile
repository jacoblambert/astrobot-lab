ARG SPACEROS_TAG=main-dev
FROM osrf/space-ros:${SPACEROS_TAG}
ARG GLIM_ROS_PACKAGE=ros-jazzy-glim-ros-cuda12.6
ARG CUDA_RUNTIME_PACKAGE=cuda-cudart-12-6

USER root

# Install the ROS 2 tools we rely on from the Jazzy overlay.
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      curl \
      build-essential \
      cmake \
      iproute2 \
      ros-jazzy-ament-cmake \
      python3-colcon-common-extensions \
      python3-yaml \
      sudo \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-nav2-bringup \
      ros-jazzy-navigation2 \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-ros2bag \
      ros-jazzy-rosbag2-storage-default-plugins \
      ros-jazzy-teleop-twist-keyboard; \
    rm -rf /var/lib/apt/lists/*

# GLIM provides the Phase 2 LiDAR-IMU SLAM stack. Prefer the CUDA Jazzy package.
# The GLIM package links against libcudart, which is not provided by the NVIDIA
# container runtime driver mounts.
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends curl gpg; \
    curl -fsSL -o /tmp/cuda-keyring.deb \
      "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"; \
    dpkg -i /tmp/cuda-keyring.deb; \
    rm -f /tmp/cuda-keyring.deb; \
    curl -s --compressed "https://koide3.github.io/ppa/ubuntu2404/KEY.gpg" \
      | gpg --dearmor > /etc/apt/trusted.gpg.d/koide3_ppa.gpg; \
    echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" \
      > /etc/apt/sources.list.d/koide3_ppa.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      "${CUDA_RUNTIME_PACKAGE}" \
      libboost-all-dev \
      libglfw3-dev \
      libiridescence-dev \
      libmetis-dev \
      "${GLIM_ROS_PACKAGE}"; \
    ldconfig; \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=177 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///config/cyclonedds_config.xml

RUN cat <<'EOF' >/etc/ros_setup.sh
if [ -f /opt/ros/spaceros/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/spaceros/setup.bash
fi
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f /workspace/astrobot-lab/astrobot-lab/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-177}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///config/cyclonedds_config.xml}"
EOF
RUN chmod 644 /etc/ros_setup.sh && \
    echo 'source /etc/ros_setup.sh' >> /etc/bash.bashrc

COPY docker/dev_entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENV BASH_ENV=/etc/ros_setup.sh
EXPOSE 8765

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
