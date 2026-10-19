ARG SPACEROS_TAG=main-dev
FROM osrf/space-ros:${SPACEROS_TAG}
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
      python3-fastapi \
      ros-jazzy-ament-cmake \
      python3-colcon-common-extensions \
      python3-uvicorn \
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

# GLIM provides the LiDAR-IMU SLAM stack. We install the lower-level dependencies
# from Koide's PPA, then build our GLIM and glim_ros2 forks from source below.
# The GLIM stack links against libcudart, which is not provided by the NVIDIA
# container runtime driver mounts.
RUN set -ex; \
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
      cuda-cudart-dev-12-6 \
      cuda-nvcc-12-6 \
      libboost-all-dev \
      libglfw3-dev \
      libgtsam-notbb-dev \
      libgtsam-points-cuda12.6-dev \
      libiridescence-dev \
      libmetis-dev \
      libopencv-dev \
      ros-jazzy-ament-cmake-auto \
      ros-jazzy-cv-bridge \
      ros-jazzy-geometry-msgs \
      ros-jazzy-image-transport \
      ros-jazzy-nav-msgs \
      ros-jazzy-rclcpp \
      ros-jazzy-rclcpp-components \
      ros-jazzy-rosbag2-compression \
      ros-jazzy-rosbag2-cpp \
      ros-jazzy-rosbag2-storage \
      ros-jazzy-sensor-msgs \
      ros-jazzy-tf2-ros; \
    ldconfig; \
    rm -rf /var/lib/apt/lists/*

# Build GLIM core and ROS wrapper from our forks. The core fork exposes
# sensors.imu_gravity_magnitude, so lunar gravity is configured in JSON instead
# of applied by replacing a single binary .so at runtime.
COPY astrobot-lab/src/glim /tmp/glim_ws/src/glim
COPY astrobot-lab/src/glim_ros2 /tmp/glim_ws/src/glim_ros2
RUN set -ex; \
    rm -f /tmp/glim_ws/src/glim/COLCON_IGNORE /tmp/glim_ws/src/glim_ros2/COLCON_IGNORE; \
    . /opt/ros/spaceros/setup.sh; \
    . /opt/ros/jazzy/setup.sh; \
    cd /tmp/glim_ws; \
    colcon build \
      --merge-install \
      --install-base /opt/astrobot/glim_source \
      --packages-select glim glim_ros \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=ON \
      -DBUILD_WITH_MARCH_NATIVE=OFF \
      -DCMAKE_PREFIX_PATH="/opt/ros/jazzy;/usr/local"; \
    rm -rf /tmp/glim_ws; \
    ldconfig; \
    rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/local/cuda-12.6/bin:${PATH}

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
if [ -f /opt/astrobot/glim_source/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/astrobot/glim_source/setup.bash
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
EXPOSE 8765 8080

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
