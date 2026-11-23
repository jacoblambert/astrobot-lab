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

# GLIM provides the LiDAR-IMU SLAM stack. CUDA comes from NVIDIA's repository; GTSAM and
# gtsam_points are built from pinned tags below (the PPA republishes newer snapshots under the
# same version numbers), then our GLIM and glim_ros2 forks.
# The GLIM stack links against libcudart, which is not provided by the NVIDIA
# container runtime driver mounts.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends curl gpg; \
    curl -fsSL -o /tmp/cuda-keyring.deb \
      "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"; \
    dpkg -i /tmp/cuda-keyring.deb; \
    rm -f /tmp/cuda-keyring.deb; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      "${CUDA_RUNTIME_PACKAGE}" \
      cuda-cudart-dev-12-6 \
      cuda-nvcc-12-6 \
      git \
      libboost-all-dev \
      libeigen3-dev \
      libglfw3-dev \
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

# GTSAM 4.3a0 and gtsam_points 1.2.1: the versions every validated run used (headers verified
# against the earlier PPA builds). Options match those builds: no TBB, system Eigen and METIS.
ARG GTSAM_TAG=4.3a0
ARG GTSAM_POINTS_TAG=v1.2.1
ARG CUDA_ARCHITECTURES="75;80;86;89;90"
# cuSPARSE/cuSOLVER/cuRAND dev packages only provide headers for gtsam_points' error helpers
# (the built library links just libcudart); they are purged in the same layer.
RUN set -ex; \
    apt-get update; \
    apt-get install -y --no-install-recommends libcusparse-dev-12-6 libcusolver-dev-12-6 \
      libcurand-dev-12-6; \
    git clone --depth 1 --branch "${GTSAM_TAG}" https://github.com/borglab/gtsam /tmp/gtsam; \
    cmake -S /tmp/gtsam -B /tmp/gtsam/build -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON \
      -DGTSAM_USE_QUATERNIONS=OFF -DGTSAM_TANGENT_PREINTEGRATION=ON \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=ON -DGTSAM_SUPPORT_NESTED_DISSECTION=ON; \
    cmake --build /tmp/gtsam/build -j"$(nproc)"; \
    cmake --install /tmp/gtsam/build; \
    git clone --depth 1 --branch "${GTSAM_POINTS_TAG}" https://github.com/koide3/gtsam_points /tmp/gtsam_points; \
    cmake -S /tmp/gtsam_points -B /tmp/gtsam_points/build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_WITH_CUDA=ON -DBUILD_WITH_TBB=OFF -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_MARCH_NATIVE=OFF \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc \
      -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}"; \
    cmake --build /tmp/gtsam_points/build -j"$(nproc)"; \
    cmake --install /tmp/gtsam_points/build; \
    rm -rf /tmp/gtsam /tmp/gtsam_points; \
    apt-get purge -y 'libcusparse*' 'libcusolver*' 'libcurand*' 'libcublas*' 'libnvjitlink*'; \
    rm -rf /var/lib/apt/lists/*; \
    ldconfig

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
