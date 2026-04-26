ARG SPACEROS_TAG=main-dev
FROM osrf/space-ros:${SPACEROS_TAG}

USER root

# Install the ROS 2 tools we rely on from the Jazzy overlay.
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      curl \
      iproute2 \
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
