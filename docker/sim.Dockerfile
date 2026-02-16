FROM isaac-sim-omnilrs:latest

ARG SIM_USER
ARG ASTRO_UID
ARG ASTRO_GID

USER root   

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-fastrtps-cpp sudo iproute2 && \
    rm -rf /var/lib/apt/lists/*

RUN if ! getent group "${ASTRO_GID}" >/dev/null; then \
        groupadd -g "${ASTRO_GID}" "${SIM_USER}"; \
    fi && \
    if ! getent passwd "${ASTRO_UID}" >/dev/null; then \
        useradd -m -u "${ASTRO_UID}" -g "${ASTRO_GID}" -s /bin/bash "${SIM_USER}"; \
    fi && \
    echo "${SIM_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"${SIM_USER}"

COPY sim_entrypoint.sh /usr/local/bin/sim_entrypoint.sh
RUN chmod +x /usr/local/bin/sim_entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///config/cyclonedds_config.xml \
    ROS_DOMAIN_ID=10 \
    ROS_DISTRO=jazzy

RUN echo 'source /opt/ros/jazzy/setup.bash' >  /etc/ros_setup.sh && \
    echo 'source /etc/ros_setup.sh'          >> /etc/bash.bashrc && \
    chmod 644 /etc/ros_setup.sh

RUN set -eux; \
    mkdir -p /isaac-sim/kit/cache /isaac-sim/kit/logs /commandhistory "/home/${SIM_USER}/Documents/Kit/shared"; \
    touch /commandhistory/.bash_history; \
    chown -R "${ASTRO_UID}:${ASTRO_GID}" /isaac-sim/kit /commandhistory "/home/${SIM_USER}/Documents"

RUN cat <<'EOF' >/etc/profile.d/omnilrs_aliases.sh
alias kk='clear'
alias tt='tmux new'
alias isaacpy='/isaac-sim/python.sh'
alias isaacapp='/isaac-sim/runapp.sh'
alias roskb='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

if [ -z "${PROMPT_COMMAND:-}" ]; then
  PROMPT_COMMAND='history -a'
elif [[ "${PROMPT_COMMAND}" != *'history -a'* ]]; then
  PROMPT_COMMAND="${PROMPT_COMMAND};history -a"
fi
export PROMPT_COMMAND
export HISTFILE=${HISTFILE:-/commandhistory/.bash_history}

if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
EOF
RUN chmod 644 /etc/profile.d/omnilrs_aliases.sh && \
    echo 'source /etc/profile.d/omnilrs_aliases.sh' >> /etc/bash.bashrc

ENV BASH_ENV=/etc/ros_setup.sh

USER ${SIM_USER}
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/sim_entrypoint.sh"]
CMD ["bash"]
