FROM omnilrs:4.5.0-humble   

ARG SIM_USER
ARG ASTRO_UID
ARG ASTRO_GID

USER root   
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp sudo iproute2&& \
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
    ROS_DOMAIN_ID=10

RUN echo 'source /opt/ros/humble/setup.bash' >  /etc/ros_setup.sh && \
    echo 'source /etc/ros_setup.sh'          >> /etc/bash.bashrc && \
    chmod 644 /etc/ros_setup.sh
ENV BASH_ENV=/etc/ros_setup.sh

USER ${SIM_USER}
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/sim_entrypoint.sh"]
CMD ["bash"]