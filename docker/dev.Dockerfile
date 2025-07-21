# .devcontainer/Dockerfile
FROM osrf/space-ros

# sudo
USER root
RUN apt-get update && apt-get install -y --no-install-recommends sudo

# Permissions
ARG DEV_USER
ARG ASTRO_UID
ARG ASTRO_GID
RUN if ! getent group "${ASTRO_GID}" >/dev/null; then \
        groupadd -g "${ASTRO_GID}" "${DEV_USER}"; \
    fi && \
    if ! getent passwd "${ASTRO_UID}" >/dev/null; then \
        useradd -m -u "${ASTRO_UID}" -g "${ASTRO_GID}" -s /bin/bash "${DEV_USER}"; \
    fi && \
    echo "${DEV_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/"${DEV_USER}"

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl iproute2 && \
    rm -rf /var/lib/apt/lists/*

# Copy entrypoint, source ROS
COPY dev_entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN echo 'source /opt/spaceros/install/setup.bash' >  /etc/ros_setup.sh && \
    echo 'source /etc/ros_setup.sh'                >> /etc/bash.bashrc && \
    chmod 644 /etc/ros_setup.sh
ENV BASH_ENV=/etc/ros_setup.sh

USER ${DEV_USER}
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]