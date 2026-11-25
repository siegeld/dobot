# Dobot CR ROS2 Driver - Docker Image
# Based on ROS2 Jazzy (latest LTS)

FROM ros:jazzy

# Install build dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-evdev \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    krb5-user libkrb5-dev python3-dev \
    && rm -rf /var/lib/apt/lists/*
# Not the host's krb5.conf: that one uses a KEYRING ccache a container cannot have.
COPY docker/krb5.conf /etc/krb5.conf

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Stage 1: Build stable Dobot packages (cached - these rarely change)
COPY DOBOT_6Axis_ROS2_V4/dobot_msgs_v4 src/dobot_msgs_v4
COPY DOBOT_6Axis_ROS2_V4/dobot_bringup_v4 src/dobot_bringup_v4
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select dobot_msgs_v4 cr_robot_ros2

# Stage 2: Build dobot_actions (changes more frequently)
COPY dobot_actions src/dobot_actions
RUN . /opt/ros/jazzy/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    colcon build --symlink-install --packages-select dobot_actions

# Install the dobot-ros CLI (creates the dobot-ros command).
# At runtime, PYTHONPATH is set so the volume-mounted source takes
# precedence — code changes work without rebuild.
COPY dobot-ros /dobot-ros-pkg
# Sign-in is the shared homelab-auth package, installed as a pinned wheel from NFS
# (compose additional_contexts → the `wheels` build context), never from the network.
# [kerberos] builds gssapi against libkrb5 (no wheel for this Python), hence the dev
# packages above; krb5-user stays for the runtime libraries and klist.
COPY --from=wheels homelab_auth-*.whl /tmp/wheels/
RUN pip3 install --break-system-packages --find-links /tmp/wheels \
      /dobot-ros-pkg requests Pillow python-multipart \
      "homelab-auth[kerberos,fastapi]==1.4.0" "itsdangerous>=2.2,<3" \
    && rm -rf /tmp/wheels

# Source workspace on shell entry
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default entrypoint sources ROS and workspace
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
