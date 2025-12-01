# Dobot CR ROS2 Driver - Docker Image
# Based on ROS2 Jazzy (latest LTS)

FROM ros:jazzy

# Install build dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy Dobot ROS2 packages
COPY DOBOT_6Axis_ROS2_V4/dobot_msgs_v4 src/dobot_msgs_v4
COPY DOBOT_6Axis_ROS2_V4/dobot_bringup_v4 src/dobot_bringup_v4

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Copy and install the dobot-ros CLI
COPY dobot-ros /dobot-ros
RUN pip3 install --break-system-packages /dobot-ros

# Source workspace on shell entry
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default entrypoint sources ROS and workspace
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
