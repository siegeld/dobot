#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source our workspace
source /ros2_ws/install/setup.bash

# Prefer volume-mounted dobot-ros source over the baked-in copy,
# so code edits take effect without rebuilding the image.
if [ -d /dobot-ros/dobot_ros ]; then
    export PYTHONPATH="/dobot-ros:${PYTHONPATH}"
fi

# Execute command
exec "$@"
