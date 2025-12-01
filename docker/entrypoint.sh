#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source our workspace
source /ros2_ws/install/setup.bash

# Execute command
exec "$@"
