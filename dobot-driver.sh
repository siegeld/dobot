#!/bin/bash
# Start the Dobot ROS2 driver
# Reads ROBOT_IP and ROBOT_TYPE from .env file

set -e
cd "$(dirname "$0")"

echo "Starting Dobot ROS2 driver..."
echo "Robot IP: ${ROBOT_IP:-from .env}"
echo "Robot Type: ${ROBOT_TYPE:-from .env}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

docker compose up dobot-driver
