#!/bin/bash
# Run dobot-ros commands
# Usage: ./dobot.sh [command] [args...]
#
# Examples:
#   ./dobot.sh              # Start interactive shell
#   ./dobot.sh position     # Get robot position
#   ./dobot.sh jog x 10     # Jog X by 10mm
#   ./dobot.sh enable       # Enable robot
#   ./dobot.sh --help       # Show help

set -e
cd "$(dirname "$0")"

if [ $# -eq 0 ]; then
    # No args - start interactive shell
    docker compose run --rm dobot dobot-ros
else
    # Pass args to dobot-ros
    docker compose run --rm dobot dobot-ros "$@"
fi
