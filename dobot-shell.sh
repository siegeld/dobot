#!/bin/bash
# Start the Dobot interactive shell
# Requires dobot-driver.sh to be running in another terminal

set -e
cd "$(dirname "$0")"

docker compose run --rm dobot dobot-ros
