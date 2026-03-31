# CLAUDE.md

## Project Overview

Dobot CR5 collaborative robot controller with two interfaces:
- **dobot-cr** (`dobot_cr/`) — Direct TCP/IP control, runs natively via `./dobot-cr.sh`
- **dobot-ros** (`dobot-ros/`) — ROS2-based control, runs in Docker via `./dobot.sh` / `./dobot-shell.sh`

The active development focus is the ROS2 path.

## Architecture

- **Docker** runs: ROS2 driver (`cr_robot_ros2`), gripper node (`gripper_node.py`), web dashboard
- **Host** runs: wrapper shell scripts that `docker compose run` into the container
- **dobot-ros CLI** lives at `dobot-ros/dobot_ros/` — volume-mounted into Docker, so code edits take effect without rebuild
- **State reads** (joint angles, cartesian pose, robot mode, gripper) come from **ROS2 topic subscriptions**, not service calls to the C++ driver's dashboard port
- **Commands** (enable, move, jog, speed) go through **ROS2 service calls** to the C++ driver
- **Gripper** has its own ROS2 node (`gripper_node.py`) that owns the single Modbus connection. Clients subscribe to `/gripper/state` topic and send commands via `/gripper/init` service and `/gripper` action
- **DDS transport**: FastRTPS with UDP-only profile (`docker/fastrtps_profile.xml`) is required for cross-container ROS2 communication. Without it, service discovery works but data exchange fails (shared memory transport broken between containers).

## System Startup

- `./startup.sh` — starts the full system (driver + gripper node + web dashboard)
- `./startup.sh --build` — rebuild Docker image first, then start
- `./startup.sh --stop` — stop everything (`docker compose down`)
- Both services use `restart: unless-stopped` — the driver auto-reconnects when the robot powers on, and recovers if the robot reboots
- The web dashboard runs independently of the driver; it shows a disconnected state until the driver connects to the robot
- Web dashboard: http://localhost:7070

## Development Workflow

- Edit `dobot-ros/` code on host → changes are live in next `docker compose run` (no rebuild)
- Rebuild only needed when changing: `dobot_actions/`, `DOBOT_6Axis_ROS2_V4/`, or `docker/Dockerfile`
- `docker compose build` — rebuild image
- `docker compose run --rm dobot dobot-ros <cmd>` — run CLI commands

## Key Files

- `startup.sh` — system startup script (driver + gripper + web dashboard)
- `docker-compose.yml` — service definitions, volume mounts, FastRTPS profile
- `docker/Dockerfile` — ROS2 image build (Jazzy LTS)
- `docker/entrypoint.sh` — sources ROS2, sets PYTHONPATH for volume-mounted code
- `docker/fastrtps_profile.xml` — forces UDP transport for cross-container DDS
- `.env` — robot IP and type (not committed)
- `dobot-ros/dobot_ros/ros_client.py` — ROS2 client (topic subscriptions for state, services for commands, gripper via gripper_node)
- `dobot-ros/dobot_ros/gripper_node.py` — standalone ROS2 node that owns the gripper Modbus connection
- `dobot-ros/dobot_ros/shell.py` — interactive shell (REPL)
- `dobot-ros/dobot_ros/cli.py` — click-based CLI commands
- `dobot-ros/dobot_ros/web/` — FastAPI web dashboard (WebSocket state push at 5Hz)

## Gripper

DH Robotics gripper via Modbus TCP through Dobot's internal gateway at 127.0.0.1:60000.
Position range: 0 (closed) to 1000 (open). Must call `gripper init` before use.

**RS-485 setup**: The Modbus TCP gateway on port 60000 requires the tool RS-485 interface to be configured first. `gripper_node.py` calls `SetTool485(115200)` automatically on startup, which configures the RS-485 baud rate and enables the gateway. No DHGrip plugin needed.

## Gotchas

- Root-owned `__pycache__` can appear in `dobot-ros/` from Docker runs — `.dockerignore` excludes them from builds, but may need `sudo rm -rf` if they cause permission issues
- `dobot_ros_config.yaml` was historically created as a directory by root — should be a file (or absent)
- The `check_connection` timeout uses `service_timeout` from config (default 5s) — UDP discovery between containers can take a few seconds on first call
