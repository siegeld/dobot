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

DH Robotics **AG-95** adaptive gripper via Modbus TCP through Dobot's internal gateway at 127.0.0.1:60000.
Position range: 0 (closed) to 1000 (open). Must call `gripper init` before use.

**AG-95 specifics**: The AG-95 has no speed register (0x0104 is ignored). Speed is controlled by force — higher force = faster movement. The speed register code is kept for future PGC/PGE grippers which do have independent speed control. Default force is 20%.

**RS-485 setup**: `SetTool485(115200)` creates the Modbus TCP gateway on port 60000. The gripper node calls this once at startup. No DHGrip plugin needed.

**Modbus connection pattern** (derived from the working DHGrip Lua plugin in `DHGrip_v2-1-4-stable/`):
- A persistent connection (SetTool485 → ModbusCreate once) is used for state polling at 5Hz
- Write operations (move, init) reconnect first (ModbusClose → SetTool485 → ModbusCreate) to get a fresh connection, matching the plugin's `NewConnection` pattern
- All Modbus operations are serialized via a threading lock (equivalent to the plugin's `Lock485`/`UnLock485`)
- Multi-register reads return comma-separated values inside braces: `0,{1,1,712},GetHoldRegs(...)` — parsed with `re.search(r'\{([^}]+)\}', ...)`

**Web dashboard gripper**: Commands (open/close/move) are fire-and-forget — the HTTP endpoint returns immediately after sending the ROS2 action goal. Position and state updates flow through the `/gripper/state` topic → WebSocket at 5Hz. The position slider updates live from WebSocket data.

## Driver Watchdog

The C++ driver connects to the robot on two ports: 29999 (commands) and 30004 (realtime feedback for joint angles, pose, mode). If the robot reboots while the driver is running, port 30004 can go stale — the driver keeps publishing the last-known joint values at 10Hz but they never update. Commands through 29999 still work.

**Auto-recovery**: The web dashboard's poll thread monitors the `header.stamp` of `/joint_states_robot` messages. If the timestamp stops advancing for 5 seconds, it signals the driver to restart by touching `/tmp/dobot-shared/driver_restart` (a shared Docker volume). The driver's wrapper loop detects the signal file, kills the ROS node, and relaunches it — all without restarting the container.

## Robot Modes

The mode values from the feedback port (used in the web dashboard):

| Value | Name | Description |
|-------|------|-------------|
| 1 | INIT | Initializing |
| 2 | BRAKE_OPEN | Brakes released |
| 3 | POWEROFF | Powered off |
| 4 | DISABLED | Motors off |
| 5 | ENABLE | Enabled, idle |
| 6 | BACKDRIVE | Freedrive/teach by hand |
| 7 | RUNNING | Executing motion |
| 8 | SINGLE_STEP | Single step mode |
| 9 | ERROR | Alarm (highest priority) |
| 10 | PAUSE | Motion paused |
| 11 | COLLISION | Collision detected |

**Note**: These differ from some online Dobot docs that start at 0 or have a different ordering. The mapping above is from the official `TCP-IP-Python-V4` SDK (`dobot_api.py`).

## Gotchas

- Root-owned `__pycache__` can appear in `dobot-ros/` from Docker runs — `.dockerignore` excludes them from builds, but may need `sudo rm -rf` if they cause permission issues
- `dobot_ros_config.yaml` was historically created as a directory by root — should be a file (or absent)
- The `check_connection` timeout uses `service_timeout` from config (default 5s) — UDP discovery between containers can take a few seconds on first call
