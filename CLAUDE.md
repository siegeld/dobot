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

## Gripper Model

DH Robotics **AG-105** (not AG-95 as originally assumed). 105mm stroke, same Modbus register map as AG-95. Flange-to-gripper-tip distance: **203mm (8 inches)**.

**Tool offset**: `Tool(1)` is pre-configured on the robot controller as `{0, 0, 203, 0, 0, 0}` (tool-Z offset, no rotation). The web dashboard's Tool-frame dropdown switches between:
- **Tool 0** (wrist / flange) — cartesian pose is the wrist
- **Tool 1** (fingertip) — cartesian pose is the fingertip. Servo rotations pivot around the fingertip and the floor guard re-computes its Z minimum using the 203 mm tool length.

Tool selection is applied via the `Tool` ROS service, persisted to the settings store, and re-applied on container restart. There is no `GetTool` service — the controller-side definition can't be read back, so tool length is a configured value in the settings store (default 203 mm).

**Picking mode** — intended combo for top-down picks via SpaceMouse: select Tool 1, click **Vertical** in Robot Control (orients RX=180, RY=0, preserves RZ), then flip **Lock** on. The servo tester then force-projects every ServoP target to RX=180 / RY=0 and zeros puck RX/RY velocity, so the gripper stays perpendicular to the table as the fingertip XY/Z tracks the puck. Yaw (RZ) is left free for jaw alignment.

**Historical workspace-protection note**: early attempts at setting `SetTool(1, {0,0,203,0,0,0})` triggered error 1479 because the TCP Z dropped below the controller's floor protection limit. Once the workspace protection limits were loosened in Dobot Studio, `Tool(1)` works and is now the canonical fingertip frame.

## Vision pick flow

- `/api/vision/execute` is the canonical entry point. Don't bypass it.
- It **auto-switches to Tool 1**, orients the wrist vertical, enables `lock_vertical`, and **re-plans against the post-setup pose** before any strategy motion runs. The setup phase is idempotent — steps already in place are skipped.
- Every pick shows a **modal confirmation** over `/ws/state` (`pick_confirm` message type, broker at `dobot_ros/web/pick_confirm.py`). The browser replies via `POST /api/pick/confirm`. User Cancel → 409, 120 s timeout → 408.
- Strategies have two methods: pure `plan(ctx)` (testable, no I/O) and `execute(ctx, plan, client, confirm_fn, servo=None)` which owns the motion. `motion_mode` class attr is `"movl"` or `"servo"`.
- When adding a strategy, use `ctx.pose_z_from_table_z(height_above_table)` instead of `table_z + clearance` — it handles the Tool 0 vs Tool 1 frame difference. Don't preserve `current_pose[3:5]` for wrist orientation; hard-code RX=180 / RY=0 (setup guarantees this is where the wrist already is).
- Servo strategies receive the running `ServoTester`; the orchestrator starts it with the post-setup pose as the anchor and restores the tester's prior `lock_vertical` and `max_velocity_xyz` on exit.

## Calibration

Two independent calibrations stored on the robot web server:

**Workspace / Table plane** (`table_plane.json`):
- 4 corner points recorded by touching gripper tip to table corners
- Defines the table plane (for safe Z) and workspace XY bounds
- Min clearance setting prevents gripper from going closer than N mm to table
- Workspace test moves: Center, C1-C4 at configurable height above table

**Camera-to-robot transform** (stored on camera server at `calibration.json`):
- Point correspondences: click pixel in camera image + record robot XYZ
- Solved via SVD (Kabsch algorithm) on camera server
- Camera depth noise is ~±38mm at working distance — XY is reliable, Z is not
- For picking: use camera for XY, use table plane for Z

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
