# Architecture

High-level layout of the Dobot CR5 control stack. For the HTTP / WebSocket
surface see `API.md`. For user-facing change history see `CHANGELOG.md`.

## System diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Host (jupiter)                                                          │
│                                                                          │
│  ┌────────────────┐ http://localhost:7070    ┌───────────────────────┐   │
│  │  Web browser   │ ─────────────────────────▶│  nginx :80 → :443    │   │
│  │  (dashboard)   │ wss:// /ws/state, /ws/... │  reverse proxy       │   │
│  └────────────────┘ ◀────────────────────────│  /dashboard/ → :7070  │   │
│                                              └───────────────────────┘   │
│                                                         │                │
│                                                         ▼                │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │  docker compose network                                          │    │
│  │                                                                  │    │
│  │  ┌──────────────────┐   ROS2 DDS (FastRTPS UDP profile)          │    │
│  │  │  dobot-web       │◄────────────────────────┐                  │    │
│  │  │  FastAPI :7070   │                         │                  │    │
│  │  │  + DobotRosClient│                         │                  │    │
│  │  │  + SpaceMouse    │                         │                  │    │
│  │  │  + ServoTester   │                         │                  │    │
│  │  │  + VLA executor  │                         │                  │    │
│  │  └──────────────────┘                         │                  │    │
│  │           │                                   │                  │    │
│  │           │  writes /tmp/dobot-shared/        │                  │    │
│  │           │  driver_restart                   │                  │    │
│  │           ▼                                   │                  │    │
│  │  ┌──────────────────┐         ┌────────────────────────┐         │    │
│  │  │  dobot-driver    │         │  dobot-gripper         │         │    │
│  │  │  Python wrapper  │         │  gripper_node.py (ROS) │         │    │
│  │  │   + C++ ROS node │         │  Modbus TCP :60000     │         │    │
│  │  │  cr_robot_ros2   │         │  via robot gateway     │         │    │
│  │  └──────────────────┘         └────────────────────────┘         │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│                   │ TCP 29999 (commands)                                 │
│                   │ TCP 30004 (realtime feedback)                        │
│                   │ TCP 60000 (Modbus via SetTool485 gateway)            │
└───────────────────┼──────────────────────────────────────────────────────┘
                    ▼
              ┌──────────────┐
              │  Robot (CR5) │
              │  10.11.6.68  │
              └──────────────┘
```

## Containers

Every service is launched from `docker-compose.yml`. All three use
`network_mode: host` so they can reach the robot directly without NAT.

| Service | Image | What it runs |
|---|---|---|
| `dobot-driver` | `dobot-dobot-driver` | Python wrapper that launches the C++ `cr_robot_ros2` ROS node; watchdog loop; waits for the robot before launching |
| `dobot-gripper` | (shared) | `gripper_node.py` — owns the single Modbus TCP connection through the robot's internal RS-485 gateway |
| `dobot-web` | (shared) | FastAPI app at `:7070`; serves the web dashboard; hosts the ServoTester, SpaceMouse reader, and VLA executor |

The nginx reverse proxy lives on the host (not a container). Cert + key are
at `/etc/nginx/ssl/jupiter.siegel.com/`.

## Control flow

### Reads (state)
Joint angles, cartesian pose, robot mode, and gripper state all come from
**ROS 2 topic subscriptions** inside `DobotRosClient` (not from direct TCP
calls to the robot's dashboard port). The FastAPI poll thread reads those
cached values at 5 Hz and pushes snapshots over `/ws/state` to the
browser.

Relevant topics (robot side, published by the C++ driver):

- `/joint_states_robot` — joint angles (radians)
- `/tool_vector_actual` — cartesian pose (mm + deg)
- `/feed_info_json` — robot mode, error state, digital IO, etc. encoded
  as JSON

Relevant topics (gripper node):
- `/gripper/state` — current position + init status at 5 Hz

### Commands (motion + enable)
Commands go through **ROS 2 service calls** from `DobotRosClient` to the
C++ driver, which forwards them over TCP 29999:

- `EnableRobot`, `DisableRobot`, `ClearError`, `Stop`
- `MovJ`, `MovL`, `RelMovLUser`, `RelMovLTool`, `ServoP`, `ServoJ`
- `SpeedFactor`, `StartDrag`, `StopDrag`
- `SetTool485`, `ModbusCreate`, `GetHoldRegs`, `SetHoldRegs` (used by the
  gripper node)

### Servo streaming (ServoP)
The `ServoTester` (and via it, the SpaceMouse pendant and VLA executor)
streams ServoP targets at up to 100 Hz. It has two input modes:

1. **Position** — `set_target_offset(offset_6d)` or a `Pattern`. The tick
   loop advances `_commanded_offset` toward the target at
   `max_velocity_{xyz,rpy}` per tick.
2. **Rate / velocity** — `set_target_velocity(v_6d)`. The tick loop
   integrates `v × dt` directly. Zero velocity means no motion: release
   the SpaceMouse → puck returns to center → `v = 0` → robot stops that
   tick with no residual catch-up.

Both modes share the same hard velocity cap and workspace-bounds clamp.

**Lock vertical** (`ServoConfig.lock_vertical`) is an optional mode on
top of either input path. When on, the tick loop zeroes RX/RY velocity
before integration and force-projects the final commanded pose to
RX=180, RY=0 before `servo_p`. This guarantees the tool Z axis points
straight down regardless of puck noise, Euler-integration roundoff, or
TCS re-solves near the vertical configuration. RZ (yaw) is untouched.
Intended for top-down picks in Tool 1, where it keeps the gripper
perpendicular to the table while the fingertip XY/Z tracks the puck.

### Tool coordinate system
The web dashboard lets the operator pick which tool the ServoTester /
cartesian pose reporting operates in. The selection is applied via the
`Tool` ROS service and persisted to the settings store:

- **Tool 0** — flange / wrist. `[X,Y,Z,RX,RY,RZ]` is the wrist pose.
  Rotations pivot around the wrist; a 203 mm gripper swings on an arc.
- **Tool 1** — pre-configured on the robot controller (AG-105 with a
  203 mm tool-Z offset). `[X,Y,Z,RX,RY,RZ]` is the fingertip pose.
  Rotations pivot around the fingertip; the fingertip is the natural
  frame for pick/place since it matches the calibrated table coordinates.

The ServoTester's floor guard (`SafetyLimits.from_table_plane`) re-reads
the configured `tool_length_mm` when the tool changes so the Z minimum
matches the active tool — the fingertip cannot be driven into the table
regardless of which tool is selected.

### Safety boundaries
- **Workspace clamp** (`dobot_ros/vla/safety.py`) is the last thing before
  a pose hits ServoP: X/Y ±1500 mm, Z ≥ table + margin, Z ≤ 1500 mm, rots
  ±360°. Every commanded pose passes through this.
- **Velocity cap** enforced in the ServoTester tick loop governs how fast
  that commanded pose can change.
- **Motion lock** (`_acquire_motion` / `_release_motion`) is a single
  mutually-exclusive lock across the servo tester, VLA executor, SpaceMouse
  reader, and high-level pick/workspace flows. Prevents two motion sources
  from fighting.
- **Driver watchdog** (feedback-stale detector) and **SpaceMouse on-disarm
  hook** exist to self-heal stuck states without operator intervention.
- **Pick-confirm broker** (`dobot_ros/web/pick_confirm.py`) is a sync
  ask/reply primitive owned by the web layer. Pick-execution threadpool
  workers call `ask(summary)` which pushes a `pick_confirm` message over
  the existing `/ws/state` WebSocket via
  `asyncio.run_coroutine_threadsafe` and blocks on a `threading.Event`.
  Browser POSTs to `/api/pick/confirm`; broker resolves the event; worker
  continues. Handles timeout, cancel, reconnect replay, and E-STOP
  `cancel_all()`. The vision pick orchestrator uses it for the unified
  pre-pick modal; strategies can attach per-waypoint confirm gates.

## Key files

| Path | Role |
|---|---|
| `docker-compose.yml` | Service definitions, host networking, volume mounts |
| `docker/Dockerfile` | ROS 2 Jazzy image build |
| `docker/entrypoint.sh` | Sources ROS 2, sets `PYTHONPATH` for the volume-mounted code |
| `docker/fastrtps_profile.xml` | Forces UDP transport for cross-container DDS (shared-memory is unreliable) |
| `dobot-ros/dobot_ros/ros_client.py` | Python wrapper around ROS 2 services + topics |
| `dobot-ros/dobot_ros/driver.py` | Driver-container wrapper loop (launches + watchdogs the C++ ROS node) |
| `dobot-ros/dobot_ros/gripper_node.py` | Gripper ROS 2 node (Modbus owner) |
| `dobot-ros/dobot_ros/servo/tester.py` | ServoP streaming bench (position + rate modes, velocity cap) |
| `dobot-ros/dobot_ros/servo/patterns.py` | Sine / circle / lissajous / square pattern generators |
| `dobot-ros/dobot_ros/spacemouse/reader.py` | HID → velocity-command pipeline |
| `dobot-ros/dobot_ros/vla/executor.py` | OpenVLA-OFT chunked-inference streaming loop |
| `dobot-ros/dobot_ros/vla/safety.py` | Workspace clamps, limit loading from `table_plane.json` |
| `dobot-ros/dobot_ros/web/server.py` | FastAPI + WebSocket routes (see `API.md`) |
| `dobot-ros/dobot_ros/web/pick_confirm.py` | `PickConfirmBroker` — sync `ask()` / `reply()` primitive for pick-time modals |
| `dobot-ros/dobot_ros/strategies/` | Pick-strategy plugins (`base.py` ABC, `simple_top_down.py`, `angled_approach.py`, `servo_top_down.py`); per-strategy defaults in `params/*.json` |
| `dobot-ros/dobot_ros/web/static/js/app.js` | Dashboard core (state, WS, display, robot/jog/gripper wiring) |
| `dobot-ros/dobot_ros/web/static/js/app-*.js` | Feature modules: vla, servo, settings, calibration, vision, sidebar, misc |
| `DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/src/` | Vendor C++ driver source (submodule) |
| `/etc/nginx/conf.d/dobot.conf` | Reverse-proxy + landing-page rules |
| `/var/www/siegel-services/index.html` | Host landing page listing services |

## Configuration sources

| File | What it holds | How it's written |
|---|---|---|
| `.env` | Robot IP, robot type | Hand-edited; read at startup |
| `dobot-ros/dobot_ros/web/table_plane.json` | Calibrated table corners | Written by the Calibration tab |
| `dobot-ros/dobot_ros/web/settings/last_settings.json` | Persistent UI/servo/VLA/spacemouse settings | `SettingsStore.patch()` via `/api/settings/current` and group-specific endpoints |
| `dobot-ros/dobot_ros/web/settings/saved/` | Named settings snapshots | `/api/settings/saved/{name}` POST |
| Robot web server — `table_plane.json`, camera `calibration.json` | Alternate/legacy storage | Rarely used; the FastAPI copy is canonical |
