# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

See `ARCHITECTURE.md` for the current system layout and `API.md` for the
HTTP / WebSocket reference.

### Added

#### Web dashboard
- Full browser dashboard at `http://localhost:7070` with real-time WebSocket
  state updates at 5 Hz. Tabs for Dashboard, Calibration, Vision, VLA, Servo,
  Settings; persistent sidebar with 3D robot view, robot control, gripper,
  and joint/cartesian position.
- **3D robot visualization** with joint poses driven off the live state.
- **Sidebar card drag-and-drop reordering** — every sidebar card is
  draggable by its header; order persists to the settings store under
  `ui_sidebar.card_order`.
- **HTTPS via nginx** — port 443 reverse-proxies `/dashboard/` to the
  FastAPI server on `:7070`; `:80` redirects to `:443`; root `/` serves a
  landing page listing services on the host. Cert trusts the Siegel Family
  internal CA.
- **Popup windows** for the pendant and SpaceMouse UIs (`window.open()`
  with `popup=yes`) so the main dashboard stays usable behind them.
  Both pages ship web manifests so Chrome's "Install as app" produces
  chromeless standalone windows.

#### SpaceMouse pendant
- **Full SpaceMouse Wireless BT** integration over HID. Python evdev
  reader runs in two threads: a read loop and a 50 Hz control tick.
- **Rate control** — axes map to a 6-D velocity command, pushed to the
  servo tester via `set_target_velocity()`. Release the puck → zero
  velocity → robot stops on the very next tick. (Previous design
  integrated to a position target, which caused catch-up lag on release.)
- Configurable deadband, per-axis sign map, max XYZ / RPY velocity, and
  max excursion. Idle auto-disarm after N seconds of no motion. Buttons
  bind to gripper open/close when armed.
- Docker device passthrough: `/dev/input` is bind-mounted read-only and
  the container joins the host `input` group for HID access.

#### Servo tester
- ServoP streaming test bench with live tuning (servo rate, ServoP t,
  gain, aheadtime) and jog / pattern inputs.
- **Hard velocity caps** (`max_velocity_xyz` mm/s, `max_velocity_rpy`
  deg/s) enforced in the tick loop. XYZ is clamped as a 3-vector
  magnitude (direction preserved); RPY is clamped per axis. Applies to
  every input path — jog sliders, patterns, rate-control velocity, etc.
- Save / Reset buttons for tuning values; settings persist via the store
  and are pre-filled on page load.
- Pattern generators (sine, circle, lissajous, square) with configurable
  amplitudes and frequencies.
- CSV logging of every tick (t, offset, commanded pose, actual pose,
  latency) for offline analysis.

#### VLA (Vision-Language-Action)
- VLA executor infrastructure for OpenVLA-OFT chunked inference over
  ServoP streaming (see VLA.md). Safety clamps on deltas and absolute
  poses; workspace/velocity/gripper bounds.
- RLDS dataset builder for drag-teach demo recording.

#### Vision & calibration
- Calibration tab: table-plane recording, camera-to-robot transform,
  workspace tests with configurable heights, phase indicator walking
  users through Table → Correspondences → Solve.
- Vision tab: RealSense-fed camera feed with optional table-plane grid
  overlay, object detection overlays, plan preview, simulate, execute.
- **Pluggable pick strategies** — each exposes JSON-declared parameters
  that render as a dynamic form in the UI; parameters persist per slug.
- `vision_transform.py` — camera-to-robot geometry (Kabsch/SVD).

#### Gripper
- Standalone `gripper_node.py` ROS 2 node owns the single Modbus TCP
  connection through the robot's internal RS-485 gateway (port 60000).
  Clients subscribe to `/gripper/state` (5 Hz) and send commands via
  `/gripper/init` service and `/gripper` action. Eliminates multi-client
  Modbus index conflicts.
- Auto-setup: `SetTool485(115200)` called once at startup (no DHGrip
  plugin required).
- Web dashboard gripper commands are fire-and-forget (HTTP returns
  immediately; state comes via topic).

#### Driver & infrastructure
- **Driver watchdog**: the web-dashboard poll thread monitors the
  `header.stamp` of `/joint_states_robot`. If it stops advancing for
  5 s, the thread touches `/tmp/dobot-shared/driver_restart`; the
  driver wrapper sees the signal file, kills the ROS node, and
  relaunches it — recovery from feedback-port staleness without a
  container restart.
- **Auto-disarm hook on SpaceMouse reader** releases the server's motion
  lock when the reader self-disarms (idle timeout, device lost), so the
  next `/api/spacemouse/arm` succeeds without manual intervention.
- Passwordless `restart: unless-stopped` containers; driver auto-
  reconnects when the robot powers back up.

#### Testing
- 78+ unit and integration tests across `tests/servo/`, `tests/spacemouse/`,
  `tests/vla/`, and more. Velocity caps, rate-control integration, release-
  stops, excursion clamps, mode transitions.
- Six safety-audit passes against `dobot-ros/` resolved 114 distinct
  issues spanning motion-lock races, concurrent-pick guards, NaN / Inf
  validation, atomic settings-store writes, and XSS in activity logs.

### Changed
- **Dashboard JS refactor**: `web/static/js/app.js` split from 2569 lines
  into 1031-line core + 7 feature modules (`app-vla.js`, `app-servo.js`,
  `app-settings.js`, `app-calibration.js`, `app-vision.js`, `app-sidebar.js`,
  `app-misc.js`). Shared helpers are exposed via `window.DobotUI`.
- **SpaceMouse reader** switched from position-integration to pure rate
  control (see above).

### Fixed
- **`TcpClient::disConnect` fd leak** in the vendor C++ driver
  (`DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/src/tcp_socket.cpp`). Set
  `fd_ = -1` **before** calling `::close(fd_)`, so close ran on `-1`
  (no-op) and the real socket leaked. Over hours of operation the driver
  process accumulated 800+ leaked socket fds; the robot's 29999 dashboard
  service tracked the stale sessions as live and rejected new connects
  with `"Connection refused, IP:Port has been occupied"` until the robot
  was power-cycled. **Patch submitted upstream**
  (Dobot-Arm/DOBOT_6Axis_ROS2_V4 PR #14).
- SpaceMouse button click no longer opens an unresponsive iframe modal —
  switched to OS-level popup windows.
- Servo tester crash on start when `table_plane.json` used the list-of-
  lists corner format: `c.get("x", …)` blew up because `.get()` was
  evaluated before the isinstance fallback. Replaced with an explicit
  `isinstance(c, dict)` check outside the `.get()` call.
- Robot-mode label now maps to the correct SDK values (1=INIT, 2=BRAKE_OPEN,
  3=POWEROFF, 4=DISABLED, 5=ENABLE, 6=BACKDRIVE, 7=RUNNING, 8=SINGLE_STEP,
  9=ERROR, 10=PAUSE, 11=COLLISION).
- `/api/servo/config` POST filter and the settings-store subscribe hook
  had two independent whitelists; keys added to the POST filter but
  not the subscribe hook persisted but didn't reach the live tester.
  Both now share a single `_SERVO_CONFIG_KEYS` tuple.

## [0.0.1] - 2025-11-22

### Added
- **Interactive shell mode** as default behavior (no arguments starts shell)
  - Persistent robot connection throughout session
  - Command history with up/down arrow navigation
  - Emacs-style line editing (Ctrl+A, Ctrl+E, Ctrl+K, etc.)
  - Tab completion for commands
  - Auto-suggestions from command history
- **Jog commands** - Incremental robot movement
  - Single joint: `jog j1 5` (move J1 by 5°)
  - All joints: `jog joints 1 2 0 0 0 -1` (move all 6 joints)
  - Cartesian linear: `jog x 10`, `jog y -5`, `jog z 3` (mm)
  - Cartesian rotation: `jog rx 5`, `jog ry 10`, `jog rz -3` (degrees)
  - User coordinate mode (world/base axes)
  - Tool coordinate mode (relative to gripper orientation)
  - Mode switching: `jog mode user` or `jog mode tool`
  - Configurable default distances and speed
- **Dance command** - Random motion testing: `dance 5 10`
  - Random offsets from starting position (no drift)
  - Safety limits (max 20°, max 100 iterations)
  - Confirmation prompt in debug mode
  - Returns to start position at end
- **Debug mode** - `debug` command
  - Shows joint angles before/after moves
  - Confirmation prompt before executing moves
- **Background feedback polling**
  - Continuous thread polling robot feedback (matches SDK pattern)
  - Thread-safe caching ensures fresh position data
  - Fixes stale feedback data issues
- **Position reporting**
  - Both joint and cartesian space
  - Individual joint or cartesian views
  - Multiple output formats (table, JSON, YAML)
- **Robot control commands**
  - `enable` - Enable robot motors
  - `disable` - Disable robot motors
  - `clear` - Clear robot errors
  - `remote` - Request TCP/IP control
  - `status` - Show connection status
- **Dual mode operation**
  - Interactive shell (default)
  - One-shot commands for scripting
- **Configuration management**
  - YAML configuration files
  - Local override support (dobot_config.local.yaml)
  - IP, ports, speed, jog defaults
- **Professional project structure**
  - CLI framework with Click and Rich
  - Comprehensive documentation (README, CHANGELOG, QUICKSTART)
  - Example scripts
  - Installation script

### Fixed
- Socket closing errors on disconnect (graceful shutdown without error messages)
- Stale feedback data from socket buffer (implemented continuous polling)
- MovJ joint mode speed parameters (removed v/a params to match SDK examples)

[Unreleased]: https://github.com/yourusername/dobot-cr-controller/compare/v0.0.1...HEAD
[0.0.1]: https://github.com/yourusername/dobot-cr-controller/releases/tag/v0.0.1
