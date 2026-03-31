# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Gripper auto-setup** — gripper_node.py now calls `SetTool485` on startup to configure the tool RS-485 interface and enable the Modbus TCP gateway on port 60000. The gripper works automatically after robot power-on without needing the DHGrip Dobot Studio plugin.
- **Web dashboard** at http://localhost:7070 — browser-based robot control with real-time WebSocket state updates (5Hz)
- **`startup.sh`** — single command to bring up driver + web dashboard
  - `./startup.sh` — start system
  - `./startup.sh --build` — rebuild image first
  - `./startup.sh --stop` — stop everything
- **Auto-reconnect** — driver and web services use `restart: unless-stopped`; driver auto-connects when robot powers on and recovers from reboots
- Web dashboard runs independently of driver, showing disconnected state until robot is available

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
