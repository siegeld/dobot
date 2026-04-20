# Dobot CR Controller - ROS2 Version

This is the ROS2-based version of the Dobot CR CLI. Instead of communicating directly with the robot via TCP/IP, it uses ROS2 services provided by the `DOBOT_6Axis_ROS2_V4` driver.

> **Related docs**
> - [ARCHITECTURE.md](ARCHITECTURE.md) — system diagram, containers, control flow, safety boundaries, file map.
> - [API.md](API.md) — HTTP + WebSocket reference for the web dashboard.
> - [CHANGELOG.md](CHANGELOG.md) — user-facing change history.
> - [VLA.md](VLA.md) — Vision-Language-Action stack design.
> - [VISION.md](VISION.md) — camera → robot geometry and pick strategies.

## Architecture

Short version: a Docker compose stack runs three services (`dobot-driver`, `dobot-gripper`, `dobot-web`) on host networking. The driver container hosts the vendor C++ ROS node, which maintains TCP sessions to the robot (29999 command, 30004 feedback). The web container runs the FastAPI dashboard, a ServoP streaming tester, the SpaceMouse pendant reader, and the VLA executor — all talking to the driver via ROS 2 services + topics. The gripper node owns the single Modbus TCP connection through the robot's internal RS-485 gateway (port 60000).

```
┌─────────────────┐     ROS2 Services      ┌──────────────────────┐     TCP/IP     ┌─────────┐
│  dobot-ros CLI  │ ◄──────────────────────► │  cr_robot_ros2 node  │ ◄─────────────► │  Robot  │
│  (Python)       │   /dobot_bringup_ros2/  │  (C++ driver)        │   Port 29999   │         │
└─────────────────┘          srv/*          └──────────────────────┘                └─────────┘
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for the full diagram including nginx, the gripper node, watchdog signaling, and the ServoTester's two input modes.

**Benefits of ROS2 approach:**
- Integration with ROS2 ecosystem (MoveIt, RViz, Gazebo)
- Multiple clients can share one robot connection
- Standard ROS2 tooling for debugging and visualization
- Topic-based real-time feedback

## Quick Start with Docker (Recommended)

Docker is the easiest way to run the ROS2 stack on any Linux system (Fedora, Ubuntu, etc.).

### Prerequisites

- Docker and Docker Compose installed
- Network access to your Dobot robot

### Build the Docker Image

```bash
cd /path/to/dobot

# Build (first time takes ~15 minutes for ROS2 message generation)
docker compose build
```

Code changes to `dobot-ros/` are volume-mounted into the container and take effect
immediately — no rebuild needed. Only rebuild when changing `dobot_actions/`,
`DOBOT_6Axis_ROS2_V4/`, or the Dockerfile itself.

### Configuration

Edit `.env` to set your robot's IP and type:

```bash
# .env file
ROBOT_IP=192.168.5.1
ROBOT_TYPE=cr5  # cr3, cr5, cr7, cr10, cr12, cr16, cr20
```

### Start the System

```bash
./startup.sh
```

This starts both the ROS2 driver and the web dashboard in Docker. Both services use `restart: unless-stopped` — the driver auto-connects when the robot powers on and auto-reconnects if the robot reboots. The web dashboard at http://localhost:7070 runs independently and shows a disconnected state until the driver connects.

```bash
./startup.sh --build    # Rebuild image first, then start
./startup.sh --stop     # Stop everything
```

### Web Dashboard Highlights

The dashboard at `http://localhost:7070` is the main interface. A few
features worth knowing about upfront — the full list lives in
[CHANGELOG.md](CHANGELOG.md):

- **Tabs** for Dashboard, Calibration, Vision, VLA, Servo, Settings;
  persistent sidebar with a 3D robot view, robot control, gripper, and
  joint/cartesian position.
- **Tool-frame selector** in the Robot Control card — switch between
  Tool 0 (wrist / flange) and Tool 1 (fingertip, 203 mm AG-105 offset).
  The ServoTester floor guard recomputes its Z minimum for the active
  tool so the fingertip can't drive into the table in either mode.
- **"Vertical"** button — one-click orient the tool so its Z axis points
  straight down (RX=180, RY=0, RZ preserved).
- **"Lock"** toggle — keeps the tool perpendicular during SpaceMouse
  jogging by re-projecting every ServoP target to RX=180 / RY=0. Pair
  with Tool 1 for the pick workflow: fingertip XY/Z tracks the puck,
  gripper stays perpendicular to the table, yaw (RZ) stays free.
- **SpaceMouse pendant** at `/spacemouse` — rate-controlled jogging
  (release = stop), IIR-smoothed axes, configurable deadband / sign map
  / velocity caps, button-to-gripper bindings, idle auto-disarm.
- **Servo tester** at the Servo tab — ServoP streaming bench with live
  tuning, hard velocity caps, pattern generators, CSV logging.
- **Calibration & Vision** tabs — table-plane recording, camera→robot
  transform, vision-guided picking with pluggable strategies.
- **VLA** tab — recorder + executor for OpenVLA-OFT closed-loop
  inference over the ServoTester streaming path.
- **Settings** tab — save/load named snapshots of all persistent state.

### Use the CLI

In another terminal:

```bash
# Start interactive shell
./dobot-shell.sh

# Or use the general wrapper
./dobot.sh              # Interactive shell
./dobot.sh position     # Get robot position
./dobot.sh jog x 10     # Jog X by 10mm
./dobot.sh enable       # Enable robot
```

### Usage with Docker Compose (Manual)

```bash
# Start driver
docker compose up dobot-driver

# Check connection
docker compose run --rm dobot dobot-ros connect

# Get robot position
docker compose run --rm dobot dobot-ros position

# Start interactive shell
docker compose run --rm dobot dobot-ros shell

# Or just run dobot-ros (starts shell by default)
docker compose run --rm dobot dobot-ros
```

**Interactive Bash Session**

```bash
# Full bash shell with ROS2 environment
docker compose run --rm dobot bash

# Inside container:
ros2 service list | grep dobot
dobot-ros position
```

### Docker Commands Reference

| Command | Description |
|---------|-------------|
| `docker compose build dobot` | Build the image |
| `./startup.sh` | Start driver + web dashboard |
| `./startup.sh --stop` | Stop all services |
| `docker compose up -d dobot-driver` | Start driver only (background) |
| `docker compose run --rm dobot dobot-ros` | Run CLI |
| `docker compose run --rm dobot bash` | Interactive shell |
| `docker compose down` | Stop all services |
| `docker compose logs dobot-driver` | View driver logs |

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `192.168.5.1` | Robot IP address |
| `ROBOT_TYPE` | `cr5` | Robot model |

---

## Native Installation (Ubuntu 24.04 only)

If you're on Ubuntu 24.04, you can install ROS2 Jazzy natively.

### Install ROS2 Jazzy

```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build the Dobot ROS2 Workspace

```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src

# Copy Dobot packages
cp -r /path/to/dobot/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4 .
cp -r /path/to/dobot/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4 .

# Build
cd ~/dobot_ws
colcon build --symlink-install

echo "source ~/dobot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install the dobot-ros CLI

```bash
cd /path/to/dobot/dobot-ros
pip install .
```

### Launch

```bash
# Terminal 1: Driver
export IP_address=192.168.5.1
export DOBOT_TYPE=cr5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

# Terminal 2: CLI
dobot-ros connect
dobot-ros shell
```

---

## CLI Commands

### One-shot Commands

```bash
dobot-ros connect              # Test connection to ROS2 services
dobot-ros position             # Show current position
dobot-ros position --joint     # Joint angles only
dobot-ros position --cartesian # Cartesian pose only
dobot-ros position --format json

dobot-ros enable               # Enable robot
dobot-ros disable              # Disable robot
dobot-ros clear                # Clear errors
dobot-ros stop                 # Stop motion

dobot-ros jog x 10             # Move +10mm in X
dobot-ros jog z -5             # Move -5mm in Z
dobot-ros jog j1 5             # Rotate J1 by +5°
dobot-ros jog rz 10 --mode tool  # Rotate in tool frame

dobot-ros config-show          # Show configuration

# Gripper control (DH Robotics AG-95 / PGE / RGD)
dobot-ros gripper init         # Initialize gripper
dobot-ros gripper open         # Open gripper
dobot-ros gripper close        # Close gripper
dobot-ros gripper move 500     # Move to position (0=closed, 1000=open)
dobot-ros gripper close -s 80 -f 100  # Close at speed=80 force=100
dobot-ros gripper status       # Show gripper position and state
dobot-ros gripper dance 5      # Random open/close dance (5 cycles)
dobot-ros gripper dance 10 -s 100  # Fast dance, 10 cycles
```

### Interactive Shell

```bash
dobot-ros shell
# Or just:
dobot-ros
```

Shell commands:

| Command | Aliases | Description |
|---------|---------|-------------|
| `position` | `pos` | Show both joint and cartesian position |
| `joint` | | Show joint position only |
| `cartesian` | `cart` | Show cartesian position only |
| `jog x/y/z [mm]` | | Jog in cartesian space |
| `jog rx/ry/rz [deg]` | | Rotate in cartesian space |
| `jog j1-j6 [deg]` | | Jog single joint |
| `jog joints <j1..j6>` | | Jog all joints at once |
| `jog mode <user\|tool>` | | Switch coordinate frame |
| `enable` | | Enable the robot |
| `disable` | | Disable the robot |
| `clear` | | Clear robot errors |
| `stop` | | Stop robot motion |
| `gripper init` | `grip init` | Initialize gripper |
| `gripper open [spd] [frc]` | | Open gripper |
| `gripper close [spd] [frc]` | | Close gripper |
| `gripper move <pos> [spd] [frc]` | | Move gripper (0=closed, 1000=open) |
| `gripper dance <cycles> [spd] [frc]` | | Random open/close dance |
| `gripper status` | | Show gripper position and state |
| `status` | | Show connection status |
| `debug` | | Toggle debug mode |
| `dance <deg> <count>` | | Random dance motion |
| `help` | `?` | Show help |
| `exit` | `quit` | Exit shell |

---

## Gripper Control (DH Robotics)

Supports DH Robotics grippers connected to the Dobot tool flange via RS485. Currently configured for the **AG-95** adaptive gripper.

### How it Works

The gripper connects to the robot's **tool flange via RS-485**. The Dobot controller has an internal **Modbus TCP gateway** at `127.0.0.1:60000` that bridges TCP to the RS-485 bus, allowing software to communicate with the gripper using standard Modbus registers.

The gateway is created by calling `SetTool485(115200)`, which configures the tool RS-485 baud rate and enables the TCP-to-RS485 bridge. The `gripper_node.py` handles this automatically on startup. No DHGrip Dobot Studio plugin needed (though the node's Modbus pattern was derived from the working DHGrip Lua plugin in `DHGrip_v2-1-4-stable/`).

```
gripper_node.py (ROS2 node)
      │
      ├── SetTool485(115200)          ← Configures tool RS-485, creates gateway
      │
      ├── ModbusCreate(127.0.0.1, 60000, slave_id=1, RTU)
      │         │
      │         ▼  Dobot Internal Gateway
      │    127.0.0.1:60000 → Tool RS-485 → Gripper
      │
      ├── SetHoldRegs / GetHoldRegs   ← Read/write gripper registers
      │
      ├── ModbusClose                 ← Closed and reopened for write operations
      │
      └── Publishes /gripper/state at 5Hz (position, grip state, init status)
```

**Connection management:**
- State polling (5Hz) uses a persistent Modbus connection for low overhead
- Write operations (move, init) reconnect first (close → SetTool485 → create) to ensure a clean connection, matching the DHGrip plugin's `NewConnection` pattern
- All operations are serialized via a mutex (equivalent to the plugin's `Lock485`/`UnLock485`)

The gripper node exposes the gripper to the rest of the system via:
- **Topic** `/gripper/state` — JSON state at 5Hz (position, grip state, init status)
- **Service** `/gripper/init` — trigger gripper initialization
- **Action** `/gripper` — move gripper to a position with feedback

**Web dashboard:** Gripper commands (open/close/move) are fire-and-forget — the HTTP API returns immediately after sending the action goal. Position and state updates flow to the browser in real time via the `/gripper/state` topic → WebSocket. The gripper position slider updates live as the gripper moves.

### Register Map

| Register | Address | Read/Write | Range | Description |
|----------|---------|------------|-------|-------------|
| Init | 0x0100 (256) | Write | 0xA5 (165) | Full initialization |
| Force | 0x0101 (257) | R/W | 20-100 % | Grip force (also controls speed on AG-95) |
| Reserved | 0x0102 (258) | - | - | - |
| Position | 0x0103 (259) | R/W | 0-1000 | 0=closed, 1000=open |
| Speed | 0x0104 (260) | R/W | 1-100 % | PGC/PGE only (AG-95 ignores) |
| Init Status | 0x0200 (512) | Read | 0/1 | 0=not init, 1=initialized |
| Grip State | 0x0201 (513) | Read | 0-3 | 0=moving, 1=reached, 2=caught, 3=dropped |
| Current Pos | 0x0202 (514) | Read | 0-1000 | Actual position |

> **Note:** The AG-95 has no independent speed control — speed is determined by force (higher force = faster). The speed register (0x0104) is written but ignored by the AG-95. The code retains speed support for future PGC/PGE grippers.

### ROS2 Action Server

The `gripper_server` node provides a `gripper` action for programmatic control:

```bash
# Launch with other action servers
ros2 launch dobot_actions actions.launch.py

# Send goals
ros2 action send_goal /gripper dobot_actions/action/Gripper \
  "{position: 0, speed: 50, force: 80}" --feedback

# Open
ros2 action send_goal /gripper dobot_actions/action/Gripper \
  "{position: 1000, speed: 100, force: 50}"
```

### CLI Usage

```bash
# One-shot commands
dobot-ros gripper init                  # Initialize (moves to find limits)
dobot-ros gripper close                 # Close fully
dobot-ros gripper open                  # Open fully
dobot-ros gripper move 500              # Move to 50% open
dobot-ros gripper close -s 80 -f 100   # Close fast with max force
dobot-ros gripper status                # Read position and state
dobot-ros gripper dance 5              # Random open/close dance (5 cycles)

# Interactive shell
dobot-ros shell
dobot-ros> gripper init
dobot-ros> gripper close 30 80         # speed=30, force=80
dobot-ros> gripper open
dobot-ros> gripper dance 5             # Random open/close, 5 cycles
dobot-ros> gripper status
```

### Default Communication Parameters

| Parameter | Value |
|-----------|-------|
| Slave ID | 1 |
| Baud Rate | 115200 |
| Parity | None |
| Data Bits | 8 |
| Stop Bits | 1 |

These are configured in the gripper itself (via DH-Robotics UI software). The Dobot controller's internal gateway handles the RS485 communication transparently.

---

## Configuration

Create `dobot_ros_config.yaml` in the current directory or `~/.config/dobot/`:

```yaml
# ROS2 namespace (if using multiple robots)
ros_namespace: ""

# Service call timeout in seconds
service_timeout: 5.0

# Output format: table, json, or yaml
output_format: table

# Decimal precision for position display
precision: 3

# Default jog distances
jog_default_distance: 10.0   # mm for X/Y/Z
jog_default_rotation: 5.0    # degrees for RX/RY/RZ and joints

# Jog speed (1-100%)
jog_speed: 10

# Jog coordinate mode: user or tool
jog_coordinate_mode: user
```

---

## Network Setup

The robot has default IPs:
- **Wired connection:** `192.168.5.1`
- **Wireless connection:** `192.168.1.6`

Configure your computer to be on the same subnet:

```bash
# Check current IP
ip addr show

# Add IP on same subnet (example for wired)
sudo ip addr add 192.168.5.10/24 dev eth0

# Test connectivity
ping 192.168.5.1
```

---

## Troubleshooting

### Joint angles not updating (stale feedback)

The driver connects to the robot on two ports: 29999 (commands) and 30004 (realtime feedback). If the robot reboots while the driver is running, the feedback port can go stale — joint angles stop updating even though commands still work.

The web dashboard automatically detects this (monitors message timestamps) and signals the driver to restart its ROS node. The driver's wrapper loop relaunches it within seconds, and joint data starts flowing again. No container restart needed.

If you need to trigger a manual restart, create the signal file:
```bash
docker compose exec dobot-driver touch /tmp/dobot-shared/driver_restart
```

### "Service not available"

The ROS2 driver is not running.

**Docker:**
```bash
./startup.sh              # Start driver + web dashboard
# or driver only:
docker compose up dobot-driver
```

**Native:**
```bash
export IP_address=192.168.5.1
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

### "Could not parse angle response"

The robot may not be responding. Check:
1. Robot is powered on
2. Network connectivity: `ping <robot_ip>`
3. Driver logs for errors

### Robot not moving after jog command

1. Enable the robot first: `dobot-ros enable`
2. Check for errors: `dobot-ros connect`
3. Clear errors if needed: `dobot-ros clear`

### Service timeout

Increase timeout in config:
```yaml
service_timeout: 10.0
```

### Docker network issues

The Docker setup uses `network_mode: host` to access the robot on your local network. Ensure:
1. Your host machine can ping the robot
2. No firewall blocking port 29999

---

## ROS2 Services Reference

The driver provides 94 services at `/dobot_bringup_ros2/srv/<name>`.

Key services:

| Service | Description |
|---------|-------------|
| `EnableRobot` | Enable robot motors |
| `DisableRobot` | Disable robot motors |
| `ClearError` | Clear error state |
| `Stop` | Stop current motion |
| `GetAngle` | Get joint angles |
| `GetPose` | Get cartesian pose |
| `MovJ` | Joint space motion |
| `MovL` | Linear motion |
| `RelMovLUser` | Relative move in user frame |
| `RelMovLTool` | Relative move in tool frame |
| `SpeedFactor` | Set speed (1-100%) |
| `RobotMode` | Get robot mode |

List all services:
```bash
# Docker
docker compose run --rm dobot ros2 service list | grep dobot_bringup_ros2

# Native
ros2 service list | grep dobot_bringup_ros2
```

---

## Differences from Direct TCP/IP Version (dobot-cr)

| Feature | dobot-cr (TCP/IP) | dobot-ros (ROS2) |
|---------|-------------------|------------------|
| Connection | Direct socket to robot | Via ROS2 driver node |
| Setup | Just robot IP | Docker or ROS2 workspace |
| `--ip` option | Per-command IP override | Not needed |
| `remote` command | Required for jog control | Not needed |
| Real-time feedback | Background thread polling | ROS2 topics |
| Multiple clients | One connection per client | Shared via ROS2 |

---

## Multiple Robots

Launch each robot with a unique node name:

**Docker:**
```bash
# Robot 1
ROBOT_IP=192.168.5.1 docker compose up dobot-driver

# Robot 2 (in another terminal, modify docker-compose.yml or use different project)
ROBOT_IP=192.168.5.2 docker compose -p robot2 up dobot-driver
```

**Native:**
```bash
# Terminal 1
export IP_address=192.168.5.1
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py robot_node_name:=robot1

# Terminal 2
export IP_address=192.168.5.2
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py robot_node_name:=robot2
```

Configure the CLI namespace in `dobot_ros_config.yaml`:
```yaml
ros_namespace: robot1
```
