# Dobot CR Controller - ROS2 Version

This is the ROS2-based version of the Dobot CR CLI. Instead of communicating directly with the robot via TCP/IP, it uses ROS2 services provided by the `DOBOT_6Axis_ROS2_V4` driver.

## Architecture

```
┌─────────────────┐     ROS2 Services      ┌──────────────────────┐     TCP/IP     ┌─────────┐
│  dobot-ros CLI  │ ◄──────────────────────► │  cr_robot_ros2 node  │ ◄─────────────► │  Robot  │
│  (Python)       │   /dobot_bringup_ros2/  │  (C++ driver)        │   Port 29999   │         │
└─────────────────┘          srv/*          └──────────────────────┘                └─────────┘
```

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

Supports DH Robotics grippers (AG-95, PGE, RGD, PGC series) connected to the Dobot tool flange via RS485.

### How it Works

The gripper communicates through the Dobot controller's **internal Modbus TCP gateway** at `127.0.0.1:60000`. This gateway is configured by the DHGrip Dobot Studio plugin.

> **Note:** A robot power cycle resets this gateway. If gripper commands return -1, re-run the DHGrip plugin on the teach pendant (install → enable → uninstall) to restore port 60000.

```
CLI / Action Server
        │
        ▼  ROS2 Services
  ModbusCreate(127.0.0.1, 60000, slave_id=1)
  SetHoldRegs / GetHoldRegs
        │
        ▼  Dobot Controller Internal Gateway
  127.0.0.1:60000 → Tool RS485 → Gripper
```

### Register Map

| Register | Address | Read/Write | Range | Description |
|----------|---------|------------|-------|-------------|
| Init | 0x0100 (256) | Write | 0xA5 (165) | Full initialization |
| Force | 0x0101 (257) | R/W | 20-100 % | Grip force |
| Position | 0x0103 (259) | R/W | 0-1000 | 0=closed, 1000=open |
| Speed | 0x0104 (260) | R/W | 1-100 % | Movement speed |
| Init Status | 0x0200 (512) | Read | 0/1 | 0=not init, 1=initialized |
| Grip State | 0x0201 (513) | Read | 0-3 | 0=moving, 1=reached, 2=caught, 3=dropped |
| Current Pos | 0x0202 (514) | Read | 0-1000 | Actual position |

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
