# Dobot CR Controller

[![Python Version](https://img.shields.io/badge/python-3.7%2B-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

A professional, production-ready CLI tool for controlling Dobot CR series collaborative robots (CR3, CR5, CR5A, CR7, CR10, CR16, etc.) via Python.

## Features

✨ **Dual Mode Operation** - Interactive shell OR one-shot commands
🎮 **Interactive Shell** - REPL with persistent connection, command history, and emacs editing
✨ **Professional CLI** - Built with Click framework, featuring beautiful Rich-formatted output
🎯 **Simple Configuration** - YAML-based config with sensible defaults
📊 **Position Reporting** - Real-time joint and cartesian space position monitoring
🔌 **Easy Integration** - Can be used as a library or standalone CLI tool
🚀 **Production Ready** - Proper packaging, error handling, and documentation
🎨 **Beautiful Output** - Colorful tables and formatted display using Rich
⚡ **Shell Completion** - Bash, Zsh, and Fish completion support

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/dobot-cr-controller.git
cd dobot-cr-controller

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install the package in development mode
pip install -e .
```

### Configuration

Create or edit `dobot_config.yaml`:

```yaml
robot:
  ip: "10.11.6.68"          # Your robot's IP address
  control_port: 29999       # Default control port
  feedback_port: 30004      # Default feedback port
  timeout: 5                # Connection timeout (seconds)

display:
  color: true               # Use colored output
  format: "table"           # Output format: table, json, yaml
  precision: 2              # Decimal precision for coordinates

logging:
  level: "INFO"             # Log level
```

For local overrides (gitignored), create `dobot_config.local.yaml`.

### Usage

The tool supports **two modes of operation**:

1. **Interactive Shell** - Persistent connection, multiple commands
2. **One-Shot Commands** - Execute single command and exit

#### Interactive Shell Mode (Default)

Start an interactive shell with persistent robot connection:

```bash
./dobot-cr.sh          # Starts shell by default!
# or explicitly:
./dobot-cr.sh shell
# or:
dobot-cr shell
```

Features:
- **Persistent connection** - Connect once, run many commands
- **Command history** - Use ↑/↓ arrows to recall previous commands
- **Emacs keybindings** - Ctrl+A (start), Ctrl+E (end), Ctrl+K (kill), etc.
- **Tab completion** - Press Tab to complete commands
- **Auto-suggestions** - See suggestions from history as you type

Available shell commands:
```
dobot> help                 # Show all commands
dobot> position             # Show both joint and cartesian
dobot> joint                # Joint space only
dobot> cartesian            # Cartesian space only
dobot> jog x 10             # Jog +10mm in X direction
dobot> jog z -5             # Jog -5mm in Z direction
dobot> jog rx 15            # Rotate +15° around X-axis
dobot> jog mode user        # Switch to user coordinates
dobot> jog mode tool        # Switch to tool coordinates
dobot> enable               # Enable robot
dobot> disable              # Disable robot
dobot> clear                # Clear errors
dobot> remote               # Request TCP/IP control (for jog/move)
dobot> status               # Connection status
dobot> exit                 # Exit shell (or Ctrl+D)
```

#### One-Shot Command Mode

Execute a single command and exit:

##### Test Connection

```bash
dobot-cr connect
# ℹ Connecting to robot at 10.11.6.68...
# ✓ Successfully connected to robot at 10.11.6.68
```

##### Get Robot Position

Show both joint and cartesian positions:

```bash
dobot-cr position
```

Output:
```
           Joint Space Position
┌───────┬─────────────┐
│ Joint │  Angle (°)  │
├───────┼─────────────┤
│  J1   │    45.23    │
│  J2   │   -30.45    │
│  J3   │    60.12    │
│  J4   │     0.00    │
│  J5   │    90.00    │
│  J6   │     0.00    │
└───────┴─────────────┘

        Cartesian Space Position
┌──────┬─────────────┐
│ Axis │    Value    │
├──────┼─────────────┤
│  X   │  245.60 mm  │
│  Y   │ -120.30 mm  │
│  Z   │  350.00 mm  │
│  RX  │    0.00°    │
│  RY  │   90.00°    │
│  RZ  │    0.00°    │
└──────┴─────────────┘
```

##### Joint Space Only

```bash
dobot-cr position --joint
```

##### Cartesian Space Only

```bash
dobot-cr position --cartesian
```

##### JSON Output

```bash
dobot-cr position --format json
```

```json
{
  "joint": {
    "J1": 45.23,
    "J2": -30.45,
    "J3": 60.12,
    "J4": 0.0,
    "J5": 90.0,
    "J6": 0.0
  },
  "cartesian": {
    "X": 245.6,
    "Y": -120.3,
    "Z": 350.0,
    "RX": 0.0,
    "RY": 90.0,
    "RZ": 0.0
  }
}
```

##### View Configuration

```bash
dobot-cr config-show
```

##### Jog Robot (Incremental Movement)

**Important:** Before jogging, request TCP/IP control:
```
dobot> remote               # Request TCP/IP control
```

Move the robot incrementally in cartesian space:

```bash
# One-shot jog commands
dobot-cr jog x 10           # Move +10mm in X
dobot-cr jog z -5           # Move -5mm in Z
dobot-cr jog rx 15          # Rotate +15° around X-axis

# With options
dobot-cr jog y 20 --mode tool    # Move in tool coordinates
dobot-cr jog z 5 --speed 30      # Slower movement (30%)
```

**Interactive shell jog** (recommended):
```
dobot> jog x 10             # Move +10mm in X (user coords)
dobot> jog z -5             # Move -5mm in Z
dobot> jog rx 15            # Rotate +15° around X-axis
dobot> jog mode tool        # Switch to tool coordinates
dobot> jog mode user        # Switch to user coordinates
dobot> jog y                # Use default distance (10mm)
dobot> status               # Shows current jog mode
```

**Coordinate modes:**
- **User** (default): Moves along world/base axes (X=forward/back, Y=left/right, Z=up/down)
- **Tool**: Moves relative to tool/gripper orientation

**Axes:**
- `x`, `y`, `z`: Linear movement in mm
- `rx`, `ry`, `rz`: Rotation in degrees

**Configuration** (dobot_config.yaml):
```yaml
jog:
  default_distance_mm: 10     # Default for x/y/z
  default_rotation_deg: 5     # Default for rx/ry/rz
  speed_percent: 50           # Movement speed (1-100)
  coordinate_mode: "user"     # Default mode
```

##### Shell Completion

Enable auto-completion for your shell:

```bash
# Bash
eval "$(_DOBOT_CR_COMPLETE=bash_source dobot-cr)"

# Zsh
eval "$(_DOBOT_CR_COMPLETE=zsh_source dobot-cr)"

# Fish
eval (env _DOBOT_CR_COMPLETE=fish_source dobot-cr)
```

## Using as a Library

```python
from dobot_cr import DobotController, Config

# Load configuration
config = Config()

# Create controller
with DobotController(
    ip=config.robot_ip,
    control_port=config.control_port,
    feedback_port=config.feedback_port
) as robot:
    # Get current position
    pos = robot.get_position()

    print(f"Joint angles: {pos.joint_dict}")
    print(f"Cartesian position: {pos.cartesian_dict}")

    # Access individual values
    print(f"X position: {pos.cartesian[0]} mm")
    print(f"J1 angle: {pos.joint[0]}°")
```

## ROS2 Version (Docker)

A ROS2-based version runs the driver in Docker and provides additional features like action servers and gripper control. See [README-ROS.md](README-ROS.md) for full details.

### Quick Start

```bash
# Configure robot IP
cp .env.example .env
vim .env                    # Set ROBOT_IP and ROBOT_TYPE

# Build (first time only, ~15 min)
docker compose build

# Start the system (driver + web dashboard)
./startup.sh
```

The driver auto-connects when the robot powers on and auto-reconnects if the robot reboots. The web dashboard is available at http://localhost:7070 and works independently — it shows a disconnected state until the driver connects.

```bash
# CLI commands (while system is running)
./dobot-shell.sh            # Interactive shell
./dobot.sh position         # One-shot command
./dobot.sh jog x 10         # Jog +10mm in X

# Stop everything
./startup.sh --stop
```

Code changes to `dobot-ros/` take effect immediately (volume-mounted, no rebuild).

### ROS2 Shell Commands

| Command | Description |
|---------|-------------|
| `position` | Show joint and cartesian position |
| `enable` / `disable` | Enable/disable robot |
| `clear` | Clear errors |
| `stop` | Stop motion |
| `jog x 10` | Move X +10mm |
| `jog j1 5` | Rotate J1 +5° |
| `jog mode tool` | Switch to tool coordinates |
| `gripper init` | Initialize DH Robotics gripper |
| `gripper open` / `close` | Open/close gripper |
| `gripper move 500` | Move gripper (0=closed, 1000=open) |
| `gripper dance 5` | Random open/close dance (5 cycles) |
| `gripper status` | Show gripper state |
| `dance 5 10` | Random arm dance (10 moves, ±5°) |
| `debug` | Toggle debug mode |
| `sync` | Toggle sync/async motion |

### ROS2 One-Shot Commands

```bash
./dobot.sh connect                      # Test connection
./dobot.sh position --format json       # Position as JSON
./dobot.sh jog z -5 --mode tool         # Jog in tool frame
./dobot.sh enable                       # Enable robot
./dobot.sh gripper dance 10 -s 100      # Fast gripper dance
```

---

## Project Structure

```
dobot/
├── dobot_cr/              # Direct TCP/IP controller (native)
│   ├── cli.py             # Click-based CLI
│   ├── config.py          # Configuration management
│   ├── robot.py           # Robot controller wrapper
│   └── shell.py           # Interactive shell
├── dobot-ros/             # ROS2 CLI (runs in Docker)
│   └── dobot_ros/
│       ├── cli.py         # Click-based CLI
│       ├── config.py      # Configuration
│       ├── ros_client.py  # ROS2 service client + gripper API
│       └── shell.py       # Interactive shell
├── dobot_actions/         # ROS2 action servers
│   ├── action/            # Action definitions (.action files)
│   └── scripts/           # Action server nodes
├── DOBOT_6Axis_ROS2_V4/   # Vendor ROS2 driver packages
├── TCP-IP-Python-V4/      # Vendor TCP/IP SDK
├── docker/                # Dockerfile, entrypoint, FastRTPS config
├── docker-compose.yml     # Docker services
├── startup.sh             # Start system (driver + web dashboard)
├── dobot-driver.sh        # Start ROS2 driver only
├── dobot-shell.sh         # Start ROS2 interactive shell
├── dobot.sh               # Run ROS2 CLI commands
├── dobot-cr.sh            # Run direct TCP/IP CLI
└── .env                   # Robot IP and type (not committed)
```

## Development

### Setup Development Environment

```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run tests (when available)
pytest

# Format code
black dobot_cr/

# Lint code
flake8 dobot_cr/

# Type checking
mypy dobot_cr/
```

### Building for Distribution

```bash
# Build wheel and source distribution
python -m build

# Install from built wheel
pip install dist/dobot_cr_controller-0.1.0-py3-none-any.whl
```

## Requirements

- Python 3.7 or higher
- Network access to Dobot CR robot (robot must be in TCP/IP mode)
- Robot firmware version 4.4.0.0 or higher

## Supported Robots

- Dobot CR3
- Dobot CR5
- **Dobot CR5A** ✓ Tested
- Dobot CR7
- Dobot CR10
- Dobot CR16
- Other CR series robots with TCP/IP V4 protocol

## Network Configuration

Ensure your computer and robot are on the same network:

1. Robot IP must be accessible from your computer
2. Ports 29999 (control) and 30004 (feedback) must be open
3. Robot must be in TCP/IP control mode (not emergency stop)

## Troubleshooting

### Connection Timeout

```bash
Error: Failed to connect to robot at 10.11.6.68: [Errno 110] Connection timed out
```

**Solutions:**
- Verify robot IP address in `dobot_config.yaml`
- Check network connectivity: `ping 10.11.6.68`
- Ensure robot is powered on and in TCP/IP mode
- Check firewall settings

### Configuration File Not Found

```bash
Error: No configuration file found. Expected 'dobot_config.yaml'
```

**Solution:** Create `dobot_config.yaml` in the directory where you run the command.

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please ensure:
- Code follows Black formatting
- All tests pass
- Documentation is updated

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built on top of [Dobot TCP-IP-Python-V4](https://github.com/Dobot-Arm/TCP-IP-Python-V4) SDK
- Uses [Click](https://click.palletsprojects.com/) for CLI framework
- Uses [Rich](https://rich.readthedocs.io/) for beautiful terminal output

## Support

- 📖 [Documentation](https://github.com/siegeld/dobot#readme)
- 🐛 [Issue Tracker](https://github.com/siegeld/dobot/issues)
- 💬 [Discussions](https://github.com/siegeld/dobot/discussions)

## Roadmap

- [ ] Add movement commands (MovJ, MovL, Arc)
- [x] Add gripper control (DH Robotics via ROS2)
- [ ] Add digital I/O control
- [ ] Add trajectory recording/playback
- [ ] Add safety monitoring
- [x] Add web-based dashboard (http://localhost:7070)
- [x] Add ROS 2 integration

---

Made with ❤️ for the robotics community
