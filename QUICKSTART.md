# Quick Start Guide

## Two Ways to Use

**Interactive Shell** (Default) - Persistent connection, multiple commands:
```bash
./dobot-cr.sh          # Starts shell by default!
```

**One-Shot Commands** - Execute single command and exit:
```bash
./dobot-cr.sh position
```

## Get Started in 3 Steps

### 1. Run the installer

```bash
bash scripts/install.sh
```

This will:
- Create a virtual environment
- Install all dependencies
- Install the `dobot-cr` command

### 2. Activate the virtual environment

```bash
source venv/bin/activate
```

Or use the convenience wrapper (no activation needed):

```bash
./dobot-cr.sh --help
```

### 3. Test your robot connection

```bash
dobot-cr connect
# or
./dobot-cr.sh connect
```

## Interactive Shell (Default Behavior)

Start the interactive shell for the best experience:

```bash
./dobot-cr.sh          # No command needed - starts shell by default!
```

Then use commands directly:
```
dobot> help                 # Show all commands
dobot> position             # Get current position
dobot> joint                # Joint angles only
dobot> cartesian            # Cartesian coordinates only
dobot> jog x 10             # Jog +10mm in X
dobot> jog z -5             # Jog -5mm in Z
dobot> jog mode tool        # Switch coordinate mode
dobot> enable               # Enable robot
dobot> disable              # Disable robot
dobot> clear                # Clear errors
dobot> status               # Connection info
dobot> exit                 # Exit (or Ctrl+D)
```

### Shell Features
- ↑/↓ arrows for command history
- Tab for command completion
- Emacs keybindings (Ctrl+A, Ctrl+E, Ctrl+K, etc.)
- Auto-suggestions from history
- Persistent robot connection

## One-Shot Commands

### View Configuration

```bash
./dobot-cr.sh config-show
```

### Get Robot Position

```bash
# Both joint and cartesian
./dobot-cr.sh position

# Joint space only
./dobot-cr.sh position --joint

# Cartesian space only
./dobot-cr.sh position --cartesian

# JSON output
./dobot-cr.sh position --format json
```

### Jog Robot (Move Incrementally)

**First, request TCP/IP control:**
```
dobot> remote                   # Request TCP/IP control
```

Then jog:
```bash
# One-shot commands
./dobot-cr.sh jog x 10          # Move +10mm in X
./dobot-cr.sh jog z -5          # Move -5mm in Z
./dobot-cr.sh jog rx 15         # Rotate +15° around X

# In interactive shell (recommended)
dobot> jog x 10                 # Move +10mm in X
dobot> jog y -20                # Move -20mm in Y
dobot> jog mode tool            # Switch to tool coordinates
dobot> jog z 5                  # Move along tool's Z-axis
dobot> jog mode user            # Switch back to world coordinates
```

**Tip:** Use default distances by omitting the number:
```
dobot> jog x                    # Uses default (10mm)
```

### Get Help

```bash
./dobot-cr.sh --help
./dobot-cr.sh position --help
```

## Configuration

Edit `dobot_config.yaml` to change settings:

```yaml
robot:
  ip: "10.11.6.69"      # Your robot's IP
  control_port: 29999
  feedback_port: 30004
  timeout: 5

display:
  format: "table"       # table, json, or yaml
  precision: 2          # Decimal places
```

For local overrides (not tracked in git), create `dobot_config.local.yaml`.

## Using as a Python Library

```python
from dobot_cr import DobotController, Config

config = Config()

with DobotController(ip=config.robot_ip) as robot:
    pos = robot.get_position()
    print(f"X: {pos.cartesian[0]} mm")
    print(f"J1: {pos.joint[0]}°")
```

See `examples/basic_usage.py` and `examples/advanced_control.py` for more.

## Troubleshooting

### Connection Issues

1. Verify robot IP: `ping 10.11.6.69`
2. Check robot is powered and in TCP/IP mode
3. Ensure ports 29999 and 30004 are accessible

### Import Errors

Activate the virtual environment:
```bash
source venv/bin/activate
```

Or use the wrapper script:
```bash
./dobot-cr.sh <command>
```

## Next Steps

- Read the full [README.md](README.md)
- Check [CHANGELOG.md](CHANGELOG.md) for version history
- View [examples/](examples/) for code samples
- Contribute on GitHub!
