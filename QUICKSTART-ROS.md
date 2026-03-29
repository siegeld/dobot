# Dobot ROS2 Quickstart

## 1. Configure

Edit `.env` with your robot's IP:

```bash
cp .env.example .env
vim .env
```

```
ROBOT_IP=192.168.5.1
ROBOT_TYPE=cr5
```

## 2. Build (first time only)

```bash
docker compose build
```

This takes ~15 minutes the first time. Subsequent rebuilds are fast due to layer caching.
Code changes to `dobot-ros/` are picked up automatically via volume mount — no rebuild needed.

## 3. Start Driver

In Terminal 1:

```bash
./dobot-driver.sh
```

Keep this running. You should see ROS2 connection messages.

## 4. Use the CLI

In Terminal 2:

```bash
# Interactive shell (recommended)
./dobot-shell.sh

# Or one-shot commands
./dobot.sh position
./dobot.sh enable
./dobot.sh jog x 10
```

## Shell Commands

| Command | Description |
|---------|-------------|
| `position` | Show joint and cartesian position |
| `enable` | Enable robot |
| `disable` | Disable robot |
| `clear` | Clear errors |
| `stop` | Stop motion |
| `jog x 10` | Move X +10mm |
| `jog j1 5` | Rotate J1 +5° |
| `jog mode tool` | Switch to tool coordinates |
| `gripper init` | Initialize DH Robotics gripper |
| `gripper open` | Open gripper |
| `gripper close` | Close gripper |
| `gripper move 500` | Move to position (0=closed, 1000=open) |
| `gripper dance 5` | Random open/close dance (5 cycles) |
| `gripper status` | Show gripper state |
| `debug` | Toggle debug mode (confirms before moves) |
| `help` | Show all commands |
| `exit` | Exit shell |

## Troubleshooting

**"Service not available"** - Driver not running. Start `./dobot-driver.sh` first.

**Robot not moving** - Run `enable` then `clear` to enable and clear errors.

**Network issues** - Ensure your machine can ping the robot IP.

---

See [README-ROS.md](README-ROS.md) for full documentation.
