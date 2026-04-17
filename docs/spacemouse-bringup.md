# SpaceMouse Pendant — Bring-up Checklist

Run this the first time after install, after any Bluetooth pairing change,
and any time the puck feels "wrong."

See `docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md` for the
full design.

## One-time host setup

1. **Pair the SpaceMouse with Bluetooth** (once):
   ```
   bluetoothctl
   scan on
   # find: Device XX:XX:... SpaceMouse Wireless BT
   pair XX:XX:XX:XX:XX:XX
   trust XX:XX:XX:XX:XX:XX
   connect XX:XX:XX:XX:XX:XX
   quit
   ```
2. Verify: `bluetoothctl info <MAC>` shows `Connected: yes` and `Paired: yes`.
3. Confirm it appears under `/dev/input/`:
   ```
   for f in /sys/class/input/event*/device/name; do
       echo "$(basename $(dirname $(dirname $f))): $(cat $f)"
   done | grep -i space
   ```
   Expected: `event21: SpaceMouse` (number may differ).

## Container prep

4. Set `INPUT_GID` in `.env` to match the host's `input` group GID:
   ```
   getent group input | cut -d: -f3
   ```
   Fedora / Ubuntu default: `104`. Edit `.env`:
   ```
   INPUT_GID=104
   ```
5. Rebuild the image so `python3-evdev` is installed:
   ```
   docker compose build dobot
   ```
6. Start the stack: `./startup.sh`.
7. Verify discovery from inside the container:
   ```
   docker compose run --rm dobot python3 -c \
     "from dobot_ros.spacemouse.device import find_device; print(find_device())"
   ```
   Expected: prints `/dev/input/event21` (or whatever event node the puck
   currently occupies). If `None`, re-check `bluetoothctl info <MAC>` on
   the host — BT may have dropped.

## Pre-flight (robot disabled — no motion expected)

8. Open `http://localhost:7070/spacemouse`.
9. Device pill should be green (`connected`) and battery percentage visible.
10. Move the puck on each of the six axes; verify each bar responds in the
    expected direction. Press each button; verify the indicator dots glow.
    **No robot motion should occur — you are disarmed.**

## In-flight (robot enabled, armed)

11. Enable the robot from the main dashboard.
12. Click **Arm**. The page should flip to show an Armed pill, a Disarm
    button, and an Anchor pose. Still no motion.
13. Gently push **+X** (right). Robot should move +X in the base frame.
    Release — motion stops instantly.
14. Walk each axis: ±X, ±Y, ±Z, ±RX, ±RY, ±RZ. Any axis that moves the
    wrong way → open **Settings** → flip the sign toggle for that axis →
    **Save**. Live — no disarm required.
15. Push full-deflection +X and hold. Watch the **Offset from anchor**
    counter — it climbs to 300 mm and stops. Release.
16. Walk away for 30 s with the puck at rest. The page auto-disarms.
17. Re-arm. Press the **left** puck button (BTN 0). Gripper **closes**.
    Press the **right** puck button (BTN 1). Gripper **opens**.
18. Disarm. Press each puck button — gripper does **nothing** (safety).
19. Re-arm. Press `Esc` on the Pendant page. E-stop fires, robot halts,
    page shows "emergency stop".

## Failure drills

- Turn the puck **off** mid-jog → page should disarm immediately and show
  the device status pill as `lost`. Turn puck back on → pill goes green;
  re-arm is manual (never silently).
- `docker compose restart dobot-web` → page reconnects over the WebSocket;
  device is rediscovered.

## Known limits (v1)

- World-frame only. Tool-frame jog is a planned follow-up.
- No CLI; the web page is the only interface.
- One puck at a time.
