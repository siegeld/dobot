# SpaceMouse Pendant — Design

**Status:** draft for review
**Date:** 2026-04-17
**Author:** David Siegel (+ Claude)

## Summary

Add support for a Bluetooth-paired 3Dconnexion **SpaceMouse Wireless BT**
(vendor `0x256F`, product `0xC63A`) as a jogging pendant for the Dobot CR5.
The puck emits continuous 6-DOF deflections; we treat them as commanded
Cartesian velocity (rate control) and stream targets to the robot through the
existing `ServoTester` / `ServoP` pipeline. Gripper open/close is mapped to
the two physical buttons on the puck.

All changes are **additive**. The existing jog, MovJ/MovL, pick, calibrate,
and VLA paths are untouched.

## Goals

- Let an operator jog the arm in Cartesian space by pushing the puck.
- Open and close the gripper from the puck's two buttons.
- Ship the whole thing inside the existing Docker compose stack so
  installation is "`./startup.sh`" — no host-side daemons.
- Provide a web UI that doubles as a diagnostic test page (live HID echo)
  for bring-up and for verifying the puck after a BT reconnect.

## Non-goals (v1)

- Tool-frame jogging (world-frame only — additive upgrade later).
- CLI / shell integration for the pendant (web UI covers test + operate).
- Soft Open/Close buttons in the web UI (puck buttons only, by request).
- Joint-space jogging from the puck (Cartesian only).
- Custom axis-permutation remapping (sign flips per axis only).

## Prior context

The `ServoTester` in `dobot_ros/servo/tester.py` already implements the core
of what a pendant needs:

- Daemon thread streaming `ServoP` at ~30 Hz.
- Live input: `set_target_offset([dx,dy,dz,drx,dry,drz])` — an absolute
  displacement from a captured anchor pose.
- Workspace clamp via `vla/safety.py`.
- Idle auto-stop (`idle_timeout_s`).
- Emergency stop that halts the stream and calls `ros.stop()`.
- Web endpoints `/api/servo/{start,stop,estop,target}`.

The SpaceMouse reader will feed `set_target_offset` directly — no changes
to `ServoTester`, no changes to `ros_client.py`.

## Architecture

New module `dobot-ros/dobot_ros/spacemouse/` sibling of `servo/`, `vla/`:

- `reader.py` — `SpaceMouseReader` (evdev reader + integration loop).
- `hid_state.py` — `HidState` dataclass (6 axes, 2 buttons, battery,
  armed/idle, device status, last_event_age_ms).
- `__init__.py`.

The reader runs as two daemon threads inside the existing web dashboard
process (same process that hosts `ServoTester`):

1. **evdev read thread** — blocks on `device.read_loop()`; decodes each
   event and stores the latest normalized axis values and button edges
   under a lock. Handles device disconnect via `OSError` → sets
   `device_status = "lost"` and wakes the tick thread.
2. **50 Hz integration tick thread** — every 20 ms, reads the latest
   axis values, applies deadband + velocity scale, integrates
   `target_offset += v * dt`, clamps to `max_excursion_*`, and
   (if armed) calls `ServoTester.set_target_offset(...)`. Also polls
   the idle timer and button debounce state.

The tick thread exists separately from the evdev thread so that *holding
the puck still with no events* still advances the integration (actually
holds at the current offset — because `v = 0` when axes rest in
deadband). Equally important, when the operator holds the puck fully
deflected, the thread keeps producing fresh ServoP targets even though
evdev emits no new events (axes are static).

### Data flow

```
[/dev/input/event* (uhid, BT HID, 0x256F:0xC63A)]
        │
        ▼
[evdev read thread]              ← decodes events, updates latest axis/button state
        │
        ▼
[50 Hz integration tick thread]  ← deadband + scale + integrate + clamp
        │
        │ (if armed)                      (on button press edge)
        ▼                                 ▼
[ServoTester.set_target_offset]     [ros_client.gripper_move]
        │
        ▼
[ServoTester thread @ 30 Hz]   ← existing, unchanged
        │
        ▼
[C++ driver → robot]
```

### Control pipeline (per tick, `dt = 0.02 s`)

```
raw[i]     = latest_axis_value[i]                   # −1..+1 after normalization
d[i]       = 0 if |raw[i]| < deadband else raw[i]   # deadband cut
v[i]       = d[i] * max_velocity[i] * sign_map[i]   # mm/s (0..2), deg/s (3..5)
offset[i] += v[i] * dt                              # integrate
offset[i]  = clamp(offset[i], ±max_excursion[i])    # hard excursion bound

if armed:
    servo_tester.set_target_offset(offset)
```

### Axis normalization

At device open, we call `device.absinfo(code).max` for each of
`ABS_{X,Y,Z,RX,RY,RZ}` to discover the actual raw range (≈ ±350 for
the SpaceMouse Wireless BT, but we don't hard-code it). Normalization
is `raw / absinfo.max`, clipped to [-1, 1].

### Default axis mapping (world frame)

| Puck axis | Robot axis | Default sign |
|-----------|------------|--------------|
| ABS_X     | X (mm)     | +            |
| ABS_Y     | Y (mm)     | +            |
| ABS_Z     | Z (mm)     | +            |
| ABS_RX    | RX (°)     | +            |
| ABS_RY    | RY (°)     | +            |
| ABS_RZ    | RZ (°)     | +            |

The sign map is exposed in settings as a 6-element `[±1]` vector.
Any combination of sign flips the operator finds intuitive is accepted.
Non-permutation remapping (e.g. "swap X and Y") is not in scope.

### Buttons → gripper

The SpaceMouse Wireless BT exposes two buttons via evdev as `BTN_0`
(left) and `BTN_1` (right). Mapping:

- `BTN_0` press-edge → `ros_client.gripper_move(0,   force=gripper_force)` — **close**
- `BTN_1` press-edge → `ros_client.gripper_move(1000, force=gripper_force)` — **open**

Buttons only fire actions when **armed**. Unarmed, the UI still shows
the press (for the test page) but no gripper action is taken. Debounce:
repeat edges within `button_debounce_ms` (default 150 ms) are dropped.

### Battery

Read via `/sys/class/power_supply/hid-*` (udev exposes BT HID battery
when the device advertises the Battery Service — which this one does,
per `bluetoothctl info`). Polled once per second by a small helper in
`hid_state.py`. Published in `HidState.battery_pct`. UI shows a
warning banner below 15 %.

## Safety model (defense in depth)

1. **Not armed = no motion.** Default state is disarmed. Deflection is
   read and echoed, but `set_target_offset` is never called and buttons
   do nothing.
2. **Arm gates.** `POST /api/spacemouse/arm` refuses if:
   - robot not in `ENABLE` (mode 5) — must be enabled and idle;
   - any other motion is active (`_motion_active` lock already in
     `web/server.py`);
   - VLA executor or recorder is running;
   - no SpaceMouse device found.
3. **Integration clamp.** `max_excursion_xyz` (default 300 mm) and
   `max_excursion_rpy` (default 90°) cap the displacement from the
   anchor. Holding full deflection does not march the robot past the
   cap; to go further, disarm → re-arm (recaptures anchor).
4. **Workspace clamp.** Unchanged: `ServoTester` clamps each commanded
   pose via `vla.safety.clamp_pose()` against table plane + bounds.
5. **Idle auto-disarm.** If all 6 axes stay inside deadband for
   `idle_auto_disarm_s` (default 30 s) and no buttons fire in that
   window, the reader disarms itself (stops ServoTester, closes the
   stream, frees `_motion_active`). UI shows "auto-disarmed (idle)."
6. **Device disconnect.** evdev `OSError` → immediate disarm + one
   `ros.stop()` call. UI shows "device lost." Re-arm requires a manual
   click after reconnect (we never silently re-arm).
7. **E-stop.** The existing `/api/servo/estop` endpoint halts
   `ServoTester` and calls `ros.stop()`. The Pendant page wires a big
   red button to it; `Esc` on the page also triggers it.
8. **Driver watchdog.** Unchanged — existing stale-feedback detector
   continues to monitor `/joint_states_robot` and can restart the driver.

Note: ordinary disarm (click or idle timeout) does **not** call
`ros.stop()` — the robot holds the last commanded pose so the operator
can click Arm again and continue without the robot jerking. Only
E-stop and device-disconnect call `ros.stop()`.

## API surface (new)

All under the `/api/spacemouse/` prefix in `dobot_ros/web/server.py`:

| Method | Path                  | Body / Params                         | Effect                                                                                   |
|--------|-----------------------|---------------------------------------|------------------------------------------------------------------------------------------|
| POST   | `/api/spacemouse/arm`    | `{}`                                  | Check gates → start `ServoTester` → start reader; returns anchor pose or 400 with reason |
| POST   | `/api/spacemouse/disarm` | `{}`                                  | Stop reader → stop `ServoTester`; robot holds last pose                                  |
| POST   | `/api/spacemouse/estop`  | `{}`                                  | Proxies to `servo_estop()` (halt stream + `ros.stop()`); always allowed                  |
| GET    | `/api/spacemouse/state`  | —                                     | Current `HidState` snapshot (polling fallback for WS)                                    |
| GET    | `/api/spacemouse/settings` | —                                   | Read current pendant settings                                                            |
| POST   | `/api/spacemouse/settings` | `{...}`                             | Update and persist via existing `_settings_store`                                        |
| WS     | `/ws/spacemouse`         | —                                     | Live `HidState` stream at 10 Hz (downsampled from the 50 Hz internal state)              |

### Arm flow

```
user clicks "Arm"
  → POST /api/spacemouse/arm
  → check gates (mode, motion lock, device present, no VLA)
  → _motion_active = "pendant"
  → ServoTester.start()           # captures current pose as anchor
  → SpaceMouseReader.arm()        # reset offset to 0, start integration
  → 200 { status: "armed", anchor_pose: [...] }
```

### Disarm flow

```
click "Disarm"  |  idle timeout  |  device disconnect  |  E-stop
  → SpaceMouseReader.disarm()     # freeze offset integration
  → ServoTester.stop()            # close ServoP stream; robot holds last pose
  → _motion_active = None         (iff reader was the owner)
  → on device-disconnect or E-stop: ros.stop() as well
```

## Settings (persisted under `pendant` group in `_settings_store`)

| Key                   | Default          | Notes                                  |
|-----------------------|------------------|----------------------------------------|
| `max_velocity_xyz`    | 80.0 (mm/s)      | Full-deflection speed on each trans axis |
| `max_velocity_rpy`    | 30.0 (°/s)       | Full-deflection speed on each rot axis |
| `deadband`            | 0.10             | Fraction of full scale                 |
| `sign_map`            | `[1,1,1,1,1,1]`  | Per-axis sign                          |
| `max_excursion_xyz`   | 300.0 (mm)       | Hard cap on integrated offset          |
| `max_excursion_rpy`   | 90.0 (°)         | Hard cap on integrated offset          |
| `idle_auto_disarm_s`  | 30.0 (s)         | Auto-disarm after N seconds idle       |
| `button_debounce_ms`  | 150              | Per-button edge debounce               |
| `gripper_force`       | 20 (%)           | Matches existing gripper default       |

The reader subscribes to `_settings_store.subscribe("pendant", ...)`
so live edits in the UI take effect without disarm/re-arm — matching
the existing `ServoTester` pattern.

## Web UI

### SpaceMouse page (new tab `/spacemouse`)

> **Name note:** the existing `pendant.html` / `pendant.js` / `pendant.css`
> are a different thing — the mobile touch-based teach pendant. We use
> the `spacemouse` namespace throughout to avoid collision.

Single-column layout. The **Live HID** panel is visible whether armed
or disarmed — it *is* the test page.

```
┌───────────────────────────────────────────────┐
│  Pendant (SpaceMouse Wireless BT)     ⚡ 97%  │  battery + device pill
│  ○ disconnected / ◔ connected / ● armed       │
├───────────────────────────────────────────────┤
│                                               │
│   [   ARM   ]           [ E-STOP ]            │
│                                               │
│   Anchor: X 350.2  Y -10.4  Z 180.0           │
│           RX 180   RY  0.0  RZ  90.0          │
│                                               │
│   Offset: X  +12.3 mm   RX  +0.0°             │
│           Y   -4.1 mm   RY  +2.7°             │
│           Z   +0.0 mm   RZ  +0.0°             │
├─── Live HID (always visible) ─────────────────┤
│   TX  ▓▓▓▓▓▓░░░░░░  +0.42                     │
│   TY  ░░░░▓▓░░░░░░  -0.10                     │
│   TZ  ░░░░░░░░░░░░   0.00   (deadband hit)   │
│   RX  ░░░░░░░░░░░░   0.00                     │
│   RY  ░░░░░░░░░░░░   0.00                     │
│   RZ  ░░░▓▓▓▓▓░░░░  +0.15                     │
│   Buttons: [CLOSE ●]  [ OPEN ○]               │
├─── Settings (collapsible) ────────────────────┤
│   Max vel XYZ / RPY, Deadband, Idle disarm,   │
│   Sign map per axis, [Save]                   │
└───────────────────────────────────────────────┘
```

Behavior:

- **Live HID panel visible always.** Pre-arm, this is the verification
  page — watch bars move, press buttons, confirm the puck is alive.
- **Arm button disabled** with hover tooltip when gates fail
  ("Robot not enabled," "Motion active — stop it first,"
  "SpaceMouse disconnected," etc.).
- **Esc key on the page → E-stop.**
- **Offset readout** updates from the WebSocket stream.
- **Buttons glow red/green on press-edge**, both armed and disarmed.
  When disarmed, caption notes "(no gripper action while disarmed)" so
  the operator isn't surprised that nothing moved.
- **BT disconnect banner.** While the device is gone, a yellow banner
  shows "Device disconnected — reconnecting…" and the status pill
  goes to `○`. When it reconnects, banner clears; re-arm is manual.

### Diagnostic echo on the existing servo-tester page

Small read-only card added to the servo-tester page: 6 tiny axis bars,
2 button dots, device status pill. No arm button, no settings. Uses
the same `/ws/spacemouse` stream. Purpose: help verify the puck during
ServoP tuning without context-switching tabs.

### Frontend stack

Matches the rest of the dashboard: static HTML + vanilla JS + WebSocket
client. No framework. Follows the `web/static/` pattern.

## Docker passthrough

The host BT stack owns pairing; only the event node needs to reach the
container. The event number can shift on reconnect, so we mount the
`/dev/input/` directory and auto-detect by vendor/product at read time.

```yaml
# docker-compose.yml
services:
  dobot:
    # ...existing config...
    devices:
      - /dev/input:/dev/input:ro
    group_add:
      - "${INPUT_GID:-104}"
```

`INPUT_GID` goes into `.env.example` with a comment and a discovery
one-liner: `getent group input | cut -d: -f3`. Read-only is sufficient
— evdev doesn't write to event nodes. No udev rules inside the
container, no `CAP_*` grants.

### Dockerfile

`apt-get install -y python3-evdev` in `docker/Dockerfile`. That's the
only dep added. (`python3-evdev` is packaged by Ubuntu/Debian for
Jazzy's base image; no pip needed.)

### Startup behavior

- Device absent at startup → web server boots normally; `GET /api/spacemouse/state`
  returns `{device: "not_found"}`; `POST /api/spacemouse/arm` returns 400
  with a reason. No crash loop.
- No BT / no SpaceMouse at all → Pendant tab shows "disconnected" forever;
  nothing else breaks.
- Permission failure (container user not in `input` group) → single
  clear ERROR log line at first `find_device()` call; UI shows
  "permission denied on /dev/input" pill. Easy to diagnose.

## Testing

Four tiers, mirroring the existing `servo/tester.py` test approach:

1. **Unit — HID decode + math.** Mock evdev with a stub feeding scripted
   `(code, value)` tuples. Verify: normalization uses actual
   `absinfo.max`, deadband cuts below threshold, `sign_map` flips
   correctly, `offset[i] += v*dt` matches hand-computed values, clamp
   bounds are enforced, button edge detection + debounce timing.
2. **Unit — state machine.** Test `SpaceMouseReader.arm / disarm /
   idle_auto_disarm` with an injected `time.monotonic`. Verify: arm
   fails if no device; idle timer fires at the right instant;
   disconnect path disarms; double-arm is a no-op.
3. **Integration — in-process, mocked `ServoTester` and `ros_client`.**
   Arm → scripted deflection sequence → assert exact `set_target_offset`
   call sequence; button presses → assert `gripper_move` args.
4. **Manual smoke — hardware checklist** in `docs/pendant-bringup.md`:
   - Open `/pendant`, verify device pill is green, battery shown.
   - Watch live bars with robot **disabled**, confirm all 6 axes +
     both buttons work.
   - Enable robot, Arm, push gently along each axis, verify direction
     of motion matches the sign map.
   - Test deadband (tiny push = no motion).
   - Test excursion clamp (hold full deflection, watch offset stop at 300 mm).
   - Test idle auto-disarm (arm, walk away, confirm disarm at 30 s).
   - Test BT disconnect (power off puck while armed) → immediate
     disarm + `ros.stop()`.
   - Test E-stop button + `Esc` key.
   - Test gripper open/close on button press; confirm no action when
     disarmed.

## Files

### New

```
dobot-ros/dobot_ros/spacemouse/__init__.py
dobot-ros/dobot_ros/spacemouse/hid_state.py
dobot-ros/dobot_ros/spacemouse/reader.py
dobot-ros/dobot_ros/web/static/spacemouse.html
dobot-ros/dobot_ros/web/static/js/spacemouse.js
dobot-ros/dobot_ros/web/static/css/spacemouse.css
tests/spacemouse/__init__.py
tests/spacemouse/conftest.py
tests/spacemouse/test_reader.py
tests/spacemouse/test_state_machine.py
tests/spacemouse/test_integration.py
docs/spacemouse-bringup.md
```

### Modified

```
dobot-ros/dobot_ros/web/server.py    (add /api/spacemouse/* routes, /ws/spacemouse, settings group, /spacemouse FileResponse, nav link)
dobot-ros/dobot_ros/web/static/index.html   (nav link to SpaceMouse page)
dobot-ros/dobot_ros/web/static/servo.html   (small HID echo card — if servo tester has its own page; otherwise add to index.html servo section)
docker-compose.yml                   (devices + group_add)
docker/Dockerfile                    (python3-evdev)
.env.example                         (INPUT_GID)
```

### Untouched (explicit non-changes for the "additive only" guarantee)

```
dobot-ros/dobot_ros/ros_client.py
dobot-ros/dobot_ros/servo/tester.py
dobot-ros/dobot_ros/servo/patterns.py
dobot-ros/dobot_ros/cli.py
dobot-ros/dobot_ros/shell.py
dobot-ros/dobot_ros/gripper_node.py
dobot-ros/dobot_ros/pick.py
dobot-ros/dobot_ros/strategies/*
dobot-ros/dobot_ros/vla/*
dobot-ros/dobot_ros/validation.py
dobot-ros/dobot_ros/driver.py
dobot-ros/dobot_ros/config.py
```

## Open questions / deferred

- **Tool-frame jogging.** Not in v1. The reader's integration is
  world-frame only. To add tool-frame later, the tick method would
  read the current TCP orientation and rotate the deflection vector
  before integration. This is a pure extension — no change to the
  public API or to `ServoTester`.
- **CLI surface.** Skipped for v1; revisit if we find ourselves
  wanting `dobot-ros pendant test` or similar from the shell.
- **Joystick / gamepad support.** Out of scope; this is a
  SpaceMouse-specific design. A generic evdev-jog driver would be a
  separate project.
- **Multiple pucks.** One device at a time. If future need arises for
  two pendants, `find_device()` can return a list and the reader
  selects by user preference.

## Risk register

| Risk                                                      | Mitigation                                                                  |
|-----------------------------------------------------------|-----------------------------------------------------------------------------|
| BT reconnect changes event node number                    | Discover by vendor/product each time; re-scan on `OSError`.                 |
| Operator bumps puck when disarmed → no motion (by design) | Arm gate explicit; UI makes disarmed state obvious.                         |
| Operator walks away armed                                 | Idle auto-disarm at 30 s (configurable).                                    |
| BT dropout mid-motion                                     | evdev `OSError` → immediate disarm + `ros.stop()`.                          |
| Puck feeds values the wrong direction                     | Sign map in settings; test page lets operator verify before arming.         |
| Full-deflection march would exceed workspace              | Excursion clamp stops integration; `ServoTester` workspace clamp is backstop. |
| Input group GID mismatch on a different host OS           | `.env.example` documents discovery; clear error pill if denied.             |
| Reader thread panics mid-jog                              | `ServoTester`'s idle timeout will eventually halt the stream; robot holds pose; E-stop always works. |
