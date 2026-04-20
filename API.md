# API Reference

HTTP + WebSocket surface of the Dobot CR5 web dashboard. Served by
`dobot-ros/dobot_ros/web/server.py`.

- **Base URLs** (all equivalent, all TLS where shown):
  - `http://localhost:7070/` — FastAPI direct
  - `https://jupiter.siegel.com/dashboard/` — via nginx reverse proxy
  - `https://localhost/dashboard/` — same (localhost cert SAN)
- **Content-Type** for POST/PATCH: `application/json`
- **Response shape** for action endpoints is typically:
  ```json
  {"success": true,  "result": …}
  {"success": false, "error":  "…"}
  ```
  GET endpoints return the data directly (no wrapper).

## Robot state

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/status` | Full state snapshot: connected, robot_mode (+ name), joint[6], cartesian[6], gripper fields, speed_factor, uptime, error |
| GET | `/api/position` | Just joint + cartesian |
| GET | `/api/config` | Robot IP, type, static bits from `.env` |

## Robot control

| Method | Path | Body | Effect |
|---|---|---|---|
| POST | `/api/enable` | — | EnableRobot; also snaps speed to 5% on enable |
| POST | `/api/disable` | — | DisableRobot |
| POST | `/api/clear` | — | ClearError |
| POST | `/api/stop` | — | Stop (emergency) |
| POST | `/api/drag/start` | — | StartDrag (teach-by-hand mode) |
| POST | `/api/drag/stop` | — | StopDrag |
| POST | `/api/speed` | `{speed: 1..100}` | SpeedFactor |
| POST | `/api/jog` | `{axis, distance, speed?, mode?}` | Validated jog: axis in {j1..j6, x, y, z, rx, ry, rz}, distance ∈ [-500, 500], speed ∈ [1, 100], mode in {user, tool} |
| POST | `/api/move_joints` | `{joints: [6]}` | Absolute joint move |

## Gripper

| Method | Path | Body | Effect |
|---|---|---|---|
| POST | `/api/gripper/init` | — | Init routine; must run before moves |
| POST | `/api/gripper/open` | — | Convenience: move to 1000 |
| POST | `/api/gripper/close` | — | Convenience: move to 0 |
| POST | `/api/gripper/move` | `{position, force?, speed?}` | position ∈ [0, 1000]; fire-and-forget |
| GET | `/api/gripper/status` | — | init status, state, current position |

Gripper state is also pushed over the `/ws/state` stream at 5 Hz.

## Calibration

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/calibration/status` | Calibration solver state |
| POST | `/api/calibration/record` | Record a (pixel → robot XYZ) correspondence |
| POST | `/api/calibration/solve` | SVD/Kabsch solve once enough points exist |
| POST | `/api/calibration/clear` | Clear all correspondences |
| POST | `/api/calibration/delete-point` | Remove one correspondence by index |
| GET | `/api/calibration/test` | Round-trip a pose through the solved transform |
| GET | `/api/calibration/camera-frame` | Current JPEG frame from the camera server |
| GET | `/api/calibration/table` | Current `table_plane.json` contents |
| POST | `/api/calibration/table/clearance` | `{clearance_mm}` — min distance the gripper may approach the table |
| POST | `/api/calibration/table/point` | Record a table corner at the current wrist pose |
| POST | `/api/calibration/table/z` | Record the table Z manually |
| POST | `/api/calibration/table/clear` | Clear table points |
| POST | `/api/calibration/workspace-move` | `{target_xy, height_mm}` — safe L-shaped move (up → XY → down) to a test point above the table |
| GET | `/api/calibration/depth-at` | `?px=…&py=…` — sample RealSense depth at pixel |

## Vision & pick

| Method | Path | Body | Purpose |
|---|---|---|---|
| GET | `/api/detection/objects` | — | Current detected objects (bbox, id, label) |
| POST | `/api/pick/plan` | `{object_id?, px?, py?}` | Plan from a single pick request |
| POST | `/api/pick/execute` | `{object_id?, px?, py?, confirm: true}` | Execute a planned pick |
| GET | `/api/camera` | — | Camera-server settings |
| POST | `/api/camera` | `{…}` | Patch camera-server settings |
| GET | `/api/strategies/list` | — | Registered pick strategies + their parameter schemas |
| GET | `/api/strategies/active` | — | Currently selected strategy + its current params |
| POST | `/api/strategies/active` | `{slug}` | Switch active strategy |
| GET | `/api/strategies/{slug}/params` | — | Read that strategy's params |
| POST | `/api/strategies/{slug}/params` | `{…}` | Merge-update that strategy's params |
| GET | `/api/vision/status` | — | calibrated?, table_z, intrinsics |
| GET | `/api/vision/grid` | — | Table-plane grid projected to pixel coords (for the overlay) |
| POST | `/api/vision/calibrate` | `{…}` | Solve the camera-to-robot transform |
| POST | `/api/vision/plan` | `{object_id?, px?, py?}` | Plan a pick from the Vision tab |
| POST | `/api/vision/execute` | `{object_id?, px?, py?, confirm: true}` | Execute a vision pick |
| POST | `/api/inverse-kin` | `{poses: [[6]…]}` | Batch IK for simulation rendering |

## VLA (Vision-Language-Action)

| Method | Path | Body | Purpose |
|---|---|---|---|
| GET | `/api/vla/status` | — | Executor state, recorder state, last error |
| GET | `/api/vla/episodes` | — | List of recorded drag-teach episodes |
| POST | `/api/vla/record/start` | `{task_name, …}` | Start a drag-teach recording |
| POST | `/api/vla/record/stop` | — | Stop + finalize |
| POST | `/api/vla/execute/start` | `{task, …}` | Start closed-loop VLA inference |
| POST | `/api/vla/execute/stop` | — | Halt the executor |

## Servo tester

| Method | Path | Body | Purpose |
|---|---|---|---|
| POST | `/api/servo/start` | `{servo_rate_hz?, t?, gain?, aheadtime?, csv_log?}` | Apply tuning and start the tick loop |
| POST | `/api/servo/stop` | — | Stop the tick loop; robot holds last target |
| POST | `/api/servo/estop` | — | Stop + ros.stop() |
| POST | `/api/servo/target` | `{offset: [6]}` | **Position mode** — set target offset; clears pattern |
| POST | `/api/servo/pattern` | `{name, params}` | Start a pattern (circle / sine / square / lissajous) |
| POST | `/api/servo/config` | `{servo_rate_hz?, t?, aheadtime?, gain?, max_velocity_xyz?, max_velocity_rpy?, idle_timeout_s?}` | Live tuning update; persists |
| GET | `/api/servo/config` | — | Current config + defaults (used by the UI's Reset button) |
| GET | `/api/servo/status` | — | running, mode, step count, latency p50/p95, clamps/overruns, anchor, last commanded pose |

Velocity mode (SpaceMouse) uses the in-process call `ServoTester.set_target_velocity(v_6d)`
and is not exposed as an HTTP endpoint — the reader runs inside the same process.

## Settings store

The settings store is the single source of truth for persistent UI +
tester config. Groups are free-form; the server only filters by key for
specific endpoints (e.g. `/api/servo/config`).

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/settings/current` | All groups merged into one object |
| PATCH | `/api/settings/current` | `{group: {key: value}, …}` deep-merge update |
| GET | `/api/settings/saved` | List named snapshots |
| POST | `/api/settings/saved` | `{name, description?}` snapshot current → named |
| GET | `/api/settings/saved/{name}` | Metadata + body for one snapshot |
| GET | `/api/settings/saved/{name}/download` | Raw JSON download |
| PATCH | `/api/settings/saved/{name}` | Rename / change description |
| DELETE | `/api/settings/saved/{name}` | Remove a snapshot |
| POST | `/api/settings/saved/{name}/load` | Apply a snapshot over `current` (safety-gated) |
| POST | `/api/settings/saved/import` | Import a JSON settings file |

## SpaceMouse pendant

| Method | Path | Body | Purpose |
|---|---|---|---|
| POST | `/api/spacemouse/arm` | — | Acquire motion lock and arm the reader; captures anchor pose |
| POST | `/api/spacemouse/disarm` | — | Graceful disarm; releases motion lock |
| POST | `/api/spacemouse/estop` | — | Hard stop (ros.stop()) + disarm |
| GET | `/api/spacemouse/state` | — | axes[6], buttons[2], device_status, armed, battery, offset, idle |
| GET | `/api/spacemouse/settings` | — | Live settings |
| POST | `/api/spacemouse/settings` | `{deadband?, sign_map?, max_velocity_xyz?, max_velocity_rpy?, …}` | Merge + validate + persist |

The reader self-disarms on idle timeout or device loss; its on-disarm
hook also releases the server motion lock, so subsequent arms succeed.

## WebSockets

| Path | Direction | Payload |
|---|---|---|
| `/ws/state` | server → client | Full `/api/status`-shape JSON pushed at 5 Hz; used by the dashboard for live joint/cartesian/gripper/mode updates |
| `/ws/spacemouse` | server → client | HID state snapshot — axes, buttons, device_status, armed, offset, event-age, idle |

Both are push-only; the client doesn't send frames.

## Error handling conventions

- Action endpoints return `200` with `{"success": false, "error": "…"}` on
  application errors (service timeout, invalid input for domain reasons,
  motion already active). `400` is reserved for schema violations; `500`
  for unhandled exceptions.
- Motion-active conflicts: arming the SpaceMouse when the servo tester or
  VLA executor is running returns `{"success": false,
   "error": "servo tester is running — stop it first"}` etc.
- Gripper commands when not initialized return a clear error instead of
  silently queueing.
