# Vision-Guided Picking — Theory, Design, and Implementation

This document explains how the Dobot CR5 picks objects from a table using a RealSense depth camera. It covers the theory behind camera-robot calibration, the math of ray-plane projection, the strategy system for pick planning, and the safety architecture that prevents the robot from crashing into things.

For VLA (Vision-Language-Action) integration, see [VLA.md](VLA.md). For general system architecture, see [CLAUDE.md](CLAUDE.md).

---

## The Problem

We have a robot (Dobot CR5) with a gripper (DH Robotics AG-105) and a camera (Intel RealSense) mounted in front of the table at an oblique angle. The goal: click on an object in the camera view and have the robot pick it up.

This sounds simple but involves three hard subproblems:

1. **Where is the object in robot coordinates?** The camera sees pixels; the robot commands millimeters. We need a transform from one to the other.
2. **What angle should the gripper approach at?** The object's orientation in the image is distorted by perspective. We need the true rotation on the table surface.
3. **How deep should the gripper go?** Camera depth is noisy (±38mm at working distance). Going too low crashes the gripper into the table; too high misses the object.

---

## Coordinate Frames

Understanding the coordinate frames is essential. There are three:

### Camera Frame
The RealSense uses the standard computer-vision convention:
- **Z**: forward (along the optical axis, away from the lens)
- **X**: right (in the image)
- **Y**: down (in the image)
- Units: **meters**

A pixel `(px, py)` maps to a 3D ray in camera frame via the camera intrinsics:
```
ray_camera = [(px - cx) / fx, (py - cy) / fy, 1.0]
```
where `(cx, cy)` is the principal point and `(fx, fy)` are the focal lengths in pixels.

### Robot Frame
The Dobot CR5 uses:
- **X**: forward from the base
- **Y**: left
- **Z**: up
- Units: **millimeters**
- Origin: center of the robot base

All joint angles, cartesian poses, and motion commands are in this frame.

### Table Frame
The table is a flat, level surface at a known height `table_z` in robot frame. For our setup:
- The table plane is `Z = table_z` (constant because the table is level)
- `table_z` is measured by touching the gripper tip to the table and recording the wrist Z
- Typical value: ~28mm (the wrist is close to the base height when the gripper tip touches the table)

---

## Camera-Robot Calibration

### What We're Solving For

We need two things:
1. **Camera intrinsics** `(fx, fy, cx, cy)` — how the camera maps 3D points to pixels. The RealSense knows these from factory calibration.
2. **Camera extrinsics** `(R, t)` — where the camera is in robot frame.
   - `R` is a 3×3 rotation matrix: transforms a direction from camera frame to robot frame
   - `t` is a 3×1 translation vector: the camera's origin expressed in robot frame (mm)

Together: `p_robot = R · p_camera + t`

### How We Solve It: Kabsch Algorithm (SVD)

We collect correspondence points — pairs of `(camera_xyz, robot_xyz)` — by:
1. Moving the robot TCP to a visible location
2. Clicking that pixel in the camera view
3. The camera server provides `camera_xyz` from depth at that pixel
4. The robot provides `robot_xyz` from its current cartesian pose

With ≥3 points, we solve for `R` and `t` using the Kabsch algorithm:

**Step 1**: Center both point sets.
```
cam_centroid = mean(cam_points)
rob_centroid = mean(rob_points)
cam_centered = cam_points - cam_centroid
rob_centered = rob_points - rob_centroid
```

**Step 2**: Compute the cross-covariance matrix.
```
H = cam_centered.T @ rob_centered    # 3×3 matrix
```

**Step 3**: SVD decomposition.
```
U, S, V^T = svd(H)
```

**Step 4**: Compute rotation with reflection correction.
```
d = det(V^T · U^T)    # +1 for proper rotation, -1 for reflection
R = V^T · diag(1, 1, d) · U^T
```

**Step 5**: Compute translation.
```
t = rob_centroid - R · cam_centroid
```

The singular values `S` tell us about the quality of the calibration:
- All large: points are well-distributed in 3D — good calibration
- One small: points are planar (normal for table-surface calibration)
- Two small: points are collinear — **degenerate**, rejected with an error

The residual error (mean distance between transformed camera points and robot points) is reported in mm. Under 5mm is excellent; 5-15mm is acceptable; over 15mm suggests a bad correspondence point.

**Implementation**: `dobot_ros/vision_transform.py`, `VisionTransform.solve_from_points()`

### Why We Solve Locally (Not on the Camera Server)

The camera server has its own Kabsch solver, but we duplicate it on the robot side because:
- We need the actual `R` and `t` matrices for ray-plane projection (the camera server only exposes a `transform` endpoint, not the raw matrices)
- Having `R` and `t` locally lets us do all the math in one place — no HTTP round-trips during time-critical pick planning
- We can inspect and debug the transform from the Settings tab

---

## Ray-Plane Projection: The Key Insight

### Why Not Use Depth for XY?

The obvious approach to finding an object's table position is:
1. Get the object's `(px, py)` from detection
2. Get the depth `d` at that pixel from the RealSense
3. Compute `camera_xyz = (px - cx) / fx * d, (py - cy) / fy * d, d)`
4. Transform to robot frame: `robot_xyz = R · camera_xyz + t`

This works when the camera is directly overhead. But our camera is **oblique** — it views the table at an angle. At oblique angles, depth noise propagates into XY:

```
At 45° oblique angle:
  38mm depth noise → ~27mm XY error on the table surface
  
The AG-105 gripper finger opening: 105mm
The object (block): ~30-50mm wide

→ 27mm XY error = you miss the object ~50% of the time
```

### The Fix: Ray-Plane Intersection

Instead of using depth, we use geometry. We know:
1. Where the camera is (from calibration: `R, t`)
2. Where the table is (from table Z calibration: `Z = table_z`)
3. The direction of the ray through the clicked pixel (from intrinsics)

So we cast a ray from the camera through the pixel and intersect it with the table plane:

```
Step 1: Pixel → ray direction in camera frame
  ray_cam = normalize([(px - cx) / fx, (py - cy) / fy, 1.0])

Step 2: Transform ray to robot frame
  ray_robot = R · ray_cam
  origin_robot = t    (camera origin in robot frame)

Step 3: Intersect with table plane (Z = table_z)
  t_param = (table_z - origin_robot.z) / ray_robot.z
  hit_x = origin_robot.x + t_param · ray_robot.x
  hit_y = origin_robot.y + t_param · ray_robot.y
```

**Result**: `(hit_x, hit_y)` in robot frame, in millimeters, with **zero depth noise**.

The only error sources are:
- Calibration accuracy (typically <5mm)
- Camera intrinsic accuracy (factory-calibrated, sub-pixel)
- Table Z accuracy (one-touch measurement, <1mm)

**Implementation**: `dobot_ros/vision_transform.py`, `VisionTransform.pixel_to_table()`

### Edge Cases

- **Parallel ray**: If the camera looks along the table plane (ray_robot.z ≈ 0), the intersection is at infinity. Detected and rejected with `ValueError`.
- **Behind camera**: If `t_param < 0`, the table is behind the camera. Rejected.
- **Principal point**: The pixel `(cx, cy)` produces a ray along the camera's optical axis. This is valid and works correctly.

---

## Perspective-Corrected Object Rotation

### The Problem

The camera server's object detection gives `rotation_deg` — the 2D orientation of the object in the **image plane**. When the camera is oblique, this is NOT the object's true rotation on the table. A block sitting square on the table appears rotated in the image due to perspective.

### The Fix: Dual-Point Projection

We project two points on the object's principal axis to the table plane:

```
Step 1: Compute two pixel points along the object's axis
  offset = 20 pixels
  p1 = (cx - offset·cos(image_rot), cy - offset·sin(image_rot))
  p2 = (cx + offset·cos(image_rot), cy + offset·sin(image_rot))

Step 2: Project both to table via ray casting
  (x1, y1) = pixel_to_table(p1)
  (x2, y2) = pixel_to_table(p2)

Step 3: Compute true angle in robot XY
  true_rot = atan2(y2 - y1, x2 - x1)
```

The result is the object's orientation on the table surface, free of perspective distortion.

**Implementation**: `dobot_ros/vision_transform.py`, `VisionTransform.image_rot_to_table_rot()`

---

## Depth as a Sanity Check (Not Primary Source)

Depth is noisy for XY but useful as a **secondary signal**:

### Object Presence Verification
Before picking, we check that something is actually above the table at the target pixel:
```
expected_depth = distance from camera to table plane at this pixel
measured_depth = camera's depth reading at this pixel
delta = expected_depth - measured_depth

if delta < 5mm → nothing there (just table surface). Abort.
if delta > 5mm → object is present, estimated height ≈ delta.
```

### Object Height Estimation
For tall objects (cups, bottles), the grasp Z should be at the object's midpoint, not at the table surface. Depth gives us the height:
```
object_height_mm = (expected_depth - measured_depth) × 1000
grasp_z = table_z + max(min_clearance, object_height / 2)
```

Since we're averaging depth over the object's bounding box (many pixels), the noise is reduced from ±38mm to ~±10-15mm — good enough for height binning (flat / short / tall).

### Confidence Scoring
Multiple signals combine into a pick confidence:

| Signal | Green | Yellow | Red |
|--------|-------|--------|-----|
| Object presence (depth delta) | >10mm above table | 5-10mm | <5mm |
| Depth consistency (stddev) | <15mm | 15-30mm | >30mm |
| Tracking stability | >5 frames | 2-5 | 1 frame |
| Within workspace | yes | near edge | outside |
| Object size | >500px | 200-500px | <200px |
| Calibration age | <1 hour | 1-8 hours | >8 hours |

Score 5-6 = auto-pick allowed. 3-4 = preview required. <3 = blocked.

**Implementation**: `dobot_ros/vision_transform.py`, `VisionTransform.pick_confidence()`

---

## Pick Strategy System

### Architecture

A pick strategy is a pluggable Python module that decides HOW to approach and grasp an object. It does two things:

1. **`plan(ctx)`** — pure planning code. Given a `PickContext` (robot-frame coordinates, table Z, active tool, object data) returns a `PickPlan` (sequence of waypoints + gripper actions). No I/O, no ROS calls, no camera access — easy to test.
2. **`execute(ctx, plan, client, confirm_fn, servo=None)`** — performs the motion. MovL strategies call `client.move_pose()`; ServoP strategies stream offsets through the running `ServoTester`. Strategies own their own motion mode so the orchestrator stays motion-agnostic.

```
Server resolves WHERE (ray projection)
   ↓
Server runs setup (Tool 1 → Vertical → lock_vertical)
   ↓
User confirms via modal (WebSocket)
   ↓
Server re-plans against post-setup pose
   ↓
strategy.execute(...) owns the motion
```

### PickContext (Input)

```python
@dataclass
class PickContext:
    robot_x: float             # mm, from ray-plane projection
    robot_y: float             # mm
    table_z: float             # mm, wrist Z when tip touched table (calibration)
    min_clearance_mm: float    # safety floor above table
    rotation_deg: float        # perspective-corrected, true table-plane rot
    current_pose: List[float]  # post-setup [X,Y,Z,RX,RY,RZ]
    tool_index: int            # 0 = wrist, 1 = fingertip
    tool_length_mm: float      # offset for Tool ≥1 (e.g. 203mm for AG-105)
    object_height_mm: float    # from depth estimate (0 if not detected)
    object_present: bool       # from depth check
    params: dict               # strategy-specific parameters from JSON

    def pose_z_from_table_z(self, height_above_table: float) -> float
        # Converts "height above table surface" → pose Z in active tool frame.
        # Tool 0: table_z + height_above_table    (table_z is wrist frame)
        # Tool N: table_z - tool_length_mm + height_above_table
```

### PickPlan (Output)

```python
@dataclass
class PickPlan:
    waypoints: List[PickWaypoint]  # ordered sequence
    move_speed: Optional[int]      # robot speed factor (MovL only)

@dataclass
class PickWaypoint:
    pose: List[float]              # [X,Y,Z,RX,RY,RZ]
    label: str                     # "approach", "grasp", etc.
    pause_s: float                 # dwell after arriving
    gripper_action: Optional[str]  # "open" | "close" | None
    gripper_pos: Optional[int]     # 0-1000
    confirm: Optional[str]         # if set, orchestrator modal-confirms BEFORE this wp
```

### Built-in Strategies

All strategies force the wrist to RX=180, RY=0 at every waypoint — the setup phase guarantees the robot is vertical and `lock_vertical` is on, so the pick never inherits a tilted wrist from the prior pose.

**Simple Top-Down (MovL)** (`simple_top_down.py`, `motion_mode="movl"`):
Straight vertical approach from directly above, point-to-point MovL between waypoints. 4 waypoints:
1. Open gripper at approach height
2. Descend to grasp Z
3. Close gripper
4. Retract straight up

Parameters: approach_height, grasp_clearance, lift_height, gripper open/close positions, force, speed, use_object_height.

**Angled Approach (MovL)** (`angled_approach.py`, `motion_mode="movl"`):
Approach from a lateral offset, then drop straight down. 5 waypoints:
1. Open gripper at hover point (laterally offset from target)
2. Move to approach (directly above target)
3. Descend to grasp Z
4. Close gripper
5. Retract straight up

Additional parameters: hover_height, approach_offset, approach_direction (auto/back/left/right/front), pre_rotate.

**Servo Top-Down (ServoP)** (`servo_top_down.py`, `motion_mode="servo"`):
Same waypoint shape as Simple Top-Down, but motion is streamed through the `ServoTester` instead of point-to-point MovL. Inherits the tester's rate-limited tick loop (`max_velocity_xyz` gives a hard cap on actual end-effector speed regardless of waypoint distance), `lock_vertical` force-projection (every emitted pose is clamped to RX=180/RY=0), and tool-aware floor guard. Each waypoint is issued via `set_target_offset()` and waited on until the tester's commanded offset converges within `converge_tol_mm`.

Additional parameters: `max_velocity_xyz` (linear speed cap during pick), `converge_tol_mm`, `waypoint_timeout_s`, `confirm_before_close` (if true, inserts a per-waypoint confirm gate right before the gripper close).

### Setup Phase and Confirm Broker

Every pick runs through a mandatory setup + confirmation sequence before any motion:

1. **Build setup list**: check active tool and current wrist RX/RY. Insert `Switch to Tool 1`, `Orient wrist vertical`, and `Enable lock_vertical` steps — steps the robot is already in get skipped.
2. **Modal confirm**: `PickConfirmBroker.ask()` pushes a `pick_confirm` message over the existing `/ws/state` WebSocket with the summary (strategy + target + setup steps + waypoint list). The Python worker thread blocks on a `threading.Event` until the browser POSTs to `/api/pick/confirm`, or 120 s timeout.
3. **Run setup motion**: idempotent — only moves what's not already in place.
4. **Re-plan** against the post-setup pose (so waypoint Z uses the now-active tool frame).
5. **Hand off** to `strategy.execute(...)`. Any waypoint with a `confirm` string pauses for another modal before running.

E-STOP cancels all pending confirms via `broker.cancel_all()` so stuck modals can't deadlock the next pick. Multiple connected browsers replay pending confirms on connect and share a first-reply-wins dismissal.

**Implementation**: `dobot_ros/web/pick_confirm.py`, `dobot_ros/web/server.py` (`vision_execute`), `app-vision.js` (`handlePickConfirm`).

### Adding a New Strategy

1. Create `dobot_ros/strategies/my_strategy.py`
2. Subclass `PickStrategy`, set `name`, `slug`, `description`, `motion_mode` (`"movl"` or `"servo"`)
3. Implement `parameter_defs()` → returns list of `ParameterDef`
4. Implement `plan(ctx: PickContext)` → returns `PickPlan`
5. Implement `execute(ctx, plan, client, confirm_fn, servo=None)` — issue motion; call `confirm_fn(msg)` before any gated waypoint (it raises on cancel/timeout)
6. Create `dobot_ros/strategies/params/my_strategy.json` with defaults
7. Restart the web server — the registry auto-discovers it

No server or GUI changes needed. The registry scans for `PickStrategy` subclasses at startup, and the GUI renders the parameter form dynamically from `parameter_defs()`.

**Implementation**: `dobot_ros/strategies/`

---

## Safety Architecture

### Defense in Depth

The system has multiple independent safety layers. A failure in any one layer is caught by the next:

```
Layer 1: Strategy (computes grasp_z ≥ table_z + min_clearance)
  ↓
Layer 2: Safety clamps (clamp_delta, clamp_pose — workspace bounds + NaN guard)
  ↓
Layer 3: ros_client validation (validate_finite, validate_pose_bounds, validate_joint_bounds)
  ↓
Layer 4: Robot firmware (joint limits, collision detection, workspace protection)
  ↓
Layer 5: Hardware E-STOP (physical button on teach pendant)
```

### What Each Layer Catches

| Layer | NaN/Inf | Out of workspace | Too fast | Below table | Collision |
|-------|---------|-----------------|----------|-------------|-----------|
| Strategy | — | — | — | ✓ (clearance) | — |
| Safety clamps | ✓ | ✓ | ✓ (step caps) | ✓ (Z floor) | — |
| ros_client | ✓ | ✓ (hard bounds) | ✓ (t clamped) | ✓ (Z floor) | — |
| Firmware | — | ✓ (joint limits) | ✓ (speed factor) | ✓ (workspace) | ✓ |
| E-STOP | ✓ | ✓ | ✓ | ✓ | ✓ |

### Mutual Exclusion

Only one motion mode can be active at a time:
- **Point-to-point** (MovJ/MovL): jog, pick, workspace move
- **Streaming servo** (ServoP): servo tester, VLA executor

The `_check_motion_free()` guard in `server.py` blocks any motion command if another mode is active. Attempting to start a pick while the servo tester is running returns HTTP 409 with a clear error.

### Pick Execution Safety

During a pick sequence:
1. Each `move_pose()` is followed by `wait_for_cartesian_motion()` — the next step only starts after the robot arrives (including orientation convergence with angle wrapping).
2. If any wait times out, the sequence aborts with `RuntimeError` and `client.stop()` is called.
3. The `finally` block always calls `client.stop()` and releases the motion lock, even on exceptions.
4. The web server runs pick execution in a threadpool (not the async event loop) so the E-STOP button, WebSocket, and status endpoints remain responsive during the 30-60s sequence.

### Feedback-Loss Halt

If `get_cartesian_pose()` raises an exception during `wait_for_cartesian_motion()` (e.g., the ROS topic subscription drops), the function immediately calls `self.stop()` and returns `False`. The robot halts rather than continuing blind.

---

## Simulation Mode

Before executing a pick on the real robot, the operator can simulate it in the 3D view:

1. Click an object → plan is computed (ray projection, strategy, waypoints)
2. Click "Simulate" → the server calls `InverseKin` for each waypoint to get joint angles (pure computation, no motion)
3. The 3D view animates the robot through the waypoint sequence with the gripper opening/closing
4. Current real pose shows as a transparent ghost; simulated pose is solid
5. If the path looks wrong → Cancel. If it looks right → Execute.

**Implementation**: `robot3d.js` (`simulateRobotMotion`, `stopRobotSimulation`)

---

## Grid Overlay

The Vision tab can project a table-plane grid onto the camera view to visualize calibration accuracy:
- Every 100mm in robot XY, a point is projected to pixel coords via `robot_to_pixel()`
- Horizontal and vertical lines are drawn on the canvas
- The grid shows perspective distortion from the oblique camera angle
- The robot base origin (0,0) is marked
- Only available when the vision transform is calibrated

This is useful for verifying that the calibration "looks right" before attempting a pick — if the grid doesn't align with the table edges, recalibrate.

**Implementation**: `server.py` `GET /api/vision/grid`, `app.js` `visFetchGrid()`

---

## Web Dashboard Integration

### Vision Tab

The operational pick interface. Shows:
- Camera view with detection overlays (bounding boxes, selected target, waypoint markers)
- Strategy selector dropdown + dynamic parameter form
- Full pick preview: ray-projected XY, corrected rotation, depth checks, confidence, waypoints
- Simulate → Execute flow with confirmation

### Calibration Tab

Setup-only. Three phases:
1. **Table Z** — touch gripper to table, record wrist Z
2. **Camera points** — record ≥3 pixel-robot correspondences
3. **Solve** — run SVD locally, save `vision_calibration.json`

### API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/vision/status` | GET | Calibration state, table Z, intrinsics |
| `/api/vision/calibrate` | POST | Solve extrinsics from correspondence points |
| `/api/vision/plan` | POST | Plan a pick (ray projection + strategy) |
| `/api/vision/execute` | POST | Execute (requires `confirm: true`). Runs setup + modal confirm + re-plan + strategy.execute |
| `/api/vision/grid` | GET | Table-plane grid as pixel coordinates |
| `/api/detection/objects` | GET | Proxy to camera server's object detection |
| `/api/inverse-kin` | POST | Compute joint angles for simulation |
| `/api/pick/confirm` | POST | Browser → backend reply for a pending `pick_confirm` modal (`{id, choice}`) |
| `/api/strategies/list` | GET | All strategies with parameter definitions + `motion_mode` |
| `/api/strategies/active` | GET/POST | Active strategy selection |
| `/api/strategies/{slug}/params` | GET/POST | Per-strategy parameter values |

**WebSocket messages** on `/ws/state` (in addition to the state push):
| `type`                     | Direction | Fields                                  |
|----------------------------|-----------|-----------------------------------------|
| `pick_confirm`             | server → browser | `{id, summary, choices[]}` — show modal |
| `pick_confirm_resolved`    | server → browser | `{id, choice}` — dismiss modal (another browser answered) |

---

## File Map

```
dobot_ros/
├── vision_transform.py    # Core math: ray-plane projection, rotation correction,
│                          # Kabsch solver, depth checks, confidence scoring
├── validation.py          # Defense-in-depth: NaN/Inf, pose bounds, joint limits
├── vision.py              # Camera server HTTP client (detection, depth, transform)
├── pick.py                # Legacy pick executor (still works, uses camera-server transform)
├── strategies/
│   ├── base.py            # PickStrategy ABC, ParameterDef, PickContext (tool-aware),
│   │                      # PickPlan, PickWaypoint (with confirm gate)
│   ├── simple_top_down.py # Vertical MovL approach (motion_mode="movl")
│   ├── angled_approach.py # Lateral MovL hover + drop (motion_mode="movl")
│   ├── servo_top_down.py  # Vertical ServoP approach through ServoTester (motion_mode="servo")
│   └── params/            # Per-strategy JSON parameter files
├── web/
│   ├── server.py          # FastAPI: /api/vision/*, /api/strategies/*, pick orchestrator
│   ├── pick_confirm.py    # PickConfirmBroker: modal request/reply over WS
│   ├── vision_calibration.json  # Stored R, t, intrinsics, table_z (from Solve)
│   └── static/
│       ├── index.html     # Includes #pick-confirm-modal (Bootstrap, static backdrop)
│       └── js/
│           ├── app.js         # WS dispatcher routes pick_confirm to window.handlePickConfirm
│           ├── app-vision.js  # Vision tab UI + modal handler
│           └── robot3d.js     # 3D view: gripper viz, simulation mode
└── vla/
    └── safety.py          # Workspace clamps (also used by servo tester)
```

---

## References

- **Kabsch algorithm**: [Finding optimal rotation and translation between corresponding 3D points](https://nghiaho.com/?page_id=671)
- **RealSense depth accuracy**: [Depth post-processing for D400 series](https://dev.intelrealsense.com/docs/depth-post-processing)
- **Camera model**: [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- **Dobot CR5 ServoP**: [TCP-IP-Python-V4 SDK](https://github.com/Dobot-Arm/TCP-IP-Python-V4)
