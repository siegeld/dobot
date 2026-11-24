# POLICIES — Hand-coded robot policies, tested in simulation

> **Status (2026-11-24):** phases 0–3 and 5 built and deep-tested in simulation (72 offline tests; sweeps at three noise levels; UI run / batch / replay / stop verified in the browser). Not yet done on
> hardware: `scripts/validate_fk.py` against the live arm (pins the Euler convention and RZ
> zero), and the first real `pick_colored_block` run (phase 4). The camera detection payload's
> `color` field is still unverified; `real/world.py` names the colour from the frame if absent.

A plan for a third control track beside vision picking ([VISION.md](VISION.md)) and
learned policies ([VLA.md](VLA.md)): **task programs written by hand in Python**
("pick up the red block on the table"), debugged against a simulator, and run
from the web dashboard on either the simulator or the real CR5.

The relationship to [HOMEBODY.md](HOMEBODY.md): a *policy* here is exactly what
HomeBody's VLM would emit as a sequence of skill calls, except a person (or
Claude in this CLI) writes the sequence. The primitive library built for this
track **becomes** the HomeBody skill library; the simulator built here is its
"rehearse before touching hardware" tier. Nothing in this plan is throwaway.

---

## What exists that this builds on

| Piece | Where | Reuse |
|---|---|---|
| Strategy pattern: pure `plan()` + I/O `execute()`, `ParameterDef` → auto-rendered form, JSON params per slug, registry | `strategies/base.py`, `/api/strategies/*` | Copy the shape for policies; reuse `ParameterDef` verbatim |
| Mock ROS client used by the VLA tests | `tests/vla/conftest.py` | The embryo of `SimRobot`: same method surface, but it must actually *move* and *grasp* |
| three.js CR5 viewer with ghost-robot "Simulate" playback (`startSimulation(steps)`) | `web/static/js/robot3d.js`, `app-vision.js` | Playback target. Today it animates joints + gripper only; it needs scene objects |
| Robot IK as an HTTP endpoint | `POST /api/inverse-kin` | **Only works with the robot powered on** — it calls the controller's IK service. The simulator cannot depend on it |
| URDF joint origins already transcribed to JS | `robot3d.js:14-19` | Port to Python for offline FK; validate against live data |
| Safety: floor guard, workspace bounds, `lock_vertical`, motion lock, `pick_confirm` broker | `web/server.py`, `servo/tester.py`, `web/pick_confirm.py` | Policies run *under* these on the real robot, and the sim enforces the same rules so a violation fails in sim first |
| Table/workspace calibration | `web/table_plane.json` | The sim world is built from it, so sim and real share one table |
| Camera server detection proxy | `GET /api/detection/objects` → `vision.py` `DetectedObject(label, …)` | The real-robot `World` adapter. Needs color naming (see risks) |

Nothing in the repo today does kinematics in Python (`grep -ri kinemat dobot-ros` is empty). That is the first gap.

---

## Design

### One interface, two backends

A policy never imports `DobotRosClient` or `requests`. It talks to two narrow
protocols, and the same policy source runs unchanged against the simulator and
the arm:

```python
class Robot(Protocol):                      # motion + gripper
    def pose(self) -> Pose6                 # [X,Y,Z,RX,RY,RZ] mm/deg, active tool frame
    def joints(self) -> list[float]
    def move_l(self, pose: Pose6, speed: int | None = None) -> None   # blocking
    def move_j(self, joints: list[float]) -> None
    def gripper(self, pos_0_1000: int, force: int = 20) -> int        # returns final jaw pos
    def gripper_pos(self) -> int
    def set_tool(self, index: int) -> None
    def stop(self) -> None

class World(Protocol):                      # perception
    def objects(self) -> list[WorldObject]  # label, color, centroid_mm (robot frame), size_mm, yaw_deg, confidence
    def table_z(self) -> float
    def find(self, **query) -> WorldObject | None   # find(color="red", label="block")
```

Backends:

- **`sim/`** — `SimRobot` + `SimWorld` (kinematic simulator, below).
- **`real/`** — `RealRobot` wraps `DobotRosClient` and runs every target through the
  existing guards (floor, workspace, vertical lock, motion lock, confirm broker).
  `RealWorld` wraps `VisionClient` + `VisionTransform` (pixel → table-plane
  projection, exactly as `/api/vision/plan` does today) and adds color naming.

### Policies

```
dobot-ros/dobot_ros/policies/
├─ base.py          # Policy ABC, PolicyResult, Trace, PolicyAbort
├─ primitives.py    # the mid-level library every policy composes
├─ registry.py      # discover policies/*.py, params JSON per slug (mirrors strategies/)
├─ params/          # <slug>.json
└─ pick_colored_block.py   # the first one
```

```python
class Policy(ABC):
    name, slug, description: str
    @classmethod
    def parameter_defs(cls) -> list[ParameterDef]: ...
    @abstractmethod
    def run(self, robot: Robot, world: World, params: dict, trace: Trace) -> PolicyResult: ...
```

`PolicyResult` is HomeBody's `SkillResult`: `ok`, `reason`, `attempts`,
`elapsed_s`, `observations` (e.g. final jaw position). `Trace` records every
primitive call with args, the robot/world state after it, and wall time — the
single artifact that the test suite asserts on, the CLI prints, and the web
viewer replays.

**Primitives** (`primitives.py`) are where the robotics knowledge goes, so
policies stay short and readable:

| Primitive | Does |
|---|---|
| `go_home(robot)` | Safe start pose above the workspace |
| `hover_above(robot, xy, h_mm, yaw)` | Vertical approach pose, tool-aware Z via the table plane |
| `descend_to(robot, h_mm)` | Straight-down MovL to a height above the table, floor-guarded |
| `grasp(robot, width_hint_mm)` | Open to width + margin, close, read jaw: fully closed ⇒ `GraspMissed` |
| `lift(robot, h_mm)` | Retract vertically |
| `place_at(robot, xy, h_mm)` | Hover → descend → open → lift |
| `look(world, **query)` | Re-query perception; raises `NotFound` with what *was* seen |

The first policy, in full, is about fifteen lines:

```python
def run(self, robot, world, p, trace):
    robot.set_tool(1)
    obj = look(world, color=p["color"], label="block")
    for attempt in range(p["max_attempts"]):
        hover_above(robot, obj.xy, p["approach_mm"], obj.yaw_deg)
        descend_to(robot, p["grasp_height_mm"])
        try:
            grasp(robot, width_hint_mm=obj.size_mm[0])
        except GraspMissed:
            lift(robot, p["approach_mm"]); obj = look(world, color=p["color"], label="block")
            continue
        lift(robot, p["lift_mm"])
        return PolicyResult(ok=True, attempts=attempt + 1)
    return PolicyResult(ok=False, reason="grasp missed after retries")
```

### The simulator

**Tier 1 — kinematic simulator (build this).** No physics engine. Fast, headless,
deterministic, runs in pytest in milliseconds, and good enough to catch the bugs
hand-coded policies actually have: wrong frame, wrong sign, wrong tool offset,
descending into the table, reaching outside the workspace, closing on nothing,
forgetting to open before descending, IK-unreachable poses.

```
dobot-ros/dobot_ros/sim/
├─ kinematics.py    # CR5 FK from URDF origins; damped-least-squares IK; joint limits; Dobot Euler convention
├─ world.py         # SimWorld: table from table_plane.json, WorldObject list, scene JSON load/save/randomize
├─ robot.py         # SimRobot: implements Robot; interpolates MovL/MovJ; gripper model; guards
├─ grasp.py         # attach rule: object between fingers, width within stroke, fingertip within object Z span
├─ collision.py     # fingertip/tool vs table plane and object AABBs; floor guard; workspace bounds
├─ noise.py         # perception noise model: XY σ, Z σ (±38 mm today), false-negative rate, yaw σ
└─ scenes/          # two_blocks.json, cluttered.json, …
```

Rules of the sim, all of which mirror a real failure mode:

- `move_l` interpolates in cartesian space and runs the floor guard and workspace
  check on every interpolated sample, raising the same exception class the real
  adapter raises. A policy that would hit the table in reality fails in sim.
- The gripper is a stroke (0–1000 ↔ 0–105 mm). On close, if a free object's OBB
  lies between the fingertips and fits the stroke, it becomes **attached** to the
  tool frame and moves with it; otherwise the jaw closes fully. That is precisely
  the grasp-verification signal HomeBody relies on, so policies learn to check it.
- On open, an attached object detaches and drops to the table plane (or the top of
  whatever is beneath it, for stacking).
- `SimWorld.objects()` returns ground truth **plus** the noise model. The default
  noise is the measured D555 behaviour, so a policy that only works with perfect
  perception fails in sim as it would on the arm.
- Every call appends to the `Trace` with joints, tool pose, gripper, and object
  poses, so the three.js viewer can replay a run frame by frame.

**Tier 2 — MuJoCo (later, same interface).** When a question needs contact
physics (does the block slip, does the stack topple, does a 20 % force hold it),
`MujocoRobot`/`MujocoWorld` implement the same two protocols. MuJoCo loads the
vendor URDF + STL meshes directly and runs headless on the arm host's T1000.
This is HOMEBODY.md's "Tier B" and is explicitly out of scope for the first
release.

### Web integration — a **Policies** tab

| Area | Behaviour |
|---|---|
| Policy picker + params form | From the registry, rendered from `parameter_defs()` exactly as the Strategies card does today |
| Target: **Sim / Robot** | Sim is the default and needs no robot. Robot requires driver connected, robot enabled, and goes through the motion lock and the confirm broker (`step` mode, as every pick does today) |
| Scene panel (sim only) | Load a scene preset, **Randomize** (seeded), edit object color/size/position in a small table, save as a new preset. Scene objects render in the 3D view on the table plane |
| **Run** | Streams `policy_trace` events over `/ws/state` as the policy executes; the 3D view follows live (sim) or shows the real joints (robot). Primitive log on the right with args, result, elapsed |
| **Replay** | Any saved trace (sim or real) plays back in the viewer with a scrubber. Real runs record the same `Trace`, so sim and real traces of the same policy sit side by side |
| **Batch** (sim only) | Run N seeds, show success rate, failure reasons histogram, mean attempts. This is the debugging instrument: fix, re-run 50 seeds, watch the number move |
| Source view | Read-only view of the policy's Python with the current line highlighted during a run. Editing happens in the repo; `dobot-ros/` is volume-mounted, so a saved file is live on the next run with no rebuild |

API surface, following the existing naming:

```
GET  /api/policies/list                 GET  /api/sim/scenes
GET  /api/policies/{slug}/params        GET/POST /api/sim/scene          (current scene, edit)
POST /api/policies/{slug}/params        POST /api/sim/scene/randomize    {seed}
POST /api/policies/run                  {slug, target: "sim"|"robot", scene?, seed?}
POST /api/policies/stop
POST /api/policies/batch                {slug, n, seed0}  → per-seed PolicyResult
GET  /api/policies/traces  ·  GET /api/policies/traces/{id}
```

The existing `POST /api/inverse-kin` grows a fallback: when the driver is not
connected, answer from `sim/kinematics.py`. That makes the current Vision
"Simulate" button work with the robot powered off for free.

### CLI, for the inner loop

```
dobot-ros policy list
dobot-ros policy run pick_colored_block --sim --scene two_blocks --seed 3 --color red
dobot-ros policy batch pick_colored_block --sim --n 50
dobot-ros policy run pick_colored_block --robot        # confirm modal in the web UI, as today
```

`--sim` runs entirely in-process with no ROS, so it also runs on a laptop.

---

## Build phases

| Phase | Deliverable | Effort | Blocked on |
|---|---|---|---|
| **0 Kinematics** | `sim/kinematics.py` FK + IK + joint limits, Euler convention pinned down. Validation script records (joints, pose) pairs from the live feed and asserts FK agrees to < 1 mm / 0.1°. `/api/inverse-kin` offline fallback | 1 day | Robot on for one 5-minute capture |
| **1 Simulator** | `sim/` world, robot, grasp, collision, noise, trace; scene presets; `tests/sim/` | 2–3 days | Phase 0 |
| **2 Policies** | `policies/` base, primitives, registry, `pick_colored_block`; `real/` adapters; `tests/policies/` runs the policy over seeded scenes and asserts ≥ 95 % success at default noise | 1–2 days | Phase 1 |
| **3 Web tab + API** | Policies tab, scene panel, live trace stream, replay, batch, 3D scene objects + attached-object rendering | 2–3 days | Phase 2 |
| **4 First real run** | `pick_colored_block` on the arm under `step` confirm mode; sim vs real trace comparison; tune noise model from the measured error | ½ day | Robot, a red block, color naming (risk 2) |
| **5 Second policy** | `place_at` + `stack_blocks` or `sort_by_color` to prove the primitives generalize | 1 day | Phase 4 |
| **6 MuJoCo backend** *(optional)* | Same protocols, contact physics | 1 week | Only if a question needs physics |

Phases 1–3 need no robot. Roughly two weeks part-time to a policy that was
written, debugged in sim, and then picked a real block.

## Testing

| Layer | Test |
|---|---|
| Kinematics | FK vs recorded live pairs; IK(FK(q)) round-trips; unreachable poses raise |
| Sim grasp/collision | Table hit raises `FloorViolation`; close on air ⇒ jaw 0; close on 40 mm block ⇒ attached, jaw ≈ 40 mm; open ⇒ drops to support |
| Noise | Seeded, reproducible; zero-noise mode for debugging the policy logic alone |
| Policies | Batch over seeds at default noise ≥ 95 %; at zero noise 100 %; the trace contains the expected primitive sequence |
| Real adapter | Mock `DobotRosClient` asserts every target passed the guards and that a confirm was requested before the first motion |
| Web | API tests against the sim target; no robot in CI |

## Risks and what to do about them

1. **Dobot's Euler convention.** The controller reports RX/RY/RZ in a specific
   rotation order. Get it wrong and FK is "almost right", which is worse than
   wrong. Phase 0 pins it empirically against live data before anything is built
   on top.
2. **"Red" is not in the detection payload today.** `DetectedObject` carries a
   label but the camera server is external and its color output is unverified.
   Fix is small either way: sample mean HSV inside the detection bbox from
   `/api/frame/color` and map to a color name in `vision.py`. Only the real
   adapter needs it; the sim has ground truth.
3. **IK branch mismatch.** The sim's joint path may differ from the controller's
   own solution. Real execution uses cartesian MovL, so the waypoints are what
   matter and they are identical; joint display is cosmetic. Reachability
   disagreements near limits are the residual risk and are caught on the arm.
4. **Kinematic sim cannot see slip or contact.** A sim pass is a logic check, not
   a safety certificate. Real runs keep every guard and the confirm modal.
   Tier 2 MuJoCo is the answer if physics starts to matter.
5. **Camera depth is ±38 mm.** Already handled by projecting onto the table
   plane; the sim noise model copies that so policies never trust Z from vision.
