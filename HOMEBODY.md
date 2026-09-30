# HOMEBODY — Explore, Remember, Act on the CR5

A plan to reproduce the method of **HomeBody: A Humanoid That Explores, Remembers, and Acts on Its Own** (Stanford TML / Caltech — Gio Huh, Cayden Gu, Takara E. Truong, C. Karen Liu, Guy Tevet) on a fixed 6-DoF arm instead of a humanoid.

Project page: <https://tml.stanford.edu/homebody/> · Repo: <https://github.com/Stanford-TML/homebody> (**"code coming soon"** — nothing released as of 2026-09-30, so this is a re-implementation from the method description, not a port).

For the learned-policy track see [VLA.md](VLA.md). For the existing pick stack see [VISION.md](VISION.md). For the base robot stack see [CLAUDE.md](CLAUDE.md).

---

## What HomeBody actually is

**It trains nothing.** There is no learned task policy, no demonstration dataset, no fine-tune. A frontier VLM ("GPT Astra") orchestrates a hand-written, composable skill library, and all pretrained components (perception, locomotion) are used off the shelf.

The contribution is the **grounding**. Three stages:

| Stage | What happens |
|-------|--------------|
| **1. Explore** | The robot walks an unseen environment collecting multimodal data: iPhone video at 0.5×, D435i stereo observations, LiDAR scans with SLAM, joint poses, and waypoints the VLM itself chose. |
| **2. Real2Sim** | A digital twin is reconstructed in Isaac Sim **from the robot's own collected data**, fusing visual, geometric, and semantic information into one metric scene. |
| **3. Act** | The VLM plans against that persistent 3D scene — not against single RGB frames — emitting calls into the skill library and receiving execution results that inform the next decision. |

Supporting cast: **SAM 2** segmentation, **Fast-FoundationStereo** depth, **SAMURAI** (memory-selective visual tracking), **AMO** for upper-body-aware locomotion, **Super Odometry** for localization. Hardware: Unitree G1, single Razer Blade with an RTX 4090 — all local except the VLM.

Five skills share one interface: `Navigate`, `Pick`, `Place`, `OpenDrawer`, `PickFromDrawer`. Each does **local retries with visual feedback**, so a recoverable failure never costs a VLM replan.

### Why this is the cheap track for us

| | VLA ([VLA.md](VLA.md)) | HomeBody (this doc) |
|---|---|---|
| Learns a policy | Yes — OpenVLA-OFT fine-tune | **No** |
| Needs demonstrations | 100–500 episodes/task | None |
| Needs the 2nd Blackwell | Yes, to train | No (perception fits a smaller card) |
| Generalizes to new objects via | More data | The VLM's priors |
| Fails at | Anything off-distribution | Anything the skill library can't express |
| Blocked on | Hardware on order + weeks of teleop | Nothing |

They are complementary, not competing, and they can share one evaluation suite — see [Head-to-head](#head-to-head-vla-vs-homebody).

---

## Translating to a fixed arm

| HomeBody | CR5 version |
|---|---|
| Explore an unseen kitchen (LiDAR SLAM + Super Odometry) | Scan the workspace from N viewpoints. **Forward kinematics replaces SLAM** — the EE pose gives exact camera extrinsics per frame, so our localization is strictly better than theirs and free |
| Real2Sim twin in Isaac Sim | TSDF fusion + object-level scene memory (Tier A); MuJoCo twin only for collision-checked rehearsal (Tier B) |
| `Navigate(2D waypoint)` | **`LookAt(region)`** — the "change your viewpoint" skill |
| `Pick` / `Place` / `OpenDrawer` / `PickFromDrawer` | Essentially unchanged |
| AMO whole-body loco-manipulation | **Dropped.** This is what we lose, and it isn't the idea |
| SAMURAI cross-frame tracking | SAM 2 video mode over the scan sequence — the need becomes re-identification across viewpoints and after disturbance |

### The problem we must earn back

Our camera is **static**, mounted oblique in front of the table. One fixed viewpoint means there is no exploration and no memory problem: the whole scene sits in one frame, a single-shot VLM baseline ties the full system, and the thesis evaporates. Two changes make the problem real, and **both are required**:

1. **A wrist-mounted RGB-D camera** — makes viewpoint a *choice*, enables next-best-view, and as a side effect kills the depth-noise ceiling described below.
2. **Occluding fixtures in the workspace** — a drawer, a shelf, a cabinet box. Objects the arm must *remember* because it cannot currently see them.

Without (2), "memory" is decoration. Without (1), "exploration" is a fiction.

---

## Hardware

### What exists

| Item | Detail |
|---|---|
| Arm | Dobot CR5, 6-DoF, base frame origin at base center |
| Gripper | DH AG-105, 105 mm stroke, 203 mm flange-to-fingertip (`Tool(1)`) |
| Camera | **Intel RealSense D555**, serial 409122301759, fw 7.56 — **DDS/Ethernet at 10.11.6.69**, not USB |
| Camera server | External service at `http://10.11.6.65:8080`, 640×360 @ 25 fps, source not in this repo |
| Workspace | From `table_plane.json`: X ∈ [−676.7, −271.4], Y ∈ [−678.9, +85.7] mm → **405 × 765 mm**, plane tilt 0.6°, mean table Z 28.3 mm |
| Compute (arm host) | `jupiter`, NVIDIA T1000 8 GB |
| Compute (GPU) | `gpurbr1` RTX Pro 6000 Blackwell 96 GB; a second Blackwell on order for VLA |

The camera server already exposes exactly what reconstruction needs — no new capture code against the sensor:

```
GET /api/frame/color        GET /api/frame/depth/raw     GET /api/pointcloud
GET /api/frame/depth        GET /api/depth/region        GET /api/pointcloud/vertices
GET /api/depth/at           GET /api/detection/objects   POST /api/snapshot
```

### What to add

**Wrist camera — the single highest-leverage purchase in the project.**

| Option | Verdict |
|---|---|
| **RealSense D405** (~$250, USB-C) | **Recommended.** Short-range stereo, ~1–2 mm accuracy at 20 cm vs the D555's ±38 mm at our working distance. Small and light on the flange |
| Second D555 on the wrist | One PoE cable is cleaner than USB, but it is bulky, long-baseline, and wrong for 20 cm work |
| Reuse the static D555 only | Kills the project (see above) |

Caveat: the CR5 flange exposes 485/IO, **not USB** — a wrist USB camera needs a cable run along the arm in a spiral wrap or drag chain, and the joint-6 rotation limit must be respected in the scan trajectory so the cable never winds.

**Occlusion fixture.** A small two-drawer unit or open cabinet inside the 405 × 765 mm workspace, rigidly mounted (its pose becomes part of the twin) with a handle the AG-105 can actually grasp. Everything interesting — `OpenDrawer`, `PickFromDrawer`, occluded retrieval, memory ablations — depends on this existing.

**Calibration target.** A ChArUco board for eye-in-hand calibration.

---

## Stage 1 — Explore

### Eye-in-hand calibration (new, and a prerequisite for everything)

The existing 4-point SVD/Kabsch solve ([VISION.md](VISION.md)) fixes a **static** camera to the robot base. A wrist camera needs the other transform: `T_flange→camera`, solved from the `AX = XB` formulation.

1. Clamp the ChArUco board anywhere in view; it does not move.
2. Drive the arm to 15–25 poses that see the board from varied positions **and orientations** (rotation diversity is what conditions the solve — pure translation is degenerate).
3. At each pose record the flange pose from FK and the board pose from `cv2.solvePnP`.
4. Solve with `cv2.calibrateHandEye` (Tsai or Park); report residual reprojection error.
5. Store as `hand_eye.json` beside `table_plane.json`. **Re-run whenever the camera is unmounted** — and record its hash in every scan's metadata, exactly as [VLA.md](VLA.md) requires for the static camera pose.

Target residual: < 2 mm translational. If it won't converge, the usual cause is too little rotation diversity, not bad code.

### Scan trajectory

Derive viewpoints from the known workspace rather than hand-teaching them:

- Sample a partial hemisphere above the workspace centroid at 250–400 mm standoff, camera aimed at the surface, elevation 30–75°, ~12–20 poses.
- Reject any pose failing IK, the floor guard, joint limits, or the joint-6 cable-wind constraint — reuse `/api/inverse-kin` and the existing workspace bounds.
- Add explicit inspection poses per fixture (look *into* the open drawer, look *behind* tall objects).
- Move `MovJ` between viewpoints, settle, then capture — do **not** capture in motion; rolling-shutter and latency will corrupt the pose association that FK is giving us for free.

### Capture

At each viewpoint record color JPEG, raw depth, camera intrinsics, EE pose, joint angles, gripper state, and a UTC stamp. The format is `vla/recorder.py`'s episode layout (`frames/NNNNNN.jpg` + `steps.jsonl` + `meta.json`) with depth added — reuse the recorder rather than writing a second capture path.

Active exploration comes later and cheaply: once a scan exists, the planner can request additional `LookAt` viewpoints to resolve low-confidence or occluded regions. That is HomeBody's VLM-chosen-waypoints loop, reduced to a fixed base.

---

## Stage 2 — Remember

### Tier A — object-centric scene memory (start here)

The twin that matters is **a queryable object graph, not a pretty mesh.** A text planner cannot reason over a point cloud; it reasons over labelled objects with metric positions and relations.

Pipeline, run offline after a scan (seconds-to-minutes, not real time):

1. **Fuse** — TSDF integrate the (depth, camera pose) stream via Open3D at 2–4 mm voxels. FK-exact extrinsics mean no pose-graph optimization is needed.
2. **Segment** — SAM 2 over the color frames, video mode across the scan so masks carry identity between viewpoints.
3. **Label** — open-vocabulary detection (OWLv2 or CLIP crops) per mask, plus a VLM pass over a few canonical crops for names and affordances the detector's vocabulary misses.
4. **Lift** — project each mask into the fused volume; fit an oriented bounding box; derive grasp candidates from the OBB's short axes filtered by the AG-105's 105 mm stroke.
5. **Relate** — derive `support` and `containment` geometrically: resting on the table plane, inside a fixture's volume, stacked on another object.
6. **Persist** — write `scene_memory.json`.

```jsonc
{
  "scan_id": "2026-09-30T14:02:11Z",
  "frame": "robot_base_mm",
  "hand_eye_hash": "sha256:…",
  "table_plane": { "normal": [0.0089, 0.0054, 0.99995], "d": -22.53 },
  "fixtures": [
    {
      "id": "drawer_1", "label": "drawer", "state": "closed",
      "obb": { "center": [-480, -600, 120], "extent": [300, 200, 150], "yaw_deg": 0.0 },
      "handle": { "centroid_mm": [-480, -498, 150], "grasp_axis_deg": 90.0 },
      "interior_volume": { "center": [-480, -600, 110], "extent": [280, 180, 90] }
    }
  ],
  "objects": [
    {
      "id": "obj_003", "label": "mug", "aliases": ["cup"], "label_confidence": 0.91,
      "centroid_mm": [-412.6, -203.4, 68.2],
      "obb": { "extent": [82, 80, 95], "yaw_deg": 12.4 },
      "grasps": [ { "axis_deg": 102.4, "width_mm": 80, "quality": 0.78 } ],
      "support": "table",
      "visibility": "visible",
      "last_seen_utc": "2026-09-30T14:02:19Z",
      "seen_from_views": [3, 4, 9],
      "provenance": { "mask_ids": [11, 14], "detector": "owlv2", "vlm_confirmed": true }
    },
    {
      "id": "obj_007", "label": "spoon",
      "centroid_mm": [-486.1, -594.8, 105.0],
      "support": "in:drawer_1",
      "visibility": "occluded",
      "last_seen_utc": "2026-09-30T13:58:02Z"
    }
  ]
}
```

`visibility: occluded` with a live `centroid_mm` **is the memory claim**, in one field: the system asserts where something is that it cannot presently see. Everything in the evaluation section exists to test whether that assertion holds up.

### Tier B — MuJoCo twin (only when Tier A works)

**MuJoCo, not Isaac Sim**, for three reasons: the arm host's T1000 8 GB cannot run Isaac Sim; we train nothing, so photorealistic rendering buys nothing; and `cr5_moveit/config/cr5_robot.urdf.xacro` already ships in `DOBOT_6Axis_ROS2_V4/`, so the robot model is minutes of work, not days.

Use it for collision-checked motion planning against reconstructed geometry (convex-decomposed meshes) and to rehearse a plan before it touches hardware. Isaac Sim only earns its cost if we later want synthetic data — which this method does not need.

### Where perception runs

A separate HTTP service, following the precedent `oft-server/` already set — **not** bolted into the external camera server, whose source isn't even in this repo.

```
POST /scan/fuse      → fuse a captured scan into a TSDF volume
POST /scan/segment   → SAM 2 + open-vocab labels over a scan
POST /scan/build     → full pipeline, returns scene_memory.json
GET  /healthz
```

Host it on `gpurbr1` (or the second Blackwell once it lands). Ship a `mock_server.py` returning a canned `scene_memory.json` so the planner and skills can be developed with no GPU and no robot — the same trick that makes the VLA executor testable offline.

---

## Stage 3 — Act

### Skills vs. strategies

The existing `PickStrategy` ABC (`strategies/base.py`) answers *how to pick once you know where* — approach heights, grasp axis, MovL vs ServoP. That is deliberately narrow and should stay so.

A **Skill** is one unit of work the planner composes. `Pick` is a skill; it *delegates* to the strategy registry. So: a new `Skill` ABC parallel to `PickStrategy`, not a widening of it.

| Skill | Signature | Notes |
|---|---|---|
| `LookAt` | `(region \| object_id)` → updated memory | Replaces `Navigate`. Moves to a viewpoint, captures, re-fuses, patches `scene_memory.json` |
| `Pick` | `(object_id, strategy?)` → grasped bool | Resolves the object from memory, builds a `PickContext`, delegates to a strategy |
| `Place` | `(target: object_id \| fixture_id \| xyz)` → placed bool | Release-height from the support surface, not from camera depth |
| `OpenDrawer` | `(fixture_id)` → open bool | Grasp handle, pull along the fixture's normal, release |
| `PickFromDrawer` | `(object_id)` → grasped bool | Composite: `OpenDrawer` → `LookAt(interior)` → `Pick`. The occluded-retrieval case |

### The result contract

HomeBody's "execution results inform subsequent decisions" is this one dataclass:

```python
@dataclass
class SkillResult:
    ok: bool
    skill: str
    args: dict
    reason: str | None          # "gripper closed fully — likely missed"
    observations: dict          # post-condition evidence
    memory_delta: dict          # objects moved/appeared/disappeared
    attempts: int
    elapsed_s: float
```

`memory_delta` is what keeps the twin honest across a long horizon: a successful `Pick` moves an object to `in:gripper`, a `Place` re-parents it, a failed `Pick` downgrades its `visibility` and forces a `LookAt` before the planner retries.

### Retries inside the skill

The point of local retries is that the VLM is never consulted for a recoverable failure. On the CR5, cheap and effective:

- **Grasp verification** without tactile sensing: read the AG-105 jaw position after closing. Fully closed ⇒ nothing between the fingers. This is free and catches most misses.
- **Re-detect and re-grasp** — a second `LookAt` at the target, refreshed centroid, next-best grasp axis from the OBB. Cap at 2–3 attempts.
- **Nudge** — perturb approach XY by a few mm on a repeat miss; depth and calibration residuals are systematic, not random.

Escalate to the planner only when the retry budget is spent, and say *why* in `reason`.

### The planner

A provider interface with two backends, chosen per run:

- **Frontier API** (Claude or GPT) — matches the paper's capability level. This is what a published comparison should use.
- **Local VLM** on `gpurbr1`'s vLLM — cheaper and offline-capable for iteration, and a legitimate ablation in its own right ("does this need a frontier model?").

Input: `scene_memory.json` rendered as compact text, the task instruction, the skill catalog with signatures, and the `SkillResult` history. Output: one skill call at a time — not a full plan up front. Single-step emission is how execution results actually feed back in; a batch plan can only be revised by discarding it.

### Confirmation, honestly

Today every pick raises a modal through `pick_confirm.py`. A 12-step autonomous plan with a modal per pick is unusable, and quietly removing the guard on a 5 kg arm is not acceptable either. Proposed:

| Mode | Behavior |
|---|---|
| `step` | Current behavior. Every motion confirmed. Default for bring-up |
| `plan` | Approve the planner's stated intent once; steps auto-confirm; a persistent abort stops motion immediately |
| `auto` | No prompts. Only inside a defined workspace with a fixture-only scene. For timed evaluation runs |

Independent of mode, keep the floor guard, workspace bounds, and `lock_vertical` enforced in code — the planner is never trusted with safety, exactly as [VLA.md](VLA.md) insists for the OFT executor.

---

## Evaluation

A reproduction becomes a result at the ablation table. Metrics per episode: task success, subtask success, planner calls, skill attempts vs. successes, wall-clock, and a failure taxonomy (perception / grasp / planner / memory-stale).

### Task suite

| Task | Tests |
|---|---|
| **Sort N objects** — "put the utensils in the drawer and the mug on the tray" | Long-horizon composition, 6–10 objects, unseen arrangements |
| **Occluded retrieval** — target inside a closed drawer or behind a tall object | Whether memory substitutes for sight |
| **Displace-to-reach** — target only reachable after moving an occluder | Whether the planner reasons about its own future viewpoints |
| **Stale memory** — object is moved by hand *after* the scan, before execution | Does the system detect and recover, or confidently grasp air? |

### Ablations

1. **No memory** — single static frame + VLM, no scan, no twin. **The central experiment.** If this matches the full system, the reconstruction isn't earning its keep, and we learn that for the cost of one config flag rather than three months.
2. **No retries** — skills fail hard to the planner. Isolates how much the local feedback loops contribute.
3. **No planner** — scripted task order. Isolates what the VLM contributes over a fixed script.
4. **Local VLM vs. frontier** — does this need a frontier model?
5. **Wrist camera vs. static only** — quantifies the hardware claim, and retroactively justifies (or refutes) the purchase.

### Head-to-head: VLA vs. HomeBody

Same tasks, same robot, same workspace: a fine-tuned OpenVLA-OFT policy against VLM-orchestrated skills. Learned policy vs. structured planning on one arm is a genuinely interesting comparison that this repo is unusually well positioned to run, because both tracks already live here. Worth designing the task suite now so it serves both.

---

## Build phases

| Phase | Goal | Blocked on |
|---|---|---|
| **0. Fixtures** | Mount the drawer unit; re-run table-plane calibration | Fixture purchase |
| **1. Wrist camera** | Mount D405, cable run, eye-in-hand calibration, < 2 mm residual | D405 + ChArUco board |
| **2. Scan → TSDF** | Scan trajectory generator, depth capture, Open3D fusion, mesh out | Phase 1 |
| **3. Scene memory** | Perception service (SAM 2 + OWLv2) + `scene_memory.json` + mock | Phase 2 for real data; mock unblocks everything else |
| **4. Skills** | `Skill` ABC, `SkillResult`, `LookAt`/`Pick`/`Place`, retries | Phase 3 mock |
| **5. Planner** | Provider interface, both backends, single-step loop, confirm modes | Phase 4 |
| **6. Drawer skills** | `OpenDrawer`, `PickFromDrawer` | Phases 0 + 4 |
| **7. Evaluation** | Task suite, ablation harness, metrics, runs | Phases 5 + 6 |
| **8. MuJoCo twin** *(optional)* | CR5 xacro + reconstructed collision geometry, plan rehearsal | Phase 2 |

Phases 3–5 can be built against the mock perception service with no GPU and no robot. Phase 1 is worth doing first regardless of this project: it independently fixes the ±38 mm depth ceiling that currently forces every pick to discard camera Z and fall back to `table_plane.json`.

Rough effort: 6–8 weeks part-time to a credible result with ablations.

---

## Code layout

```
dobot/
├─ HOMEBODY.md                             # this doc
├─ perception-server/                      # (new) SAM 2 + OWLv2 + TSDF service
│  ├─ Dockerfile · docker-compose.yml · requirements.txt
│  ├─ server.py                            # real service (GPU)
│  ├─ mock_server.py                       # canned scene_memory.json, no GPU
│  └─ README.md
└─ dobot-ros/dobot_ros/
   ├─ web/server.py                        # (modified) /api/scan/*, /api/homebody/*
   ├─ web/static/js/app.js                 # (modified) Scene tab
   ├─ web/hand_eye.json                    # (new) eye-in-hand extrinsics
   ├─ scan/                                # (new) exploration + reconstruction
   │  ├─ trajectory.py                     # viewpoint generation, IK/limit filtering
   │  ├─ capture.py                        # RGB-D + pose capture (wraps vla/recorder)
   │  ├─ hand_eye.py                       # ChArUco collection + calibrateHandEye
   │  └─ client.py                         # HTTP client for perception-server
   ├─ memory/                              # (new) the twin
   │  ├─ schema.py                         # SceneMemory, SceneObject, Fixture
   │  ├─ store.py                          # load/save/patch, memory_delta application
   │  └─ query.py                           # resolve("the red mug") → object_id
   ├─ skills/                              # (new) composable skill library
   │  ├─ base.py                           # Skill ABC, SkillResult, retry harness
   │  ├─ look_at.py · pick.py · place.py
   │  ├─ open_drawer.py · pick_from_drawer.py
   │  └─ registry.py
   └─ planner/                             # (new) VLM orchestration
      ├─ provider.py                       # frontier API | local vLLM
      ├─ prompt.py                         # memory + catalog + history → messages
      ├─ loop.py                           # single-step emit → execute → feed back
      └─ transcript.py                     # per-episode log for evaluation
```

Note `skills/` sits beside, not inside, `strategies/` — a strategy is *how to pick*, a skill is *a unit of work the planner composes*.

---

## Testing strategy

Everything except the arm itself is testable offline, following the pattern [VLA.md](VLA.md) already established:

| Component | Test approach |
|---|---|
| Scan trajectory | Assert every generated pose passes IK, floor guard, joint and cable-wind limits; assert workspace coverage |
| Hand-eye solve | Synthetic board poses through a known `T_flange→camera`; assert recovery to < 0.5 mm |
| TSDF fusion | Fixture scan of a known object; assert OBB extents within tolerance of caliper measurements |
| Scene memory | Fixture `scene_memory.json`; assert schema, relation derivation, and `memory_delta` application |
| Memory query | `resolve()` against fixture scenes including ambiguous and absent referents |
| Skills | Mock ROS client + mock perception; assert waypoints, retry counts, `SkillResult` contents |
| Planner loop | Scripted provider returning fixed skill calls; assert the loop terminates, feeds results back, and respects the retry budget |
| Ablation harness | Run the full suite against mocks; assert per-condition metrics are produced |

`plan()` is already pure in the strategy layer; keep skill planning pure the same way and the whole stack above the driver stays unit-testable.

---

## Open questions and known gaps

- **Wait for their code?** The repo says "code coming soon." Phases 0–2 (fixture, wrist camera, hand-eye, scan) are pure infrastructure and are not wasted whatever ships. The memory schema and skill interfaces are where a release could save real work — worth re-checking the repo before starting Phase 3.
- **No force/torque sensing.** The CR5 has collision detection but no wrist F/T sensor, and the AG-105 has no tactile feedback. `OpenDrawer` therefore cannot be compliant in the usual sense — the first version pulls along a calibrated axis and relies on collision thresholds to abort. A drawer with meaningful stiction may simply not be openable this way; measure before designing around it.
- **Depth on the static D555 is the current ceiling.** ±38 mm at working distance, which is why [VISION.md](VISION.md) projects rays onto the table plane instead of trusting depth. Multi-view fusion from a wrist D405 should collapse this, but the fused-volume accuracy needs measuring against calipers before any Z is trusted.
- **Camera server is external and unversioned here.** Source lives on another host, not in this repo. Anything we need from it should be pulled through its existing HTTP API, and new perception belongs in `perception-server/`.
- **D555 currently reports `connected: false`.** Worth resolving before any scan work; the DDS transport is a different failure surface than USB.
- **640×360 is low for SAM 2 mask quality** on small objects like utensils. Check whether the D555 can stream higher for capture, or whether the wrist camera carries segmentation alone.
- **Object identity across a long horizon.** Two identical mugs are the case that breaks naive re-identification. Position-based tracking through `memory_delta` handles it while the robot is the only mover; a human reaching in breaks it. That is exactly what the stale-memory task is for.
- **Confirm-mode safety review.** `plan` and `auto` modes need a deliberate sign-off before first use, not a default flip. See [Confirmation, honestly](#confirmation-honestly).
- **Planner cost.** A long-horizon episode with per-step VLM calls plus vision is not free. Log token usage per episode in the transcript from day one so the frontier-vs-local ablation has real numbers.
