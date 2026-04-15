# VLA — Vision-Language-Action Integration

This document describes the OpenVLA-OFT integration plan for the Dobot CR5. It captures architecture, rationale, data pipeline, fine-tune workflow, and how the pieces slot into the existing ROS2 stack.

For the base robot stack see [CLAUDE.md](CLAUDE.md) and [README.md](README.md).

---

## Goal

Drive the CR5 from natural-language instructions + a single RGB frame, using a fine-tuned **OpenVLA-OFT** model. Collect demonstrations via the CR5's existing drag-teach mode, convert to RLDS, fine-tune on a local RTX Pro 6000 Blackwell (96 GB), and execute actions on the robot through the driver's `ServoP` streaming-servo interface.

---

## Model: OpenVLA-OFT

**Why OFT over base OpenVLA:**
- 26× faster inference, 3× lower latency.
- +20% success rate on standard benchmarks.
- Emits **action chunks** (8–25 actions per forward pass) via parallel decoding — no per-step autoregressive overhead.
- Supports multi-view (scene + optional wrist camera).
- Same RLDS data format as base OpenVLA — no cost to choosing OFT over base.

**Inputs:** RGB image (224×224 after internal resize), natural-language instruction, optional proprioceptive state.
**Outputs:** Continuous 7-D action chunk — 6 DoF end-effector delta (Δx, Δy, Δz, Δrx, Δry, Δrz) + gripper command.

**Why not vLLM / SGLang / TGI:** OpenVLA-OFT cannot be served on vLLM-class inference engines:
- Custom Prismatic DINOv2 + SigLIP fused vision backbone — not a registered model class.
- Continuous L1-regression action head — not a next-token-generation path.
- Parallel chunk decoding — incompatible with autoregressive KV-cache decoders.
Run OFT using its native PyTorch inference path wrapped in a thin FastAPI server.

**Why not zero-shot OpenVLA:** CR5 is not in the Open X-Embodiment training mix. The model must be fine-tuned on CR5-specific demonstrations before it will produce usable actions.

---

## Hardware

| Host | GPU | Role |
|------|-----|------|
| **gpurbr1** | RTX Pro 6000 Blackwell 96 GB | Existing. Runs the vLLM-based `llm-server` for Siegel Machine / OpenClaw. Not touched by VLA work. |
| **(new, on order)** | RTX Pro 6000 Blackwell 96 GB | Dedicated to VLA: OFT inference + fine-tune jobs. |

96 GB comfortably handles OFT inference (~16–18 GB), LoRA fine-tune (27–72 GB), and full fine-tune. No cloud rental needed.

---

## Deployment model: standalone, not integrated with llm-server

OFT runs as its own docker-compose service on the second Blackwell. It is **not** a slot inside the existing `/srv/docker/llm-server` manager on gpurbr1, because:
- llm-server is a vLLM manager — OFT can't run on vLLM (see above).
- OFT's operational shape (checkpoint hot-swap, non-OpenAI-chat API, per-episode state) isn't known yet. Bolting unknowns into a working production system adds risk.
- The second Blackwell is new capacity — isolated dev environment costs nothing.

Reused from llm-server: shared `llm-net` Docker bridge network (so DCGM metrics visualize both cards from one UI), Dockerfile conventions, image structure.

Revisit integration into llm-server after the pipeline is stable (post-Phase 4 below).

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│ Dobot host (existing)                                                  │
│                                                                        │
│  dobot-ros container (Docker)                                          │
│   ┌──────────────────────────────────────────────────────────────┐     │
│   │  VLA Executor (new ROS2 node)                                │     │
│   │   - subscribes: camera frame (HTTP proxy), /joint_states,    │     │
│   │                 ToolVectorActual, /gripper/state             │     │
│   │   - POSTs: image + instruction + state → OFT server          │     │
│   │   - streams: ServoP at ~20-50 Hz, gripper actions on change  │     │
│   └──────────────────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────────────────┐     │
│   │  Episode Recorder (new)                                      │     │
│   │   - captures synced (RGB, EE pose, joints, gripper) @ 10 Hz  │     │
│   │   - writes episodes/<name>/ with steps.jsonl + frames/       │     │
│   └──────────────────────────────────────────────────────────────┘     │
│   ┌──────────────────────────────────────────────────────────────┐     │
│   │  dobot_bringup_v4 (C++ driver, unchanged)                    │     │
│   │   - exposes ServoP / ServoJ / MovJ / gripper action          │     │
│   └──────────────────────────────────────────────────────────────┘     │
│                                                                        │
│  Camera server (existing, on 10.11.6.65:8080)                          │
│   - /api/frame/color  →  RGB JPEG                                      │
└────────────────────────┬───────────────────────────────────────────────┘
                         │  HTTP
                         ▼
┌────────────────────────────────────────────────────────────────────────┐
│ OFT inference host (second Blackwell)                                  │
│                                                                        │
│  oft-server container                                                  │
│   - FastAPI /predict, /health, /reload                                 │
│   - loads OFT checkpoint (bf16 on Blackwell, optional 4-bit)           │
│   - joins shared llm-net bridge                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Control path: `ServoP`, not `MovJ`

**Critical design decision.** OFT emits action chunks at ~10 Hz intended control rate, and we want to upsample to 20–50 Hz for smooth motion. `MovJ` (point-to-point with trajectory planning) stutters and queues at that rate. `ServoP` is the Dobot's streaming-servo interface:

- Accepts absolute cartesian pose + time budget `t`.
- `t` range [0.004, 3600.0] s → up to 250 Hz theoretical.
- `gain` (P term) and `aheadtime` (D term) tune responsiveness.
- Expects **monotonic streaming** — pausing halts the controller.

The executor:
1. Reads current EE pose.
2. Queries OFT with `(image, instruction, state)`.
3. Receives action chunk (8 deltas say).
4. Optionally linearly interpolates to 3–5× the model rate.
5. Converts each interpolated delta to an absolute target pose.
6. Streams via `ServoP(x, y, z, rx, ry, rz, t=period)` on a fixed-rate loop.
7. Issues gripper action only when the gripper command crosses a threshold (Modbus is ~5 Hz max).
8. Re-queries the model before the chunk runs out; overlaps inference with execution.

Safety clamps applied at step 5: enforce workspace bounds from `table_plane.json` and max per-step motion.

---

## Data pipeline

### Episode format (on-disk, pre-RLDS)

```
episodes/
└─ <instruction_slug>_<timestamp>/
   ├─ meta.json           # instruction, dates, robot config, camera pose, calib refs
   ├─ steps.jsonl         # one JSON line per step (see below)
   └─ frames/
      ├─ 000000.jpg
      ├─ 000001.jpg
      └─ ...
```

**`steps.jsonl` schema per line:**
```json
{
  "step": 0,
  "t": 1713190812.345,
  "image_path": "frames/000000.jpg",
  "state": {
    "ee_pose": [X, Y, Z, RX, RY, RZ],       // mm, deg
    "joints":  [J1, J2, J3, J4, J5, J6],    // deg
    "gripper_pos": 1000,                    // 0–1000
    "gripper_state": 1                      // DH AG-105 state code
  },
  "action": {
    "ee_delta": [dX, dY, dZ, dRX, dRY, dRZ], // delta to next state
    "gripper_target": 1000                   // absolute target for next step
  },
  "is_first": true,
  "is_last": false,
  "is_terminal": false
}
```

Recorded at 10 Hz. Action at step _i_ = (state at step _i+1_) − (state at step _i_). Last step sets `is_last = is_terminal = true` and zero action.

### RLDS conversion

The converter (`dobot_ros/vla/rlds_builder.py`) reads one or more episode directories and produces a TFDS-compatible RLDS dataset using the `kpertsch/rlds_dataset_builder` pattern. Feature schema:

| Field | Shape / dtype | Source |
|-------|---------------|--------|
| `steps/observation/image` | (H, W, 3) uint8 | `frames/*.jpg` |
| `steps/observation/wrist_image` | (H, W, 3) uint8 *(optional)* | reserved for future wrist cam |
| `steps/observation/state` | (7,) float32 | `[X, Y, Z, RX, RY, RZ, gripper_pos/1000]` |
| `steps/action` | (7,) float32 | `[dX, dY, dZ, dRX, dRY, dRZ, gripper_target/1000]` |
| `steps/language_instruction` | string | replicated per step |
| `steps/is_first`, `is_last`, `is_terminal` | bool | |
| `steps/reward`, `discount` | float32 | 0.0 except terminal |
| `episode_metadata/file_path` | string | |

To fine-tune OFT: register this dataset in `prismatic/vla/datasets/rlds/oxe/configs.py` and `transforms.py` in the OFT repo, then run the provided fine-tune script.

### Demo count targets

- **Smoke test:** 10–20 episodes of one coarse task. Goal: prove the pipeline end-to-end, not achieve good behavior.
- **Per-task fine-tune:** ~100–500 episodes. OFT's published numbers are typically in this range.
- **Multi-task:** several tasks × ~100 each, plus a language prefix so the model learns task-conditioning.

---

## Fine-tune workflow

1. Collect N episodes via drag-teach + foot pedal (see "Teleop" below).
2. `dobot-ros vla convert-rlds` → RLDS dataset on disk.
3. Copy dataset to OFT repo checkout on the second Blackwell.
4. Register in `configs.py` / `transforms.py`.
5. Run OFT LoRA fine-tune (overnight, ~hours).
6. `scp` checkpoint to the inference container's model directory.
7. `POST /reload` on the OFT server to hot-swap.
8. Execute on the robot, observe, iterate.

---

## Teleop for data collection

Minimum viable teleop: **drag mode (existing) + USB foot pedal** for gripper.

- Drag mode: `client.start_drag()` puts the CR5 in mode 6 (BACKDRIVE). Operator moves the arm by hand.
- Foot pedal: any USB HID pedal mapped to key presses. One pedal → open, two pedals → open/close/middle.
- Both hands free for arm manipulation; foot commits gripper transitions.

**Upgrade path** (only if demo quality becomes the bottleneck):
- 3Dconnexion SpaceMouse driving `jog()` at 20–50 Hz. Cleaner, more consistent trajectories than kinesthetic drag.
- Phone/VR teleop for 6-DoF end-effector control.

---

## Camera

Existing RealSense camera server at `http://10.11.6.65:8080`, `/api/frame/color` returns an RGB JPEG. OFT takes only RGB — depth is ignored — so the existing stream is sufficient.

**Non-negotiable:** camera pose must match between training and deployment. Moving the camera mid-project invalidates the dataset. Record the camera mounting pose in every episode's `meta.json` for traceability.

Wrist camera: not required. Consider adding only if precise-task success plateaus. If added, it must be present from day one of a given dataset (demos can't be retroactively re-viewed).

---

## Build phases

| Phase | Goal | Can start |
|-------|------|-----------|
| **1. Inference plumbing** | OFT + mock servers, HTTP client, basic bench. | Now |
| **2. Data collection** | Episode recorder + teleop loop + RLDS converter. | Now |
| **3. Fine-tune** | Register dataset, run LoRA fine-tune. | After 2nd Blackwell arrives + ≥50 demos |
| **4. Closed-loop execution** | ServoP-based executor, web VLA tab. | After Phase 1 + 2 skeletons, tested against mock |
| **5. Ops** | Checkpoint swap, metrics, failure dashboards. | After Phase 4 |

Phases 1, 2, and 4 (against the mock server) can run in parallel before the second Blackwell lands.

---

## Code layout

```
dobot/
├─ VLA.md                                 # this doc
├─ oft-server/                            # (new) OFT inference container
│  ├─ Dockerfile
│  ├─ docker-compose.yml
│  ├─ requirements.txt
│  ├─ server.py                           # real OFT FastAPI server
│  ├─ mock_server.py                      # random-action mock for offline dev
│  └─ README.md
└─ dobot-ros/dobot_ros/
   ├─ ros_client.py                       # (modified) adds servo_p()
   ├─ cli.py                              # (modified) adds `vla` subgroup
   ├─ web/server.py                       # (modified) adds /api/vla/* endpoints
   ├─ web/static/js/app.js                # (modified) adds VLA tab
   └─ vla/                                # (new)
      ├─ __init__.py
      ├─ client.py                        # HTTP client for OFT server
      ├─ recorder.py                      # episode recorder
      ├─ executor.py                      # closed-loop VLA execution via ServoP
      ├─ rlds_builder.py                  # episode dir → RLDS TFDS dataset
      ├─ safety.py                        # workspace bounds, rate limits
      └─ types.py                         # shared dataclasses
```

---

## Testing strategy

Everything that doesn't require the robot or a real GPU is testable offline:

| Component | Test approach |
|-----------|---------------|
| Episode recorder | Mock camera + mock ROS client; assert on-disk schema. |
| RLDS builder | Tiny fixture dataset; load back via `tfds.load()`; check shapes. |
| OFT client | Start `mock_server.py`; hit `/predict`; parse response. |
| Executor | Mock OFT client (returns scripted chunks); mock ServoP; assert pose trajectory + safety clamping. |
| Safety bounds | Feed out-of-workspace actions; assert clamp behavior. |
| ServoP method | Integration test: requires real robot; part of the manual bring-up checklist. |

All offline tests run via `pytest` inside the dobot container.

---

## Open questions / known gaps

- **ServoP tuning.** Defaults (gain=500, aheadtime=50) are starting point. Need empirical sweep on real robot once executor lands.
- **Action scaling.** OFT normalizes actions per dataset; the `unnorm_key` used at inference must match training. Recorder should emit raw scales; the OFT server de-normalizes.
- **Inference↔execution overlap.** First version queries model between chunks (stall during inference). Later: double-buffer so the next chunk is ready before the current one drains.
- **Multi-GPU.** With 96 GB we may be able to run inference + fine-tune on the same card; default is `CUDA_VISIBLE_DEVICES=0` for inference, queue fine-tunes separately.
- **Workspace safety.** `table_plane.json` already has floor and XY bounds. VLA executor must enforce them **before** each ServoP call, not trust the model.
