# Known Issues

## High Priority

### 1. Debug prints left in production code
**Files:** `dobot_cr/robot.py:211-214`, `dobot_cr/robot.py:306-313`

`[DEBUG]` prints to stderr fire on every position read and every `move_joints` call. Should be removed or moved behind `logging.debug()`.

### 2. Tight feedback polling loop
**File:** `dobot_cr/robot.py:84-97`

`_feedback_loop` runs with no sleep — busy loop that maxes out a CPU core. Add a small `time.sleep(0.008)` (matching the ~8ms Dobot feedback cycle) or use a blocking read.

### 3. `.env` tracked in git
`.env` is in `.gitignore` but is already tracked in the repo. Should be removed from tracking:
```bash
git rm --cached .env
```

### 4. ~~`dobot_ros_config.yaml` is a directory, not a file~~ (FIXED)
Fixed: replaced directory with file, removed broken mount from docker-compose.yml.

## Medium Priority

### 5. Placeholder author and URLs in pyproject.toml
`pyproject.toml` still has `"Your Name"`, `your.email@example.com`, and `yourusername` in all URLs. Same placeholder URLs appear in `README.md` support section.

### 6. Version mismatch
`dobot_cr/__init__.py` says `0.0.1`, `pyproject.toml` says `0.1.0`.

### 7. Hardcoded robot IP in version-controlled config
`dobot_config.yaml` has `ip: "10.11.6.68"` (site-specific). For redistribution, should be a placeholder like `192.168.1.6` with instructions to override via `dobot_config.local.yaml`.

### 8. Fragile sys.path.insert for vendor SDK
**File:** `dobot_cr/robot.py:12`
```python
sys.path.insert(0, str(Path(__file__).parent.parent / "TCP-IP-Python-V4"))
```
Breaks if the package is installed from a different location. Consider adding the SDK as a proper dependency or documenting the requirement.

### 9. Commented-out code in shell
**File:** `dobot_cr/shell.py:402-413`

`RequestControl` block is commented out with notes about corrupting feedback data. Should be resolved or documented as a known limitation, not left as dead code.

### 10. No tests
`pyproject.toml` references a `tests/` directory that doesn't exist. Unit tests for `Config` and `Position` would be straightforward to add.

## Low Priority / Cleanup

### 11. Duplicated table rendering
`cmd_position` in `shell.py` duplicates the table-rendering logic from `cli.py`. Could share a formatting function.

### 12. Dual dependency declarations
Both `requirements.txt` and `pyproject.toml` declare dependencies — potential for drift.

### ~~13. Stale roadmap~~ (FIXED)
Updated: "ROS 2 integration" and "web-based dashboard" now marked as complete in README.md roadmap.

### 14. No py.typed marker
No `py.typed` file for downstream mypy consumers.

---

## Safety Audit (2026-04-16)

Three audit passes. 58 bugs found and fixed. 29 remaining (tracked below).

Previously fixed: see commits `e24668a` (32 fixes) and `7d7421e` (26 fixes).

### CRITICAL — could cause robot crash or injury

| # | Status | File | Issue |
|---|--------|------|-------|
| S-C1 | **OPEN** | ros_client.py | `wait_for_cartesian_motion` ignores RX/RY/RZ — gripper closes while wrist is still rotating during pick |
| S-C2 | **OPEN** | server.py | `pick_execute` and `vision_execute` are `async def` that block the event loop 30-60s — E-STOP unresponsive. Fix: change to `def` so FastAPI uses threadpool. |
| S-C3 | **OPEN** | server.py | `_get_client()` not thread-safe — concurrent init creates duplicate ROS nodes |
| S-C4 | **OPEN** | server.py | VLA executor, servo tester, and pick can run simultaneously — competing motion streams |
| S-C5 | **OPEN** | server.py + driver.py | Driver watchdog auto-restart (documented in CLAUDE.md) not implemented |
| S-C6 | **OPEN** | robot3d.js | `simulationTick` never sets `simulating=false` — `simOnComplete` fires ~18× |

### HIGH — could cause incorrect motion

| # | Status | File | Issue |
|---|--------|------|-------|
| S-H1 | **OPEN** | ros_client.py | `_validate_pose_bounds` Z allows negative values (e.g., Z=-500 passes) |
| S-H2 | **OPEN** | server.py | `MoveJointsRequest.angles` is bare `list` — no type enforcement |
| S-H3 | **OPEN** | vla/safety.py | `clamp_delta`/`clamp_pose` don't check NaN/Inf |
| S-H4 | **OPEN** | vla/executor.py | `stop()` reports success even if join times out — old thread still streaming |
| S-H5 | DEFERRED | ros_client.py | Tool-frame jog `wait=True` uses user-frame math for target |
| S-H6 | DEFERRED | server.py | Workspace move uses straight diagonal path, not L-shaped |

### MEDIUM — correctness / reliability

| # | Status | File | Issue |
|---|--------|------|-------|
| S-M1 | DEFERRED | server.py | Settings import no schema validation |
| S-M2 | **OPEN** | ros_client.py | `get_position()` reads joint/cartesian from separate lock acquisitions |
| S-M3 | **OPEN** | server.py | `calibration_record` no px/py bounds validation |
| S-M4 | **OPEN** | servo/tester.py | `update_config` uses `setattr` without type coercion |
| S-M5 | **OPEN** | strategies | Angled approach at (0,0) uses silent fallback direction |
| S-M6 | **OPEN** | server.py | `workspace_move` fire-and-forget jog — no wait-for-completion |
| S-M7 | **OPEN** | settings_store.py | `_flush()` holds lock during file I/O |
| S-M8 | **OPEN** | server.py | `_TABLE_FILE` non-atomic reads race with writes |
| S-M9 | **OPEN** | servo/tester.py | CSV open failure silent — user thinks logging is active |
| S-M10 | DEFERRED | server.py | No rate limiting on motion commands |

### LOW — cosmetic / edge cases

| # | Status | File | Issue |
|---|--------|------|-------|
| S-L1 | **OPEN** | ros_client.py | ServoP `if t != -1` guards are dead code |
| S-L2 | **OPEN** | vla/executor.py | Gripper threshold no debounce |
| S-L3 | **OPEN** | servo/patterns.py | Negative amplitudes not clamped |
| S-L4 | DEFERRED | app.js | `logActivity` innerHTML XSS |
| S-L5 | DEFERRED | settings_store.py | Orphaned temp files on kill |
| S-L6 | DEFERRED | server.py | `_settings_store` created at import time |

### Test coverage gaps

| # | Missing test |
|---|--------------|
| S-T1 | Concurrent executor + servo tester operation |
| S-T2 | `wait_for_cartesian_motion` with rotation |
| S-T3 | `_get_client()` concurrent initialization |
| S-T4 | `MockRosClient.stop()` not implemented in mock |
| S-T5 | Strategy `plan()` with NaN rotation_deg |
| S-T6 | Pick failure recovery (safe state after collision) |
