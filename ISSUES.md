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

Three audit passes completed 2026-04-16. 87 total issues found, 81 fixed, 6 remaining.

Previously fixed: `e24668a` (32), `7d7421e` (26), `5297a23` (10), `pending` (13).

### CRITICAL — could cause robot crash or injury

| # | Status | File | Issue |
|---|--------|------|-------|
| S-C1 | FIXED | ros_client.py | `wait_for_cartesian_motion` now checks RX/RY/RZ with angle wrapping |
| S-C2 | FIXED | server.py | Pick/vision execute + plan + workspace_move changed to `def` (threadpool) |
| S-C3 | FIXED | server.py | `_get_client()` uses double-checked locking with `_client_init_lock` |
| S-C4 | FIXED | server.py | Global `_check_motion_free()` guard blocks competing motion streams |
| S-C5 | DEFERRED | server.py + driver.py | Driver watchdog auto-restart not implemented |
| S-C6 | FIXED | robot3d.js | `simulationTick` calls `stopSimulation()` on completion |

### HIGH — could cause incorrect motion

| # | Status | File | Issue |
|---|--------|------|-------|
| S-H1 | FIXED | ros_client.py | Z floor at MIN_Z=-200mm in `_validate_pose_bounds` |
| S-H2 | FIXED | server.py | `MoveJointsRequest`, `InverseKinRequest`, `ServoTargetRequest` use `List[float]` |
| S-H3 | FIXED | vla/safety.py | `clamp_delta`/`clamp_pose` check `isfinite()` first |
| S-H4 | FIXED | vla/executor.py | `stop()` checks `thread.is_alive()` after join; keeps `running=True` if stuck |
| S-H5 | DEFERRED | ros_client.py | Tool-frame jog `wait=True` uses user-frame math for target |
| S-H6 | DEFERRED | server.py | Workspace move uses straight diagonal path, not L-shaped |

### MEDIUM — correctness / reliability

| # | Status | File | Issue |
|---|--------|------|-------|
| S-M1 | DEFERRED | server.py | Settings import no schema validation |
| S-M2 | FIXED | ros_client.py | `get_position()` reads both under single lock acquisition |
| S-M3 | FIXED | server.py | `calibration_record` validates px/py bounds [0, 1280]×[0, 720] |
| S-M4 | FIXED | servo/tester.py | `update_config` validates field names + type-coerces values |
| S-M5 | DEFERRED | strategies | Angled approach at (0,0) uses silent fallback direction |
| S-M6 | FIXED | server.py | `workspace_move` now calls `jog(wait=True, timeout=15)` |
| S-M7 | FIXED | settings_store.py | `_flush()` correctness restored (lock held during write) |
| S-M8 | FIXED | server.py | All `_TABLE_FILE` writes now use `_atomic_write()` |
| S-M9 | FIXED | servo/tester.py | CSV open failure sets `last_error` in status |
| S-M10 | DEFERRED | server.py | No rate limiting on motion commands |

### LOW — cosmetic / edge cases

| # | Status | File | Issue |
|---|--------|------|-------|
| S-L1 | FIXED | ros_client.py | Removed dead `if t != -1` guards in ServoP/ServoJ |
| S-L2 | FIXED | vla/executor.py | Gripper commands debounced (0.5s minimum between commands) |
| S-L3 | FIXED | servo/patterns.py | Negative amplitudes abs()'d before clamping |
| S-L4 | DEFERRED | app.js | `logActivity` innerHTML XSS |
| S-L5 | DEFERRED | settings_store.py | Orphaned temp files on kill |
| S-L6 | DEFERRED | server.py | `_settings_store` created at import time |

### Test coverage gaps

| # | Status | Missing test |
|---|--------|--------------|
| S-T1 | OPEN | Concurrent executor + servo tester operation |
| S-T2 | OPEN | `wait_for_cartesian_motion` with rotation |
| S-T3 | OPEN | `_get_client()` concurrent initialization |
| S-T4 | FIXED | `MockRosClient.stop()` added to conftest |
| S-T5 | OPEN | Strategy `plan()` with NaN rotation_deg |
| S-T6 | OPEN | Pick failure recovery (safe state after collision) |
