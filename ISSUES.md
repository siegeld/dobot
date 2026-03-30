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
