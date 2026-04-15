# Servo tester tests

Offline tests for `dobot_ros.servo`. No robot, no GPU, no network.

```bash
# From repo root, inside the dobot container:
pytest tests/servo -v
```

Or bare-metal:

```bash
PYTHONPATH=dobot-ros pytest tests/servo -v
```

## Coverage

| File | Covers |
|------|--------|
| `test_patterns.py` | Circle / Lissajous / Square / Sine math, dispatch, duration handling |
| `test_tester.py` | Hold-pose streaming, live jog offset, pattern integration, workspace clamps, emergency stop (calls `ros.stop()`), live config update, double-start idempotency |
