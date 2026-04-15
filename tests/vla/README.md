# VLA pipeline tests

Offline tests for the VLA integration. None of them require the real robot,
a GPU, or network access.

## Running

```bash
# From repo root, inside the dobot container (has rclpy + dependencies):
./dobot-shell.sh
pytest tests/vla -v
```

Or bare-metal (most tests work; the `test_executor.py` imports need
`dobot_ros` on PYTHONPATH):

```bash
PYTHONPATH=dobot-ros pytest tests/vla -v
```

## What's covered

| File | Covers |
|------|--------|
| `test_recorder.py` | On-disk episode schema, action computation, start/stop lifecycle |
| `test_safety.py` | Per-step delta caps, workspace clamps, combined pipeline |
| `test_rlds_builder.py` | Load-episode roundtrip, numpy shapes, gripper normalization, iter skipping |
| `test_executor.py` | Closed-loop with mock OFT: ServoP streaming, upsampling, gripper thresholding, clean stop |
| `test_mock_server.py` | Mock OFT server HTTP surface (health, predict, reload) |

## Not covered (requires real hardware)

- Real ServoP calls to the Dobot driver — covered by manual bring-up checklist.
- Real OFT inference on a GPU — covered by the `oft-server` README smoke test.
- End-to-end demo → fine-tune → execute — covered by the phase plan in VLA.md.
