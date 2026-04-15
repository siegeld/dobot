# oft-server

OpenVLA-OFT inference server. Hosts the fine-tuned checkpoint, accepts RGB + instruction + state over HTTP, returns an action chunk. Deployed on the second RTX Pro 6000 Blackwell.

Runs as a standalone docker-compose stack. Not integrated with the gpurbr1 vLLM manager (`/srv/docker/llm-server`) — OFT is incompatible with vLLM; see [../VLA.md](../VLA.md).

## Quick start

```bash
# Mock mode — no GPU required, returns random action chunks.
# Use for developing the robot-side executor.
OFT_MODE=mock docker compose up

# Real mode — requires GPU, checkpoint at $OFT_CHECKPOINT.
OFT_MODE=real OFT_CHECKPOINT=/data/checkpoints/oft-cr5-v1 docker compose up
```

Server listens on port **7071**.

## API

### `POST /predict`

Request:
```json
{
  "image_b64": "<base64 JPEG>",
  "wrist_image_b64": "<optional base64 JPEG>",
  "instruction": "pick up the red block",
  "state": [x, y, z, rx, ry, rz, gripper_norm],
  "unnorm_key": "dobot_cr5"
}
```

Response:
```json
{
  "actions": [[dx, dy, dz, drx, dry, drz, gripper], ...],
  "chunk_size": 8,
  "latency_ms": 42.1,
  "model": "openvla-oft-cr5-v1"
}
```

### `GET /health`

```json
{"status": "ok", "mode": "mock|real", "model": "...", "device": "cuda:0"}
```

### `POST /reload`

Body: `{"checkpoint": "/path/to/new/checkpoint"}`. Hot-swaps the model without restarting the container.

## Environment variables

| Var | Default | Description |
|-----|---------|-------------|
| `OFT_MODE` | `mock` | `mock` or `real` |
| `OFT_CHECKPOINT` | `/data/checkpoints/latest` | Path inside container |
| `OFT_DEVICE` | `cuda:0` | CUDA device |
| `OFT_DTYPE` | `bfloat16` | Inference dtype (bfloat16/float16/float32) |
| `OFT_LOAD_IN_4BIT` | `0` | Set `1` to quantize |
| `OFT_CHUNK_SIZE` | `8` | Action chunk size |
| `OFT_PORT` | `7071` | Bind port |

## GPU + Blackwell notes

- Requires **CUDA 12.8+** and a PyTorch build with SM 120 support.
- Power draw on a Pro 6000 Blackwell under load is up to 600 W — verify PSU and breaker.
- `docker-compose.yml` reserves the NVIDIA runtime; override `device_ids` if sharing with other workloads.
