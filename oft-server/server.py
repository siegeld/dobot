"""OpenVLA-OFT inference server (real model).

Hosts a fine-tuned OpenVLA-OFT checkpoint, exposes /predict, /health, /reload.
The OFT model loader is deliberately wrapped behind `_ModelBackend` so the
server can be imported and unit-tested without the heavy deps installed.
"""

import base64
import io
import logging
import os
import threading
import time
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel, Field


log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


# ── Config ──────────────────────────────────────────────────────────────
CHECKPOINT = os.environ.get("OFT_CHECKPOINT", "/data/checkpoints/latest")
DEVICE = os.environ.get("OFT_DEVICE", "cuda:0")
DTYPE = os.environ.get("OFT_DTYPE", "bfloat16")
LOAD_IN_4BIT = os.environ.get("OFT_LOAD_IN_4BIT", "0") == "1"
CHUNK_SIZE = int(os.environ.get("OFT_CHUNK_SIZE", 8))


# ── Schemas ─────────────────────────────────────────────────────────────
class PredictRequest(BaseModel):
    image_b64: str
    wrist_image_b64: Optional[str] = None
    instruction: str
    state: Optional[List[float]] = Field(default=None, description="Proprioceptive state (7-D: EE pose + gripper)")
    unnorm_key: Optional[str] = None


class PredictResponse(BaseModel):
    actions: List[List[float]]
    chunk_size: int
    latency_ms: float
    model: str


class ReloadRequest(BaseModel):
    checkpoint: str


# ── Model backend ──────────────────────────────────────────────────────
# Wrapping the real OFT load/predict in a class keeps the server importable
# even when the heavy OFT deps are not installed (e.g. in CI/tests).
class _ModelBackend:
    def __init__(self, checkpoint: str):
        self.checkpoint = checkpoint
        self.model = None
        self.processor = None
        self.lock = threading.Lock()
        self.load()

    def _torch_dtype(self):
        import torch
        return {"bfloat16": torch.bfloat16, "float16": torch.float16, "float32": torch.float32}[DTYPE]

    def load(self):
        log.info("Loading OFT checkpoint from %s (device=%s dtype=%s 4bit=%s)",
                 self.checkpoint, DEVICE, DTYPE, LOAD_IN_4BIT)
        import torch
        from transformers import AutoModelForVision2Seq, AutoProcessor

        kwargs = dict(
            torch_dtype=self._torch_dtype(),
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        )
        if LOAD_IN_4BIT:
            from transformers import BitsAndBytesConfig
            kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=self._torch_dtype(),
            )

        self.processor = AutoProcessor.from_pretrained(self.checkpoint, trust_remote_code=True)
        self.model = AutoModelForVision2Seq.from_pretrained(self.checkpoint, **kwargs).to(DEVICE)
        self.model.eval()
        log.info("Checkpoint loaded")

    def reload(self, checkpoint: str):
        with self.lock:
            old = self.model
            self.checkpoint = checkpoint
            self.load()
            del old

    def predict(self, image: Image.Image, instruction: str,
                wrist_image: Optional[Image.Image],
                state: Optional[List[float]],
                unnorm_key: Optional[str]) -> List[List[float]]:
        import torch

        with self.lock:
            # OFT processors commonly accept dict-style observation with keys
            # "full_image" + optional "wrist_image" + optional "state". Exact
            # processor API depends on the OFT checkpoint; adapt here.
            obs = {"full_image": image}
            if wrist_image is not None:
                obs["wrist_image"] = wrist_image
            if state is not None:
                obs["state"] = torch.tensor(state, dtype=self._torch_dtype(), device=DEVICE)

            prompt = f"In: What action should the robot take to {instruction.lower()}?\nOut:"
            inputs = self.processor(prompt, obs, return_tensors="pt").to(DEVICE)

            with torch.inference_mode():
                # OFT's predict_action returns a chunk of continuous actions.
                # The exact method name depends on the checkpoint — we try the
                # common ones in order.
                actions = None
                for fn_name in ("predict_action_chunk", "predict_action", "generate"):
                    fn = getattr(self.model, fn_name, None)
                    if fn is None:
                        continue
                    out = fn(**inputs, unnorm_key=unnorm_key) if unnorm_key else fn(**inputs)
                    actions = out
                    break
                if actions is None:
                    raise RuntimeError("No compatible predict method on OFT model")

            # Normalize to python list of lists.
            if hasattr(actions, "tolist"):
                actions = actions.tolist()
            if isinstance(actions, (list, tuple)) and actions and isinstance(actions[0], (int, float)):
                actions = [list(actions)]  # single action → wrap as 1-chunk
            return [[float(v) for v in a] for a in actions]


# ── App ────────────────────────────────────────────────────────────────
app = FastAPI(title="oft-server", version="0.1.0")
_backend: Optional[_ModelBackend] = None


def _get_backend() -> _ModelBackend:
    global _backend
    if _backend is None:
        _backend = _ModelBackend(CHECKPOINT)
    return _backend


def _decode_image(b64: str) -> Image.Image:
    try:
        data = base64.b64decode(b64, validate=True)
        return Image.open(io.BytesIO(data)).convert("RGB")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"bad image: {e}")


@app.on_event("startup")
def _startup():
    # Eagerly load so first request isn't slow. Safe to fail — /health still works.
    try:
        _get_backend()
    except Exception as e:
        log.exception("startup model load failed (server will keep running): %s", e)


@app.get("/health")
def health():
    return {
        "status": "ok",
        "mode": "real",
        "model": _backend.checkpoint if _backend else CHECKPOINT,
        "loaded": _backend is not None and _backend.model is not None,
        "device": DEVICE,
        "dtype": DTYPE,
        "chunk_size": CHUNK_SIZE,
    }


@app.post("/reload")
def reload(req: ReloadRequest):
    backend = _get_backend()
    backend.reload(req.checkpoint)
    return {"status": "ok", "checkpoint": req.checkpoint}


@app.post("/predict", response_model=PredictResponse)
def predict(req: PredictRequest) -> PredictResponse:
    backend = _get_backend()
    image = _decode_image(req.image_b64)
    wrist = _decode_image(req.wrist_image_b64) if req.wrist_image_b64 else None
    t0 = time.perf_counter()
    actions = backend.predict(image, req.instruction, wrist, req.state, req.unnorm_key)
    latency_ms = (time.perf_counter() - t0) * 1000.0
    return PredictResponse(
        actions=actions,
        chunk_size=len(actions),
        latency_ms=latency_ms,
        model=backend.checkpoint,
    )
