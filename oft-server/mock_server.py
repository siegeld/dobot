"""Mock OFT inference server.

Returns random small action chunks. Used to develop the robot-side executor
without needing a GPU or a real checkpoint.
"""

import base64
import os
import random
import time
from typing import List, Optional

from fastapi import FastAPI
from pydantic import BaseModel, Field


CHUNK_SIZE = int(os.environ.get("OFT_CHUNK_SIZE", 8))


class PredictRequest(BaseModel):
    image_b64: str
    wrist_image_b64: Optional[str] = None
    instruction: str
    state: Optional[List[float]] = None
    unnorm_key: Optional[str] = None


class PredictResponse(BaseModel):
    actions: List[List[float]]
    chunk_size: int
    latency_ms: float
    model: str


app = FastAPI(title="oft-server (mock)", version="0.1.0")


@app.get("/health")
def health():
    return {
        "status": "ok",
        "mode": "mock",
        "model": "random-actions",
        "device": "cpu",
        "chunk_size": CHUNK_SIZE,
    }


@app.post("/reload")
def reload(body: dict):
    return {"status": "ok", "checkpoint": body.get("checkpoint", "mock"), "mode": "mock"}


@app.post("/predict", response_model=PredictResponse)
def predict(req: PredictRequest) -> PredictResponse:
    t0 = time.perf_counter()
    # Sanity-check the image decodes; we don't actually use it.
    try:
        base64.b64decode(req.image_b64, validate=True)
    except Exception:
        # In mock mode we tolerate junk, but report it via a small stall.
        time.sleep(0.001)

    # Generate a small smooth-ish action chunk. Deltas in approximately the
    # scales the real model would emit post-unnormalization: mm/step for xyz,
    # deg/step for rx/ry/rz, unit-interval for gripper.
    actions: List[List[float]] = []
    for _ in range(CHUNK_SIZE):
        actions.append([
            random.uniform(-2.0, 2.0),   # dx mm
            random.uniform(-2.0, 2.0),   # dy mm
            random.uniform(-2.0, 2.0),   # dz mm
            random.uniform(-0.5, 0.5),   # drx deg
            random.uniform(-0.5, 0.5),   # dry deg
            random.uniform(-0.5, 0.5),   # drz deg
            random.uniform(0.0, 1.0),    # gripper target, 0..1
        ])

    latency_ms = (time.perf_counter() - t0) * 1000.0
    return PredictResponse(
        actions=actions,
        chunk_size=CHUNK_SIZE,
        latency_ms=latency_ms,
        model="mock-random",
    )
