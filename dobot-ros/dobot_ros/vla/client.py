"""HTTP client for the OFT inference server.

Works against both the real server and the mock. The executor talks to this.
"""

from __future__ import annotations

import base64
import io
import logging
from dataclasses import dataclass
from typing import List, Optional

import requests
from PIL import Image


log = logging.getLogger(__name__)

DEFAULT_URL = "http://gpurbr2:7071"  # tentative hostname for the 2nd Blackwell


@dataclass
class VLAPrediction:
    actions: List[List[float]]
    chunk_size: int
    latency_ms: float
    model: str


class OFTClient:
    """Thin HTTP client for oft-server."""

    def __init__(self, base_url: str = DEFAULT_URL, timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    # ── Image helpers ────────────────────────────────────────────
    @staticmethod
    def encode_image(img: Image.Image, fmt: str = "JPEG", quality: int = 90) -> str:
        buf = io.BytesIO()
        img.save(buf, format=fmt, quality=quality)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    @staticmethod
    def encode_jpeg_bytes(jpeg_bytes: bytes) -> str:
        return base64.b64encode(jpeg_bytes).decode("ascii")

    # ── API ──────────────────────────────────────────────────────
    def health(self) -> dict:
        r = requests.get(f"{self.base_url}/health", timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def reload(self, checkpoint: str) -> dict:
        r = requests.post(
            f"{self.base_url}/reload",
            json={"checkpoint": checkpoint},
            timeout=30.0,
        )
        r.raise_for_status()
        return r.json()

    def predict(
        self,
        image_b64: str,
        instruction: str,
        state: Optional[List[float]] = None,
        wrist_image_b64: Optional[str] = None,
        unnorm_key: Optional[str] = None,
    ) -> VLAPrediction:
        payload = {
            "image_b64": image_b64,
            "instruction": instruction,
        }
        if state is not None:
            payload["state"] = state
        if wrist_image_b64 is not None:
            payload["wrist_image_b64"] = wrist_image_b64
        if unnorm_key is not None:
            payload["unnorm_key"] = unnorm_key

        r = requests.post(f"{self.base_url}/predict", json=payload, timeout=self.timeout)
        r.raise_for_status()
        data = r.json()
        return VLAPrediction(
            actions=data["actions"],
            chunk_size=data["chunk_size"],
            latency_ms=data["latency_ms"],
            model=data.get("model", "unknown"),
        )
