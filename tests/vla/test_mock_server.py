"""Test the mock OFT server end-to-end via FastAPI TestClient.

Confirms the wire protocol the executor relies on without needing a container.
"""

from __future__ import annotations

import base64
import importlib.util
import io
from pathlib import Path

import pytest
from PIL import Image


MOCK_SERVER_PATH = Path(__file__).resolve().parents[2] / "oft-server" / "mock_server.py"


def _load_mock_app():
    spec = importlib.util.spec_from_file_location("oft_mock_server", MOCK_SERVER_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore
    return mod.app


def _tiny_jpeg_b64():
    img = Image.new("RGB", (32, 24), color=(200, 50, 50))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


@pytest.fixture
def client():
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    from fastapi.testclient import TestClient
    return TestClient(_load_mock_app())


def test_health(client):
    r = client.get("/health")
    assert r.status_code == 200
    data = r.json()
    assert data["mode"] == "mock"


def test_predict_shape(client):
    r = client.post("/predict", json={
        "image_b64": _tiny_jpeg_b64(),
        "instruction": "pick the red block",
        "state": [0, 0, 0, 0, 0, 0, 0.5],
    })
    assert r.status_code == 200
    data = r.json()
    assert len(data["actions"]) == data["chunk_size"]
    assert all(len(a) == 7 for a in data["actions"])
    assert data["latency_ms"] >= 0


def test_reload_is_noop(client):
    r = client.post("/reload", json={"checkpoint": "/fake/path"})
    assert r.status_code == 200
    assert r.json()["status"] == "ok"
