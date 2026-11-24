"""FastAPI router for hand-coded policies + the simulator.

Registered from server.py via ``register(app, deps)``; the deps closure
gives this module the server's singletons (ROS client, confirm broker,
websocket broadcast, motion lock, settings) without importing server.py.
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from dobot_ros.policies.guards import Guards, WorkspaceBounds
from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.runner import PolicyRunner, RunRequest
from dobot_ros.policies.types import PolicyAbort
from dobot_ros.sim import kinematics as K
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.scene import SCENES_DIR, list_scenes, load_scene, save_scene
from dobot_ros.sim.world import SimWorld, load_table_plane

log = logging.getLogger(__name__)

router = APIRouter()
_registry: Optional[PolicyRegistry] = None
_runner: Optional[PolicyRunner] = None
_deps: Dict[str, Any] = {}
_motion_held = threading.Event()

TRACES_DIR = Path(__file__).resolve().parent.parent / "policies" / "traces"


# ── registration ────────────────────────────────────────────────────

def register(app, **deps) -> None:
    """deps: get_client, pick_confirm, broadcast, acquire_motion, release_motion,
    settings_store, get_vision_transform, state, camera_url, get_table_z,
    get_min_clearance, table_file."""
    global _registry, _runner
    _deps.update(deps)
    _registry = PolicyRegistry()
    _runner = PolicyRunner(_registry, TRACES_DIR, robot_factory=_robot_factory)
    app.include_router(router)


def _broadcast(payload: dict) -> None:
    fn = _deps.get("broadcast")
    if fn:
        try:
            fn(payload)
        except Exception:
            pass


def _robot_factory(trace, stop_event, confirm_mode: str):
    from dobot_ros.real.robot import RealRobot
    from dobot_ros.real.world import RealWorld
    from dobot_ros.web.pick_confirm import PickConfirmCancelled, PickConfirmTimeout

    client = _deps["get_client"]()
    state = _deps["state"]
    settings = _deps["settings_store"]
    table_z = _deps["get_table_z"]()
    if table_z is None:
        raise RuntimeError("table Z not calibrated — record the table plane first")
    min_clr = _deps["get_min_clearance"]()
    try:
        tool_len = float((settings.get("tool") or {}).get("tool_length_mm", 203.0))
    except Exception:
        tool_len = 203.0
    plane = load_table_plane(_deps.get("table_file"))
    guards = Guards(table_z, tool_len, min_clr, WorkspaceBounds.from_points(plane["points"]))

    def confirm(msg: str) -> None:
        try:
            _deps["pick_confirm"].ask(msg, timeout_s=120.0)
        except PickConfirmCancelled as e:
            raise PolicyAbort(f"cancelled: {e}") from None
        except PickConfirmTimeout as e:
            raise PolicyAbort(f"confirm timed out: {e}") from None

    def on_tool_change(idx: int) -> None:
        state["tool_index"] = idx
        try:
            settings.patch("tool", {"active_index": idx})
        except Exception:
            pass

    robot = RealRobot(client, trace, guards, tool_len,
                      get_tool_index=lambda: int(state.get("tool_index", 0)),
                      on_tool_change=on_tool_change, confirm_fn=confirm,
                      confirm_mode=confirm_mode, stop_event=stop_event)
    vt = _deps["get_vision_transform"]()
    world = RealWorld(_deps["camera_url"], vt, table_z, min_clr, tool_len)
    return robot, world


# ── models ──────────────────────────────────────────────────────────

class RunBody(BaseModel):
    slug: str
    target: str = "sim"
    params: Dict[str, Any] = {}
    scene: Any = "two_blocks"
    seed: Optional[int] = None
    random_positions: bool = False
    noise: Optional[Dict[str, Any]] = None
    pace: float = 1.0
    confirm_mode: str = "step"


class BatchBody(BaseModel):
    slug: str
    n: int = 20
    params: Dict[str, Any] = {}
    scene: Any = "two_blocks"
    seed: int = 0
    noise: Optional[Dict[str, Any]] = None


class SceneBody(BaseModel):
    name: str
    scene: Dict[str, Any]


class RandomizeBody(BaseModel):
    scene: Any = "two_blocks"
    seed: Optional[int] = None


class IKBody(BaseModel):
    poses: List[List[float]]
    seed: Optional[List[float]] = None
    tool_length_mm: float = 0.0


# ── policies ────────────────────────────────────────────────────────

@router.get("/api/policies/list")
def policies_list():
    return {"policies": _registry.list_policies(), "runner": _runner.status()}


@router.post("/api/policies/reload")
def policies_reload():
    _registry.reload()
    return {"success": True, "policies": _registry.slugs()}


@router.get("/api/policies/{slug}/params")
def policy_params(slug: str):
    if slug not in _registry.slugs():
        return JSONResponse(status_code=404, content={"error": f"unknown policy {slug}"})
    return {"slug": slug, "params": _registry.get_params(slug)}


@router.post("/api/policies/{slug}/params")
def policy_set_params(slug: str, params: Dict[str, Any]):
    try:
        return {"slug": slug, "params": _registry.set_params(slug, params)}
    except KeyError as e:
        return JSONResponse(status_code=404, content={"error": str(e)})


@router.get("/api/policies/{slug}/source")
def policy_source(slug: str):
    if slug not in _registry.slugs():
        return JSONResponse(status_code=404, content={"error": f"unknown policy {slug}"})
    return {"slug": slug, "path": str(_registry.source_path(slug)), "source": _registry.source(slug)}


@router.get("/api/policies/status")
def policies_status():
    return _runner.status()


@router.post("/api/policies/run")
def policies_run(body: RunBody):
    if body.slug not in _registry.slugs():
        return JSONResponse(status_code=404, content={"success": False, "error": f"unknown policy {body.slug}"})
    if _runner.is_running():
        return JSONResponse(status_code=409, content={"success": False, "error": "a policy run is already active"})
    if body.target not in ("sim", "robot"):
        return JSONResponse(status_code=400, content={"success": False, "error": "target must be sim or robot"})

    held = False
    if body.target == "robot":
        state = _deps["state"]
        if not state.get("connected"):
            return JSONResponse(status_code=409, content={"success": False, "error": "robot not connected"})
        blocked = _deps["acquire_motion"]("policy")
        if blocked:
            return JSONResponse(status_code=409, content={"success": False, "error": blocked})
        held = True

    req = RunRequest(slug=body.slug, target=body.target, params=body.params, scene=body.scene,
                     seed=body.seed, random_positions=body.random_positions, noise=body.noise,
                     pace=body.pace if body.target == "sim" else 0.0, confirm_mode=body.confirm_mode)

    def on_event(run_id: str, ev: dict) -> None:
        _broadcast({"type": "policy_event", "run_id": run_id, "target": body.target,
                    "slug": body.slug, "event": ev})

    def on_done(rec: dict) -> None:
        if held:
            _deps["release_motion"]()
        _broadcast({"type": "policy_done", "run_id": rec["id"], "slug": rec["slug"],
                    "target": rec["target"], "result": rec.get("result"),
                    "events": len(rec.get("events", []))})

    try:
        run_id = _runner.start(req, on_event=on_event, on_done=on_done)
    except Exception as e:
        if held:
            _deps["release_motion"]()
        return JSONResponse(status_code=500, content={"success": False, "error": str(e)})
    return {"success": True, "run_id": run_id, "target": body.target}


@router.post("/api/policies/batch")
def policies_batch(body: BatchBody):
    if body.slug not in _registry.slugs():
        return JSONResponse(status_code=404, content={"success": False, "error": f"unknown policy {body.slug}"})
    if _runner.is_running():
        return JSONResponse(status_code=409, content={"success": False, "error": "a policy run is already active"})
    n = max(1, min(500, int(body.n)))
    req = RunRequest(slug=body.slug, target="sim", params=body.params, scene=body.scene,
                     seed=body.seed, noise=body.noise, pace=0.0, random_positions=True)

    def on_event(batch_id: str, ev: dict) -> None:
        _broadcast({"type": "policy_batch", "batch_id": batch_id, **ev})

    def on_done(summary: dict) -> None:
        _broadcast({"type": "policy_batch_done", "batch_id": summary["batch_id"],
                    "summary": {k: v for k, v in summary.items() if k != "runs"},
                    "runs": summary["runs"]})

    batch_id = _runner.start_batch(req, n, on_event=on_event, on_done=on_done)
    return {"success": True, "batch_id": batch_id, "n": n}


@router.post("/api/policies/stop")
def policies_stop():
    was = _runner.stop()
    try:
        _deps["pick_confirm"].cancel_all("policy stopped")
    except Exception:
        pass
    return {"success": True, "was_running": was}


@router.get("/api/policies/traces")
def policies_traces(limit: int = 50):
    return {"traces": _runner.list_traces(limit=limit)}


@router.get("/api/policies/traces/{trace_id}")
def policies_trace(trace_id: str):
    rec = _runner.get_trace(trace_id)
    if rec is None:
        return JSONResponse(status_code=404, content={"error": "trace not found"})
    return rec


@router.delete("/api/policies/traces/{trace_id}")
def policies_trace_delete(trace_id: str):
    return {"success": _runner.delete_trace(trace_id)}


# ── simulator ───────────────────────────────────────────────────────

@router.get("/api/sim/scenes")
def sim_scenes():
    return {"scenes": list_scenes(), "noise_defaults": NoiseModel().to_dict()}


@router.get("/api/sim/scenes/{name}")
def sim_scene(name: str):
    try:
        scene = load_scene(name)
    except FileNotFoundError as e:
        return JSONResponse(status_code=404, content={"error": str(e)})
    world = SimWorld.from_scene(scene)
    return {"scene": scene, "objects": world.snapshot(), "table": _table_info(world)}


@router.post("/api/sim/scenes")
def sim_scene_save(body: SceneBody):
    try:
        p = save_scene(body.name, body.scene)
    except ValueError as e:
        return JSONResponse(status_code=400, content={"error": str(e)})
    return {"success": True, "name": p.stem}


@router.delete("/api/sim/scenes/{name}")
def sim_scene_delete(name: str):
    p = SCENES_DIR / f"{Path(name).name}.json"
    if not p.exists():
        return JSONResponse(status_code=404, content={"error": "scene not found"})
    p.unlink()
    return {"success": True}


@router.post("/api/sim/scenes/randomize")
def sim_scene_randomize(body: RandomizeBody):
    try:
        world = SimWorld.from_scene(body.scene, seed=body.seed, random_positions=True)
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})
    scene = world.to_scene()
    return {"scene": scene, "objects": world.snapshot(), "seed": body.seed, "table": _table_info(world)}


@router.get("/api/policies/safety")
def policies_safety():
    """The engine fail-safes as they will apply to a real run right now."""
    table_z = _deps["get_table_z"]()
    plane = load_table_plane(_deps.get("table_file"))
    try:
        tool_len = float((_deps["settings_store"].get("tool") or {}).get("tool_length_mm", 203.0))
    except Exception:
        tool_len = 203.0
    g = Guards(table_z if table_z is not None else plane["table_z"], tool_len,
               _deps["get_min_clearance"](), WorkspaceBounds.from_points(plane["points"]))
    return {"guards": g.to_dict(), "table_calibrated": table_z is not None,
            "floor_z_tool0": g.min_pose_z(0), "floor_z_tool1": g.min_pose_z(1),
            "surface_z": g.table_z - g.tool_length_mm,
            "layers": [
                "endpoint guards: floor, workspace, tilt envelope",
                "per-move travel cap",
                "model path preflight (every sample reachable and above the floor)",
                "straight-line MovL only — never joint-interpolated cartesian moves",
                "step confirm modal before each motion (confirm mode = step)",
                "10 Hz watchdog: live floor / workspace / ERROR / COLLISION → Stop",
                "speed and gripper force caps",
                "sim rehearsal against the live scene before the first real motion",
                "motion lock: no other motion source while a policy runs",
            ]}


@router.get("/api/sim/table")
def sim_table():
    world = SimWorld.from_scene({"name": "empty", "objects": []})
    return _table_info(world)


@router.post("/api/sim/ik")
def sim_ik(body: IKBody):
    """Offline IK from the URDF model — works with the robot powered off."""
    out = []
    seed = body.seed
    for p in body.poses:
        try:
            q = K.ik(p, seed_deg=seed, tool_length_mm=body.tool_length_mm)
        except K.IKError as e:
            return JSONResponse(status_code=422, content={"success": False, "error": str(e)})
        out.append([round(v, 3) for v in q])
        seed = q
    return {"success": True, "joints": out}


@router.post("/api/sim/fk")
def sim_fk(body: Dict[str, Any]):
    joints = body.get("joints")
    tl = float(body.get("tool_length_mm", 0.0))
    if not joints or len(joints) != 6:
        return JSONResponse(status_code=400, content={"error": "joints[6] required"})
    return {"pose": [round(v, 3) for v in K.fk(joints, tl)]}


def _table_info(world: SimWorld) -> dict:
    return {"table_z": world.table_z, "surface_z": world.surface_z,
            "tool_length_mm": world.tool_length_mm, "min_clearance_mm": world.min_clearance_mm,
            "workspace": world.workspace.to_dict()}
