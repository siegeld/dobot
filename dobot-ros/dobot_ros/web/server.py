"""
FastAPI web server for Dobot CR5 robot control.

Provides REST API + WebSocket for real-time monitoring and control.
"""

import asyncio
import json
import logging
import math
import os
import subprocess
import time
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

import rclpy
from rclpy.executors import MultiThreadedExecutor
from dobot_ros.config import Config
from dobot_ros.ros_client import DobotRosClient
from dobot_ros.web.settings_store import (
    SettingsStore, SettingsBlocked, SettingsNotFound, SettingsConflict, SettingsError,
)


# ── Global state ────────────────────────────────────────────────

_client: Optional[DobotRosClient] = None
_executor: Optional[MultiThreadedExecutor] = None
_config: Optional[Config] = None
_lock = threading.Lock()
_ws_clients: set = set()

# Cached state for polling.
# speed_factor starts at -1 to indicate "unknown" until we've reconciled with
# the robot or the settings store; the UI treats -1 as "--". See below —
# _get_client() pushes the stored jog.speed to the robot on first connect.
_state = {
    "connected": False,
    "robot_mode": -1,
    "joint": [0.0] * 6,
    "cartesian": [0.0] * 6,
    "gripper_position": -1,
    "gripper_state": -1,
    "gripper_initialized": False,
    "speed_factor": -1,
    "last_update": 0,
    "uptime_start": 0,
    "error": None,
}


def _get_client() -> DobotRosClient:
    global _client, _executor
    if _client is None:
        if not rclpy.ok():
            rclpy.init()
        _executor = MultiThreadedExecutor()
        _client = DobotRosClient(
            namespace=_config.ros_namespace if _config else '',
            service_timeout=_config.service_timeout if _config else 5.0,
            subscribe_topics=True,
            managed_executor=True,
        )
        _executor.add_node(_client)
        # Dedicated spin thread processes ALL callbacks (topics + service responses)
        # continuously — no more fighting between poll thread and API calls.
        spin_thread = threading.Thread(target=_executor.spin, daemon=True)
        spin_thread.start()

        # Reconcile global speed with the settings store so the status display
        # reflects reality, not the cached default. Fire and forget — the robot
        # may not accept the command until Enable, in which case the next
        # /api/speed or /api/enable call will push it again.
        try:
            stored = int(_settings_store.get("jog").get("speed", 5))
        except Exception:
            stored = 5
        try:
            _client.set_speed_factor(stored)
        except Exception as e:
            logging.info("startup set_speed_factor(%d) deferred: %s", stored, e)
        with _lock:
            _state["speed_factor"] = stored
    return _client


def _poll_state():
    """Poll robot state in a background thread.

    Robot state comes from ROS2 topic subscriptions cached in the
    DobotRosClient. The dedicated executor spin thread processes
    callbacks continuously — we just read the latest cached values.
    """
    while True:
        try:
            client = _get_client()
            with _lock:
                try:
                    angles = client.get_joint_angles()
                    pose = client.get_cartesian_pose()
                    mode = client.get_robot_mode()
                    stale = client.is_feedback_stale(max_age=5.0)
                    _state["joint"] = angles
                    _state["cartesian"] = pose
                    _state["robot_mode"] = mode
                    _state["connected"] = not stale
                    _state["error"] = "Robot feedback stale — robot may be offline" if stale else None
                    _state["last_update"] = time.time()
                except Exception as e:
                    _state["connected"] = False
                    _state["error"] = str(e)
                # Gripper state from /gripper/state topic (no Modbus)
                _state["gripper_position"] = client.gripper_get_position()
                _state["gripper_state"] = client.gripper_get_state()
                _state["gripper_initialized"] = client.gripper_get_init_status() == 1
        except Exception:
            _state["connected"] = False
        time.sleep(0.2)  # 5 Hz


# ── Lifespan ────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    _state["uptime_start"] = time.time()
    # Start background polling thread (handles robot + gripper state via topics)
    t1 = threading.Thread(target=_poll_state, daemon=True)
    t1.start()
    yield
    # Cleanup
    global _client, _executor
    if _executor:
        try:
            _executor.shutdown()
        except Exception:
            pass
        _executor = None
    if _client:
        try:
            _client.shutdown()
        except Exception:
            pass
        _client = None
    if rclpy.ok():
        rclpy.shutdown()


# ── App ─────────────────────────────────────────────────────────

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="Dobot CR5 Controller", lifespan=lifespan)


# ── Models ──────────────────────────────────────────────────────

class JogRequest(BaseModel):
    axis: str  # x, y, z, rx, ry, rz, j1-j6
    distance: float
    speed: int = 50
    mode: str = "user"  # user or tool

class GripperMoveRequest(BaseModel):
    position: int  # 0-1000
    speed: int = 50
    force: int = 50

class SpeedRequest(BaseModel):
    speed: int  # 1-100

class MoveJointsRequest(BaseModel):
    angles: list  # 6 joint angles

class ConfigUpdateRequest(BaseModel):
    jog_distance: Optional[float] = None
    jog_rotation: Optional[float] = None
    jog_speed: Optional[int] = None
    jog_mode: Optional[str] = None
    sync_mode: Optional[bool] = None


# ── REST API ────────────────────────────────────────────────────

@app.get("/api/status")
async def get_status():
    """Get full robot status."""
    with _lock:
        uptime = time.time() - _state["uptime_start"] if _state["uptime_start"] else 0
        mode_names = {
            1: "INIT", 2: "BRAKE_OPEN", 3: "POWEROFF",
            4: "DISABLED", 5: "ENABLE", 6: "BACKDRIVE",
            7: "RUNNING", 8: "SINGLE_STEP", 9: "ERROR",
            10: "PAUSE", 11: "COLLISION",
        }
        return {
            "connected": _state["connected"],
            "robot_mode": _state["robot_mode"],
            "robot_mode_name": mode_names.get(_state["robot_mode"], "UNKNOWN"),
            "joint": _state["joint"],
            "cartesian": _state["cartesian"],
            "gripper_position": _state["gripper_position"],
            "gripper_state": _state["gripper_state"],
            "gripper_initialized": _state["gripper_initialized"],
            "speed_factor": _state["speed_factor"],
            "uptime_seconds": int(uptime),
            "last_update": _state["last_update"],
            "error": _state["error"],
        }


@app.get("/api/position")
async def get_position():
    """Get current position (joint + cartesian)."""
    with _lock:
        return {
            "joint": dict(zip(["J1", "J2", "J3", "J4", "J5", "J6"], _state["joint"])),
            "cartesian": dict(zip(["X", "Y", "Z", "RX", "RY", "RZ"], _state["cartesian"])),
        }


@app.post("/api/enable")
async def enable_robot():
    """Enable the robot."""
    try:
        client = _get_client()
        res = client.enable_robot()
        # Safety: set speed to 5% on enable
        client.set_speed_factor(5)
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/disable")
async def disable_robot():
    """Disable the robot."""
    try:
        client = _get_client()
        res = client.disable_robot()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/clear")
async def clear_error():
    """Clear robot errors."""
    try:
        client = _get_client()
        res = client.clear_error()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/stop")
async def stop_robot():
    """Emergency stop."""
    try:
        client = _get_client()
        res = client.stop()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/drag/start")
async def start_drag():
    """Enter drag/freedrive mode."""
    try:
        client = _get_client()
        res = client.start_drag()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/drag/stop")
async def stop_drag():
    """Exit drag/freedrive mode."""
    try:
        client = _get_client()
        res = client.stop_drag()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/speed")
async def set_speed(req: SpeedRequest):
    """Set global speed factor. Also persists to the settings store so the
    value survives container restarts."""
    try:
        client = _get_client()
        res = client.set_speed_factor(req.speed)
        with _lock:
            _state["speed_factor"] = req.speed
        try:
            _settings_store.patch("jog", {"speed": int(req.speed)})
        except Exception as e:
            logging.warning("persisting jog.speed failed: %s", e)
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/jog")
async def jog_robot(req: JogRequest):
    """Jog robot by relative distance."""
    try:
        client = _get_client()
        client.set_speed_factor(req.speed)
        with _lock:
            _state["speed_factor"] = req.speed

        axis = req.axis.lower()
        if axis.startswith('j'):
            joint_num = int(axis[1])
            client.jog_joint(joint_num, req.distance)
        else:
            client.set_jog_mode(req.mode)
            params = {a: 0.0 for a in ('x', 'y', 'z', 'rx', 'ry', 'rz')}
            params[axis] = req.distance
            client.jog(**params)

        return {"success": True, "axis": req.axis, "distance": req.distance}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/move_joints")
async def move_joints(req: MoveJointsRequest):
    """Move to absolute joint positions."""
    try:
        if len(req.angles) != 6:
            return {"success": False, "error": "Must provide 6 joint angles"}
        client = _get_client()
        res = client.move_joints(req.angles)
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ── Gripper API ─────────────────────────────────────────────────

@app.post("/api/gripper/init")
async def gripper_init():
    """Initialize the gripper."""
    try:
        client = _get_client()
        client.gripper_init()
        with _lock:
            _state["gripper_initialized"] = True
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/open")
async def gripper_open(speed: int = 100, force: int = 20):
    """Open the gripper (fire-and-forget, state updates via WebSocket)."""
    try:
        client = _get_client()
        client.gripper_open(force=force, speed=speed, wait=False)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/close")
async def gripper_close(speed: int = 100, force: int = 20):
    """Close the gripper (fire-and-forget, state updates via WebSocket)."""
    try:
        client = _get_client()
        client.gripper_close(force=force, speed=speed, wait=False)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/move")
async def gripper_move(req: GripperMoveRequest):
    """Move gripper to position (fire-and-forget, state updates via WebSocket)."""
    try:
        client = _get_client()
        client.gripper_move(req.position, force=req.force, speed=req.speed, wait=False)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/gripper/status")
async def gripper_status():
    """Get gripper status."""
    with _lock:
        return {
            "initialized": _state["gripper_initialized"],
            "position": _state["gripper_position"],
            "state": _state["gripper_state"],
            "state_name": {0: "moving", 1: "reached", 2: "caught", 3: "dropped"}.get(
                _state["gripper_state"], "unknown"
            ),
        }


# ── Config API ──────────────────────────────────────────────────

@app.get("/api/config")
async def get_config():
    """Get current configuration."""
    if _config:
        return {
            "robot_ip": os.environ.get("IP_address", _config.robot_ip),
            "ros_namespace": _config.ros_namespace,
            "service_timeout": _config.service_timeout,
            "jog_distance": _config.jog_default_distance,
            "jog_rotation": _config.jog_default_rotation,
            "jog_speed": _config.jog_speed,
            "jog_mode": _config.jog_coordinate_mode,
            "sync_mode": _config.sync_mode,
            "motion_tolerance_deg": _config.motion_tolerance_deg,
            "motion_tolerance_mm": _config.motion_tolerance_mm,
            "motion_timeout": _config.motion_timeout,
            "precision": _config.precision,
        }
    return {}


# ── Calibration API (proxies to camera server) ──────────────────

CAMERA_URL = os.environ.get("CAMERA_URL", "http://10.11.6.65:8080")


@app.get("/api/calibration/status")
async def calibration_status():
    """Get calibration status from camera server."""
    try:
        import requests
        resp = requests.get(f"{CAMERA_URL}/api/calibration", timeout=5)
        return resp.json()
    except Exception as e:
        return {"error": str(e)}


@app.post("/api/calibration/record")
async def calibration_record(req: dict = None):
    """Record a calibration point: robot XYZ + camera 3D at pixel (px, py).

    If px/py provided, uses depth at that pixel. Otherwise uses frame center.
    """
    import requests
    try:
        client = _get_client()
        robot_pose = client.get_cartesian_pose()
        robot_xyz = robot_pose[:3]  # [X, Y, Z] in mm

        px = req.get("px", 320) if req else 320
        py = req.get("py", 180) if req else 180

        # Use a small region around the clicked pixel for more robust depth
        patch = 10  # +/- pixels
        region_resp = requests.get(
            f"{CAMERA_URL}/api/depth/region",
            params={"x": px - patch, "y": py - patch,
                    "width": patch * 2, "height": patch * 2},
            timeout=5,
        ).json()

        depth_mean = region_resp.get("depth_mean", 0)
        valid = region_resp.get("valid_pixels", 0)
        if depth_mean <= 0 or valid < 10:
            return {"success": False,
                    "error": f"No good depth at pixel ({px}, {py}) — {valid} valid pixels"}

        # Log depth quality for debugging
        stddev = region_resp.get("depth_stddev", 0)

        # Get 3D position at the center pixel
        point_resp = requests.get(
            f"{CAMERA_URL}/api/depth/at", params={"x": px, "y": py}, timeout=5
        ).json()

        distance = point_resp.get("distance", 0)
        if distance <= 0.1 or distance > 2.0:
            return {"success": False,
                    "error": f"Bad depth at ({px}, {py}): {distance:.3f}m — expected 0.1-2.0m"}

        camera_xyz = point_resp["position_3d"]
        result = requests.post(f"{CAMERA_URL}/api/calibration/point", json={
            "camera_xyz": camera_xyz,
            "robot_xyz": robot_xyz,
        }, timeout=5).json()

        return {
            "success": True,
            "robot_xyz": robot_xyz,
            "camera_xyz": camera_xyz,
            "depth_m": round(distance, 3),
            "depth_stddev": round(stddev, 3),
            "valid_pixels": valid,
            "total_points": result.get("total_points", 0),
        }
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/solve")
async def calibration_solve():
    """Solve the camera-to-robot transform."""
    try:
        import requests
        resp = requests.post(f"{CAMERA_URL}/api/calibration/solve", timeout=10)
        return resp.json()
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/clear")
async def calibration_clear():
    """Clear all calibration points."""
    try:
        import requests
        requests.post(f"{CAMERA_URL}/api/calibration/clear", timeout=5)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/delete-point")
async def calibration_delete_point(req: dict):
    """Delete a calibration point by index. Re-creates remaining points."""
    import requests as rq
    try:
        index = req.get("index", -1)
        status = rq.get(f"{CAMERA_URL}/api/calibration", timeout=5).json()
        points = status.get("points", [])
        if index < 0 or index >= len(points):
            return {"success": False, "error": f"Invalid index {index}"}
        rq.post(f"{CAMERA_URL}/api/calibration/clear", timeout=5)
        for i, pt in enumerate(points):
            if i != index:
                rq.post(f"{CAMERA_URL}/api/calibration/point", json={
                    "camera_xyz": pt["camera_xyz"],
                    "robot_xyz": pt["robot_xyz"],
                }, timeout=5)
        return {"success": True, "remaining": len(points) - 1}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/calibration/test")
async def calibration_test():
    """Test calibration: for each detected object, show predicted robot coords."""
    import requests
    try:
        client = _get_client()
        robot_pose = client.get_cartesian_pose()
        actual_xyz = robot_pose[:3]

        objects_resp = requests.get(
            f"{CAMERA_URL}/api/detection/objects", timeout=5
        ).json()

        results = []
        for obj in objects_resp.get("objects", []):
            try:
                transform_resp = requests.get(
                    f"{CAMERA_URL}/api/calibration/transform",
                    params={"x": obj["position_3d"][0],
                            "y": obj["position_3d"][1],
                            "z": obj["position_3d"][2]},
                    timeout=5,
                ).json()
                results.append({
                    "id": obj["id"],
                    "label": obj["label"],
                    "camera_3d": obj["position_3d"],
                    "robot_xyz": transform_resp["robot_xyz"],
                    "depth_m": obj["depth_mean"],
                })
            except Exception:
                pass

        return {
            "success": True,
            "actual_robot_xyz": actual_xyz,
            "objects": results,
        }
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/calibration/camera-frame")
async def calibration_camera_frame():
    """Proxy the detection frame from camera server."""
    import requests
    from fastapi.responses import Response
    try:
        resp = requests.get(f"{CAMERA_URL}/api/frame/color", timeout=5)
        return Response(content=resp.content, media_type="image/jpeg")
    except Exception as e:
        return {"error": str(e)}


_TABLE_FILE = Path(__file__).parent / "table_plane.json"


@app.get("/api/calibration/table")
async def get_table():
    """Get saved table plane data."""
    try:
        if _TABLE_FILE.exists():
            return json.loads(_TABLE_FILE.read_text())
    except Exception:
        pass
    return {"points": [], "plane": None, "min_clearance_mm": 25}


@app.post("/api/calibration/table/clearance")
async def set_table_clearance(req: dict):
    """Set minimum clearance above table in mm."""
    try:
        data = json.loads(_TABLE_FILE.read_text()) if _TABLE_FILE.exists() else {"points": [], "plane": None}
        data["min_clearance_mm"] = req.get("min_clearance_mm", 25)
        _TABLE_FILE.write_text(json.dumps(data, indent=2))
        return {"success": True, "min_clearance_mm": data["min_clearance_mm"]}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/table/point")
async def add_table_point():
    """Record a table point from current robot Z position."""
    try:
        client = _get_client()
        pose = client.get_cartesian_pose()
        xyz = pose[:3]

        data = json.loads(_TABLE_FILE.read_text()) if _TABLE_FILE.exists() else {"points": [], "plane": None}
        data["points"].append(xyz)

        # Solve plane if we have 3+ points
        if len(data["points"]) >= 3:
            import numpy as np
            pts = np.array(data["points"])
            # Fit plane: ax + by + cz + d = 0 using SVD
            centroid = pts.mean(axis=0)
            centered = pts - centroid
            _, _, Vt = np.linalg.svd(centered)
            normal = Vt[-1]
            # Ensure normal points up (positive Z component)
            if normal[2] < 0:
                normal = -normal
            d = -normal.dot(centroid)
            data["plane"] = {
                "normal": normal.tolist(),
                "d": float(d),
                "z_at_origin": float(-d / normal[2]) if abs(normal[2]) > 0.001 else None,
                "mean_z": float(pts[:, 2].mean()),
            }

        _TABLE_FILE.write_text(json.dumps(data, indent=2))
        return {"success": True, "num_points": len(data["points"]), "point": xyz, "plane": data.get("plane")}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/table/z")
async def set_table_z():
    """Record the current wrist Z as the table floor (single-touch calibration).

    This is the simpler, preferred workflow: touch the gripper tip to any point
    on the (flat, level) table, click once, done. The 4-point / plane-fitting
    flow is still available in /api/calibration/table/point for rare cases
    where tilt compensation is needed.

    Stores alongside any existing `points`/`plane` so the two schemas coexist.
    """
    try:
        client = _get_client()
        pose = client.get_cartesian_pose()
        wrist_z = float(pose[2])
        data = json.loads(_TABLE_FILE.read_text()) if _TABLE_FILE.exists() else {}
        data["table_z"] = wrist_z
        data.setdefault("min_clearance_mm", 25)
        _TABLE_FILE.write_text(json.dumps(data, indent=2))
        return {"success": True, "table_z": wrist_z}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/calibration/table/clear")
async def clear_table():
    """Clear table plane data (both single-Z and multi-point schemas)."""
    try:
        _TABLE_FILE.write_text(json.dumps({"points": [], "plane": None}))
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _get_table_z() -> Optional[float]:
    """Return the effective table Z in wrist coords, from either schema."""
    try:
        if not _TABLE_FILE.exists():
            return None
        data = json.loads(_TABLE_FILE.read_text())
        if "table_z" in data:
            return float(data["table_z"])
        plane = data.get("plane") or {}
        mz = plane.get("mean_z")
        if mz is not None:
            return float(mz)
    except Exception:
        pass
    return None


def _get_min_clearance_mm() -> float:
    try:
        if not _TABLE_FILE.exists():
            return 25.0
        return float(json.loads(_TABLE_FILE.read_text()).get("min_clearance_mm", 25))
    except Exception:
        return 25.0


@app.post("/api/calibration/workspace-move")
async def workspace_move(req: dict):
    """Move robot to a workspace position (center or corner) at given height above table."""
    try:
        data = json.loads(_TABLE_FILE.read_text()) if _TABLE_FILE.exists() else {}
        pts = data.get("points", [])
        plane = data.get("plane")
        if len(pts) < 3 or not plane:
            return {"success": False, "error": "Need at least 3 table points"}

        pos = req.get("position", "center")
        height_mm = req.get("height_mm", 127)

        import numpy as np
        pts_arr = np.array(pts)

        if pos == "center":
            target_xy = pts_arr[:, :2].mean(axis=0)
        elif pos in ("c1", "c2", "c3", "c4"):
            idx = int(pos[1]) - 1
            if idx >= len(pts):
                return {"success": False, "error": f"Corner {pos} not recorded"}
            target_xy = pts_arr[idx, :2]
        else:
            return {"success": False, "error": f"Unknown position: {pos}"}

        # Compute table Z at this XY using plane equation
        normal = np.array(plane["normal"])
        d = plane["d"]
        # ax + by + cz + d = 0 => z = -(ax + by + d) / c
        table_z = -(normal[0] * target_xy[0] + normal[1] * target_xy[1] + d) / normal[2]
        target_z = table_z + height_mm

        client = _get_client()
        client.set_speed_factor(5)

        # height_mm is above table for gripper tip
        # Table Z is in wrist coords (gripper tip touches table at this Z)
        # So wrist target = table_z + height_mm (no gripper offset needed
        # because table was calibrated by touching gripper tip to table)
        current = client.get_cartesian_pose()
        dx = float(target_xy[0]) - current[0]
        dy = float(target_xy[1]) - current[1]
        dz = float(target_z) - current[2]
        client.jog(x=dx, y=dy, z=dz)

        return {
            "success": True,
            "target": [float(target_xy[0]), float(target_xy[1]), float(target_z)],
            "table_z": round(float(table_z), 1),
        }
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/calibration/depth-at")
async def calibration_depth_at(x: int, y: int):
    """Get depth at a pixel from camera server."""
    import requests
    try:
        resp = requests.get(f"{CAMERA_URL}/api/depth/at", params={"x": x, "y": y}, timeout=2).json()
        return resp
    except Exception as e:
        return {"error": str(e)}


# ── Object detection + pick planning ────────────────────────────
#
# Proxies the camera server's detection so the browser never talks to the
# camera server directly (keeps CORS simple and lets us add overlays later).
# The /api/pick/plan endpoint returns waypoints without moving — callers
# then call /api/pick/execute to actually run.

@app.get("/api/detection/objects")
async def detection_objects():
    """Proxy the camera server's object detection."""
    import requests
    try:
        resp = requests.get(f"{CAMERA_URL}/api/detection/objects", timeout=3).json()
        return resp
    except Exception as e:
        return {"objects": [], "error": str(e)}


def _compute_pick_waypoints(
    robot_x: float, robot_y: float,
    rotation_deg: float = 0.0,
    approach_mm: float = 100.0,
    grasp_clearance_mm: float = 5.0,
    lift_mm: float = 150.0,
):
    """Compute the three waypoints for a pick at (robot_x, robot_y).
    Z floor comes from the calibrated table_z plus the user's min_clearance_mm
    so we never drive below the safe height.

    Returns (approach, grasp, retract, table_z, clearance_mm) with each waypoint
    a 6-D list [X, Y, Z, RX, RY, RZ]. RX/RY are held at current; RZ is the
    desired gripper rotation.
    """
    table_z = _get_table_z()
    if table_z is None:
        raise RuntimeError("table Z not calibrated")
    min_clr = _get_min_clearance_mm()

    grasp_z = table_z + max(grasp_clearance_mm, min_clr)
    approach_z = grasp_z + approach_mm
    retract_z = grasp_z + lift_mm

    # Use current RX/RY from the robot so the wrist orientation is preserved.
    client = _get_client()
    cur = client.get_cartesian_pose()
    rx, ry = cur[3], cur[4]

    approach = [robot_x, robot_y, approach_z, rx, ry, rotation_deg]
    grasp = [robot_x, robot_y, grasp_z, rx, ry, rotation_deg]
    retract = [robot_x, robot_y, retract_z, rx, ry, rotation_deg]

    return approach, grasp, retract, table_z, min_clr


class PickPlanRequest(BaseModel):
    px: Optional[int] = None
    py: Optional[int] = None
    object_id: Optional[int] = None
    approach_mm: float = 100.0
    grasp_clearance_mm: float = 5.0
    lift_mm: float = 150.0


@app.post("/api/pick/plan")
async def pick_plan(req: PickPlanRequest):
    """Resolve a target (pixel or object id) into robot coordinates and
    compute the full waypoint plan for preview. Does NOT move the robot.

    Returns waypoints, the resolved camera 3-D, robot 3-D, table Z, and any
    safety warnings (out-of-bounds, unreachable, etc.)."""
    import requests
    try:
        obj = None
        camera_xyz = None
        rotation_deg = 0.0

        if req.object_id is not None:
            objs = requests.get(f"{CAMERA_URL}/api/detection/objects", timeout=3).json()
            for o in objs.get("objects", []):
                if o.get("id") == req.object_id:
                    obj = o
                    camera_xyz = o["position_3d"]
                    rotation_deg = float(o.get("rotation_deg", 0.0))
                    break
            if obj is None:
                return JSONResponse(status_code=404,
                    content={"success": False, "error": f"object id {req.object_id} not found"})
        elif req.px is not None and req.py is not None:
            depth = requests.get(f"{CAMERA_URL}/api/depth/at",
                params={"x": req.px, "y": req.py}, timeout=3).json()
            if depth.get("distance", 0) <= 0:
                return JSONResponse(status_code=400,
                    content={"success": False, "error": f"no depth at pixel ({req.px}, {req.py})"})
            camera_xyz = depth["position_3d"]
        else:
            return JSONResponse(status_code=400,
                content={"success": False, "error": "provide object_id or (px, py)"})

        # Transform camera → robot via camera-server calibration.
        tf_resp = requests.get(
            f"{CAMERA_URL}/api/calibration/transform",
            params={"x": camera_xyz[0], "y": camera_xyz[1], "z": camera_xyz[2]},
            timeout=3,
        )
        if tf_resp.status_code != 200:
            # Camera server returns text/plain for errors; surface that message.
            msg = tf_resp.text.strip() or f"camera transform failed (HTTP {tf_resp.status_code})"
            return JSONResponse(status_code=400,
                content={"success": False, "error": msg})
        try:
            tf = tf_resp.json()
        except Exception:
            return JSONResponse(status_code=500,
                content={"success": False, "error": "camera transform returned non-JSON"})
        if not tf.get("robot_xyz"):
            return JSONResponse(status_code=400,
                content={"success": False, "error": "camera calibration not solved or transform failed"})
        robot_xyz = tf["robot_xyz"]

        approach, grasp, retract, table_z, clearance = _compute_pick_waypoints(
            robot_x=float(robot_xyz[0]), robot_y=float(robot_xyz[1]),
            rotation_deg=rotation_deg,
            approach_mm=req.approach_mm,
            grasp_clearance_mm=req.grasp_clearance_mm,
            lift_mm=req.lift_mm,
        )

        # Safety sanity checks — surface warnings without blocking.
        warnings = []
        if grasp[2] < table_z - 1:
            warnings.append(
                f"grasp Z {grasp[2]:.1f} is below table_z {table_z:.1f} — would crash gripper"
            )

        return {
            "success": True,
            "target": {
                "camera_xyz": camera_xyz,
                "robot_xyz": list(map(float, robot_xyz)),
                "rotation_deg": rotation_deg,
                "object": obj,
            },
            "table_z": table_z,
            "min_clearance_mm": clearance,
            "waypoints": {
                "approach": approach, "grasp": grasp, "retract": retract,
            },
            "warnings": warnings,
        }
    except Exception as e:
        logging.exception("pick plan failed")
        return JSONResponse(status_code=500, content={"success": False, "error": str(e)})


class PickExecuteRequest(PickPlanRequest):
    # Execute accepts the same target inputs as plan and re-runs planning
    # internally so the user's "pick" click always works on the freshest frame.
    confirm: bool = False


@app.post("/api/pick/execute")
async def pick_execute(req: PickExecuteRequest):
    """Execute a full pick sequence: approach → grasp → retract.
    The client must send confirm=true — a safety interlock so an errant POST
    can't launch the robot."""
    if not req.confirm:
        return JSONResponse(status_code=400,
            content={"success": False, "error": "pass confirm=true to execute"})

    # Re-plan just before executing — no stale waypoints.
    plan_req = PickPlanRequest(**{k: getattr(req, k) for k in PickPlanRequest.model_fields.keys()})
    plan_resp = await pick_plan(plan_req)
    if isinstance(plan_resp, JSONResponse):
        return plan_resp
    if not plan_resp.get("success"):
        return JSONResponse(status_code=400, content=plan_resp)

    try:
        import time as _time
        wp = plan_resp["waypoints"]
        approach, grasp, retract = wp["approach"], wp["grasp"], wp["retract"]
        client = _get_client()

        logging.info("pick: opening gripper")
        try:
            client.gripper_move(800, force=50, speed=50, wait=True)
        except Exception as e:
            logging.warning("gripper open failed (continuing): %s", e)

        logging.info("pick: approach %s", approach)
        client.move_pose(approach)

        logging.info("pick: descend to grasp %s", grasp)
        client.move_pose(grasp)
        _time.sleep(0.2)

        logging.info("pick: closing gripper")
        try:
            client.gripper_move(200, force=50, speed=50, wait=True)
        except Exception as e:
            logging.warning("gripper close failed: %s", e)

        logging.info("pick: retract %s", retract)
        client.move_pose(retract)

        return {"success": True, "waypoints": wp, "target": plan_resp["target"]}
    except Exception as e:
        logging.exception("pick execute failed")
        return JSONResponse(status_code=500, content={"success": False, "error": str(e)})


# ── Camera State ─────────────────────────────────────────────────

_CAMERA_FILE = Path(__file__).parent / "camera_state.json"


@app.get("/api/camera")
async def get_camera():
    """Get saved camera view (now stored in settings_store under 'camera_view')."""
    try:
        view = _settings_store.get("camera_view")
        if view:
            return view
        # Fallback to legacy file if the store is empty.
        if _CAMERA_FILE.exists():
            return json.loads(_CAMERA_FILE.read_text())
    except Exception:
        pass
    return {}


@app.post("/api/camera")
async def save_camera(request: dict):
    """Save camera view (position + target) — routes through settings store."""
    try:
        _settings_store.set_group("camera_view", request)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ── Inverse Kinematics (query-only, no motion) ──────────────────

class InverseKinRequest(BaseModel):
    poses: list  # List of [X, Y, Z, RX, RY, RZ]


@app.post("/api/inverse-kin")
async def inverse_kin(req: InverseKinRequest):
    """Compute joint angles for one or more cartesian poses using the robot's
    own IK solver. Pure query — the robot does NOT move.

    Used by the 3D simulation mode to animate the planned trajectory.
    """
    try:
        client = _get_client()
        results = []
        for pose in req.poses:
            if len(pose) != 6:
                return JSONResponse(status_code=400,
                    content={"success": False, "error": f"each pose must be 6-D, got {len(pose)}"})
            joints = client.inverse_kin(pose)
            results.append(joints)
        return {"success": True, "joints": results}
    except Exception as e:
        logging.exception("inverse-kin failed")
        return JSONResponse(status_code=500, content={"success": False, "error": str(e)})


# ── VLA (Vision-Language-Action) ─────────────────────────────────
#
# Recorder + executor live in dobot_ros.vla. The web layer owns singleton
# instances so multiple clients hitting the dashboard see shared state.
# See VLA.md for the full design.

_vla_recorder = None
_vla_executor = None
_vla_lock = threading.Lock()

VLA_EPISODES_DIR = os.environ.get("VLA_EPISODES_DIR", "/data/episodes")
VLA_OFT_URL = os.environ.get("VLA_OFT_URL", "http://gpurbr2:7071")


# ── Settings store (single source of truth for persistent UI + VLA + servo config) ─

_SETTINGS_DIR = Path(__file__).parent / "settings"

_SETTINGS_DEFAULTS = {
    "servo": {
        "servo_rate_hz": 30.0, "t": 0.05,
        "aheadtime": 50.0, "gain": 500.0,
    },
    "vla": {
        "server_url": VLA_OFT_URL,
        "servo_rate_hz": 30.0,
        "model_rate_hz": 10.0,
        "chunk_actions_to_execute": 4,
        "gripper_threshold": 0.1,
        "unnorm_key": None,
    },
    "table_plane": {},
    "camera_view": {},
    "gripper": {"force": 20, "speed": 50},
    "jog": {"linear_step": 5, "angular_step": 5, "speed": 5, "mode": "user"},
}

# Legacy per-file JSON paths migrate into the store on first startup.
_SETTINGS_LEGACY = {
    "table_plane": Path(__file__).parent / "table_plane.json",
    "camera_view": Path(__file__).parent / "camera_state.json",
}

_settings_store: Optional[SettingsStore] = SettingsStore(
    base_dir=_SETTINGS_DIR,
    defaults=_SETTINGS_DEFAULTS,
    legacy_files=_SETTINGS_LEGACY,
)


def _settings_load_safety_check():
    """Guard used when loading a saved settings bundle — refuses when the
    robot is moving, in an error/collision state, or when streaming control
    paths (servo tester, VLA executor/recorder) are active.

    See feedback_always_safe memory.
    """
    reasons = []
    try:
        if _servo_tester is not None and _servo_tester.is_running():
            reasons.append("servo tester is running — stop it first")
    except Exception:
        pass
    try:
        if _vla_executor is not None and _vla_executor.is_running():
            reasons.append("VLA executor is running — stop it first")
    except Exception:
        pass
    try:
        if _vla_recorder is not None and _vla_recorder.is_recording():
            reasons.append("VLA recorder is recording — stop it first")
    except Exception:
        pass

    unsafe_modes = {6: "BACKDRIVE/drag", 7: "RUNNING", 9: "ERROR", 11: "COLLISION"}
    mode = _state.get("robot_mode", -1)
    if mode in unsafe_modes:
        reasons.append(f"robot is in {unsafe_modes[mode]} mode — stop/clear first")

    return (not reasons, reasons)


def _get_vla_recorder():
    global _vla_recorder
    with _vla_lock:
        if _vla_recorder is None:
            from dobot_ros.vla.recorder import EpisodeRecorder, CameraFrameSource
            _vla_recorder = EpisodeRecorder(
                ros_client=_get_client(),
                camera=CameraFrameSource(camera_url=CAMERA_URL),
                episodes_dir=VLA_EPISODES_DIR,
                rate_hz=10.0,
            )
        return _vla_recorder


@app.get("/api/vla/status")
async def vla_status():
    """Combined status of recorder + executor + OFT server health."""
    from dobot_ros.vla.client import OFTClient

    rec = _vla_recorder
    ex = _vla_executor

    server_health = None
    try:
        server_health = OFTClient(base_url=VLA_OFT_URL, timeout=1.5).health()
    except Exception as e:
        server_health = {"status": "unreachable", "error": str(e)}

    return {
        "recording": rec.is_recording() if rec else False,
        "executing": ex.is_running() if ex else False,
        "executor": (ex.status().__dict__ if ex else None),
        "server_url": VLA_OFT_URL,
        "server": server_health,
        "episodes_dir": VLA_EPISODES_DIR,
    }


@app.get("/api/vla/episodes")
async def vla_list_episodes():
    from dobot_ros.vla.recorder import list_episodes
    return {"episodes": list_episodes(VLA_EPISODES_DIR)}


class VLARecordStart(BaseModel):
    instruction: str
    name: Optional[str] = None


@app.post("/api/vla/record/start")
async def vla_record_start(req: VLARecordStart):
    try:
        rec = _get_vla_recorder()
        # Auto-enter drag mode for kinesthetic teaching.
        try:
            _get_client().start_drag()
        except Exception as e:
            logging.warning("start_drag failed (continuing): %s", e)
        ep_dir = rec.start(req.instruction, episode_name=req.name)
        return {"success": True, "episode_dir": str(ep_dir)}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/vla/record/stop")
async def vla_record_stop():
    try:
        rec = _get_vla_recorder()
        sealed = rec.stop()
        try:
            _get_client().stop_drag()
        except Exception:
            pass
        return {"success": True, "episode_dir": str(sealed) if sealed else None}
    except Exception as e:
        return {"success": False, "error": str(e)}


class VLAExecuteStart(BaseModel):
    instruction: str
    server_url: Optional[str] = None
    servo_rate_hz: Optional[float] = None
    model_rate_hz: Optional[float] = None
    chunk_actions_to_execute: Optional[int] = None
    unnorm_key: Optional[str] = None


@app.post("/api/vla/execute/start")
async def vla_execute_start(req: VLAExecuteStart):
    global _vla_executor
    try:
        with _vla_lock:
            if _vla_executor is not None and _vla_executor.is_running():
                return {"success": False, "error": "executor already running"}

            from dobot_ros.vla.executor import VLAExecutor, ExecutorConfig
            from dobot_ros.vla.recorder import CameraFrameSource

            # Pull defaults from the settings store; request values override per-run.
            vla_cfg = _settings_store.get("vla")

            table_plane_path = str(Path(__file__).parent / "table_plane.json")
            if not Path(table_plane_path).exists():
                table_plane_path = None

            cfg = ExecutorConfig(
                instruction=req.instruction,
                server_url=req.server_url or vla_cfg.get("server_url") or VLA_OFT_URL,
                servo_rate_hz=req.servo_rate_hz if req.servo_rate_hz is not None else vla_cfg.get("servo_rate_hz", 30.0),
                model_rate_hz=req.model_rate_hz if req.model_rate_hz is not None else vla_cfg.get("model_rate_hz", 10.0),
                chunk_actions_to_execute=req.chunk_actions_to_execute if req.chunk_actions_to_execute is not None else vla_cfg.get("chunk_actions_to_execute", 4),
                gripper_threshold=vla_cfg.get("gripper_threshold", 0.1),
                table_plane_path=table_plane_path,
                unnorm_key=req.unnorm_key if req.unnorm_key is not None else vla_cfg.get("unnorm_key"),
            )
            _vla_executor = VLAExecutor(
                ros_client=_get_client(),
                config=cfg,
                camera=CameraFrameSource(camera_url=CAMERA_URL),
            )
            _vla_executor.start()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/vla/execute/stop")
async def vla_execute_stop():
    try:
        ex = _vla_executor
        if ex is None:
            return {"success": True, "note": "no executor"}
        ex.stop()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


# ── Servo Tester (ServoP streaming) ─────────────────────────────
#
# Additive: this is a manual tool for validating ServoP behavior. It does not
# replace any existing motion path (MovJ, MovL, jog, pick, calibrate all keep
# working exactly as before). See the servo/ module for details.

_servo_tester = None
_servo_lock = threading.Lock()


def _get_servo_tester():
    global _servo_tester
    with _servo_lock:
        if _servo_tester is None:
            from dobot_ros.servo.tester import ServoTester, ServoConfig
            from dobot_ros.vla.safety import SafetyLimits

            # Initial config from the settings store so last tuning persists.
            cfg_dict = _settings_store.get("servo")
            cfg = ServoConfig(**{
                k: v for k, v in cfg_dict.items()
                if k in ("servo_rate_hz", "t", "aheadtime", "gain", "idle_timeout_s")
            })

            # Workspace limits from the store's table_plane group (falls back
            # to the legacy file if that's all we have).
            tp = _settings_store.get("table_plane")
            if tp and (tp.get("corners") or tp.get("points")):
                tp_file = _SETTINGS_DIR / "table_plane_for_tester.json"
                tp_file.write_text(json.dumps(tp))
                limits = SafetyLimits.from_table_plane(tp_file)
            else:
                legacy = Path(__file__).parent / "table_plane.json"
                limits = SafetyLimits.from_table_plane(legacy) if legacy.exists() else SafetyLimits()

            _servo_tester = ServoTester(
                ros_client=_get_client(), config=cfg, limits=limits,
            )

            # Subscribe so future store updates push into the running tester.
            def _on_servo_change(new_cfg: dict):
                try:
                    _servo_tester.update_config(**{
                        k: v for k, v in new_cfg.items()
                        if k in ("servo_rate_hz", "t", "aheadtime", "gain", "idle_timeout_s")
                    })
                except Exception:
                    logging.exception("failed to apply servo settings update")

            _settings_store.subscribe("servo", _on_servo_change)
        return _servo_tester


class ServoStartRequest(BaseModel):
    servo_rate_hz: Optional[float] = None
    t: Optional[float] = None
    aheadtime: Optional[float] = None
    gain: Optional[float] = None
    csv_log: Optional[bool] = False


@app.post("/api/servo/start")
async def servo_start(req: ServoStartRequest):
    try:
        tester = _get_servo_tester()
        # Apply tuning before starting.
        kwargs = {k: v for k, v in req.dict().items()
                  if k in ("servo_rate_hz", "t", "aheadtime", "gain") and v is not None}
        if kwargs:
            tester.update_config(**kwargs)
        if req.csv_log:
            ts = int(time.time())
            tester.log_csv_path = f"/data/servo-logs/servo_{ts}.csv"
        status = tester.start()
        return {"success": True, "status": status.__dict__}
    except Exception as e:
        logging.exception("servo start failed")
        return {"success": False, "error": str(e)}


@app.post("/api/servo/stop")
async def servo_stop(call_robot_stop: bool = False):
    try:
        tester = _get_servo_tester()
        tester.stop(call_robot_stop=call_robot_stop)
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/servo/estop")
async def servo_estop():
    """Emergency stop: halt thread AND tell the controller to Stop."""
    try:
        tester = _get_servo_tester()
        tester.emergency_stop()
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


class ServoTargetRequest(BaseModel):
    offset: list  # 6-D: [dx, dy, dz, drx, dry, drz]


@app.post("/api/servo/target")
async def servo_target(req: ServoTargetRequest):
    try:
        tester = _get_servo_tester()
        status = tester.set_target_offset(req.offset)
        return {"success": True, "status": status.__dict__}
    except Exception as e:
        return {"success": False, "error": str(e)}


class ServoPatternRequest(BaseModel):
    name: str
    params: Optional[dict] = None


@app.post("/api/servo/pattern")
async def servo_pattern(req: ServoPatternRequest):
    try:
        from dobot_ros.servo.patterns import build_pattern
        tester = _get_servo_tester()
        pat = build_pattern(req.name, req.params or {})
        status = tester.set_pattern(pat, name=req.name)
        return {"success": True, "status": status.__dict__}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/servo/config")
async def servo_config(req: dict):
    try:
        # Route config updates through the settings store — this both persists
        # the value to last_settings.json and notifies the running tester via
        # the subscribe hook.
        filtered = {k: v for k, v in req.items()
                    if k in ("servo_rate_hz", "t", "aheadtime", "gain", "idle_timeout_s")}
        _settings_store.patch("servo", filtered)
        tester = _get_servo_tester()
        return {"success": True, "status": tester.status().__dict__}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.get("/api/servo/status")
async def servo_status():
    tester = _servo_tester
    if tester is None:
        return {"running": False, "initialized": False}
    st = tester.status()
    return {"running": st.running, "initialized": True, **st.__dict__}


# ── Settings store endpoints ─────────────────────────────────────
#
# All persistent settings (servo tuning, VLA defaults, table plane, camera
# view, gripper/jog defaults) are managed by the SettingsStore.
#
# Safety: /api/settings/saved/{name}/load is gated by _settings_load_safety_check.
# It refuses (HTTP 409) when the robot is moving, errored, or streaming.


@app.get("/api/settings/current")
async def settings_current():
    return {"settings": _settings_store.get_all()}


@app.patch("/api/settings/current")
async def settings_patch_current(req: dict):
    """Merge a partial update into current settings. Expected shape:
    {"servo": {"gain": 700}}  or  {"gripper": {"force": 30}, "jog": {"speed": 10}}
    """
    try:
        if not isinstance(req, dict):
            raise ValueError("body must be a JSON object keyed by group name")
        for group, partial in req.items():
            if not isinstance(partial, dict):
                raise ValueError(f"group '{group}' value must be an object")
            _settings_store.patch(group, partial)
        return {"success": True, "settings": _settings_store.get_all()}
    except Exception as e:
        return JSONResponse(status_code=400, content={"success": False, "error": str(e)})


@app.get("/api/settings/saved")
async def settings_list_saved():
    return {"saved": _settings_store.list_saved()}


class SettingsSaveRequest(BaseModel):
    name: str
    description: str = ""


@app.post("/api/settings/saved")
async def settings_save(req: SettingsSaveRequest):
    try:
        meta = _settings_store.save_current_as(req.name, req.description)
        return {"success": True, "saved": meta}
    except SettingsConflict as e:
        return JSONResponse(status_code=409, content={"success": False, "error": str(e)})
    except Exception as e:
        return JSONResponse(status_code=400, content={"success": False, "error": str(e)})


@app.get("/api/settings/saved/{name}")
async def settings_get_saved(name: str):
    try:
        return _settings_store.get_saved(name)
    except SettingsNotFound as e:
        raise HTTPException(status_code=404, detail=str(e))


@app.get("/api/settings/saved/{name}/download")
async def settings_download_saved(name: str):
    try:
        bundle = _settings_store.get_saved(name)
        content = json.dumps(bundle, indent=2)
        from fastapi.responses import Response
        return Response(
            content=content, media_type="application/json",
            headers={"Content-Disposition": f'attachment; filename="{name}.json"'},
        )
    except SettingsNotFound as e:
        raise HTTPException(status_code=404, detail=str(e))


class SettingsSavedPatch(BaseModel):
    new_name: Optional[str] = None
    description: Optional[str] = None


@app.patch("/api/settings/saved/{name}")
async def settings_patch_saved(name: str, req: SettingsSavedPatch):
    try:
        result = None
        if req.new_name is not None and req.new_name != name:
            result = _settings_store.rename_saved(name, req.new_name)
            name = req.new_name  # subsequent update uses new name
        if req.description is not None:
            result = _settings_store.update_description(name, req.description)
        if result is None:
            result = {"slug": name}
        return {"success": True, "saved": result}
    except SettingsNotFound as e:
        raise HTTPException(status_code=404, detail=str(e))
    except SettingsConflict as e:
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.delete("/api/settings/saved/{name}")
async def settings_delete_saved(name: str):
    try:
        _settings_store.delete_saved(name)
        return {"success": True}
    except SettingsNotFound as e:
        raise HTTPException(status_code=404, detail=str(e))


@app.post("/api/settings/saved/{name}/load")
async def settings_load_saved(name: str):
    """Load a saved bundle into current settings. Refuses (409) when the robot
    is in motion, errored, or a streaming control path is active."""
    try:
        _settings_store.load_saved(name, safety_check=_settings_load_safety_check)
        return {"success": True, "settings": _settings_store.get_all()}
    except SettingsBlocked as e:
        return JSONResponse(
            status_code=409,
            content={"success": False, "error": "blocked by safety check", "reasons": e.reasons},
        )
    except SettingsNotFound as e:
        return JSONResponse(status_code=404, content={"success": False, "error": str(e)})
    except Exception as e:
        return JSONResponse(status_code=400, content={"success": False, "error": str(e)})


@app.post("/api/settings/saved/import")
async def settings_import_saved(bundle: dict):
    """Import a previously downloaded JSON bundle. Body is the JSON directly
    (no multipart needed — the frontend reads the file and POSTs the parsed JSON)."""
    try:
        meta = _settings_store.import_bundle(bundle)
        return {"success": True, "saved": meta}
    except Exception as e:
        return JSONResponse(status_code=400, content={"success": False, "error": str(e)})


# ── WebSocket ───────────────────────────────────────────────────

@app.websocket("/ws/state")
async def ws_state(websocket: WebSocket):
    """WebSocket endpoint for real-time robot state at ~5Hz."""
    await websocket.accept()
    _ws_clients.add(websocket)
    try:
        while True:
            with _lock:
                mode_names = {
                    1: "INIT", 2: "BRAKE_OPEN", 3: "POWEROFF",
                    4: "DISABLED", 5: "ENABLE", 6: "BACKDRIVE",
                    7: "RUNNING", 8: "SINGLE_STEP", 9: "ERROR",
                    10: "PAUSE", 11: "COLLISION",
                }
                data = {
                    "connected": _state["connected"],
                    "robot_mode": _state["robot_mode"],
                    "robot_mode_name": mode_names.get(_state["robot_mode"], "UNKNOWN"),
                    "joint": _state["joint"],
                    "cartesian": _state["cartesian"],
                    "gripper_position": _state["gripper_position"],
                    "gripper_state": _state["gripper_state"],
                    "gripper_initialized": _state["gripper_initialized"],
                    "speed_factor": _state["speed_factor"],
                    "error": _state["error"],
                    "timestamp": time.time(),
                }
            await websocket.send_json(data)
            await asyncio.sleep(0.2)  # 5 Hz
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        _ws_clients.discard(websocket)


# ── Static files ────────────────────────────────────────────────

app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


# Dev convenience: the HTML/JS/CSS in static/ changes often during active
# development; serving the HTML entry points with no-cache keeps iframe and
# browser caches from pinning stale copies.
_NO_CACHE_HEADERS = {
    "Cache-Control": "no-cache, no-store, must-revalidate",
    "Pragma": "no-cache",
    "Expires": "0",
}


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"), headers=_NO_CACHE_HEADERS)


@app.get("/pendant")
async def pendant():
    return FileResponse(str(STATIC_DIR / "pendant.html"), headers=_NO_CACHE_HEADERS)


@app.middleware("http")
async def _no_cache_js(request, call_next):
    """Also prevent caching of our own JS/CSS so edits during development show up
    immediately. Static third-party assets from CDN are unaffected."""
    response = await call_next(request)
    path = request.url.path
    if path.startswith("/static/js/") or path.startswith("/static/css/"):
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response


# ── Run ─────────────────────────────────────────────────────────

def start_server(config: Config, host: str = "0.0.0.0", port: int = 8080):
    """Start the web server."""
    import uvicorn
    global _config
    _config = config
    uvicorn.run(app, host=host, port=port, log_level="info")
