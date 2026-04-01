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

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel

import rclpy
from rclpy.executors import MultiThreadedExecutor
from dobot_ros.config import Config
from dobot_ros.ros_client import DobotRosClient


# ── Global state ────────────────────────────────────────────────

_client: Optional[DobotRosClient] = None
_executor: Optional[MultiThreadedExecutor] = None
_config: Optional[Config] = None
_lock = threading.Lock()
_ws_clients: set = set()

# Cached state for polling
_state = {
    "connected": False,
    "robot_mode": -1,
    "joint": [0.0] * 6,
    "cartesian": [0.0] * 6,
    "gripper_position": -1,
    "gripper_state": -1,
    "gripper_initialized": False,
    "speed_factor": 50,
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
                    _state["joint"] = angles
                    _state["cartesian"] = pose
                    _state["robot_mode"] = mode
                    _state["connected"] = True
                    _state["error"] = None
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
        # Watchdog: if feedback port (30004) is dead, kill the driver node
        # so the restart loop in the container relaunches it
        try:
            if client.is_feedback_stale(max_age=5.0):
                logging.warning('Feedback port stale — signaling driver restart')
                Path('/tmp/dobot-shared/driver_restart').touch()
                time.sleep(10)  # wait for driver to restart
        except Exception:
            pass
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
    """Set global speed factor."""
    try:
        client = _get_client()
        res = client.set_speed_factor(req.speed)
        with _lock:
            _state["speed_factor"] = req.speed
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


@app.post("/api/calibration/table/clear")
async def clear_table():
    """Clear table plane data."""
    try:
        _TABLE_FILE.write_text(json.dumps({"points": [], "plane": None}))
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


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


# ── Camera State ─────────────────────────────────────────────────

_CAMERA_FILE = Path(__file__).parent / "camera_state.json"


@app.get("/api/camera")
async def get_camera():
    """Get saved camera view."""
    try:
        if _CAMERA_FILE.exists():
            return json.loads(_CAMERA_FILE.read_text())
    except Exception:
        pass
    return {}


@app.post("/api/camera")
async def save_camera(request: dict):
    """Save camera view (position + target)."""
    try:
        _CAMERA_FILE.write_text(json.dumps(request, indent=2))
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


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


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/pendant")
async def pendant():
    return FileResponse(str(STATIC_DIR / "pendant.html"))


# ── Run ─────────────────────────────────────────────────────────

def start_server(config: Config, host: str = "0.0.0.0", port: int = 8080):
    """Start the web server."""
    import uvicorn
    global _config
    _config = config
    uvicorn.run(app, host=host, port=port, log_level="info")
