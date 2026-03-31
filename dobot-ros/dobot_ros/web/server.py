"""
FastAPI web server for Dobot CR5 robot control.

Provides REST API + WebSocket for real-time monitoring and control.
"""

import asyncio
import json
import math
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
from dobot_ros.config import Config
from dobot_ros.ros_client import DobotRosClient


# ── Global state ────────────────────────────────────────────────

_client: Optional[DobotRosClient] = None
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
    global _client
    if _client is None:
        if not rclpy.ok():
            rclpy.init()
        _client = DobotRosClient(
            namespace=_config.ros_namespace if _config else '',
            service_timeout=_config.service_timeout if _config else 5.0,
            subscribe_topics=True,
        )
    return _client


def _poll_state():
    """Poll robot state in a background thread.

    Robot state and gripper state come from ROS2 topic subscriptions
    cached in the DobotRosClient. We call spin_once to process callbacks.
    """
    while True:
        try:
            client = _get_client()
            # Process topic subscription callbacks
            rclpy.spin_once(client, timeout_sec=0)
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
    global _client
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
            1: "INIT", 2: "BRAKE_OPEN", 3: "DISABLED",
            4: "ENABLE", 5: "BACKDRIVE", 6: "RUNNING",
            7: "RECORDING", 8: "ERROR", 9: "PAUSE",
            10: "JOG", 11: "TEACH",
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
        with _lock:
            res = client.enable_robot()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/disable")
async def disable_robot():
    """Disable the robot."""
    try:
        client = _get_client()
        with _lock:
            res = client.disable_robot()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/clear")
async def clear_error():
    """Clear robot errors."""
    try:
        client = _get_client()
        with _lock:
            res = client.clear_error()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/stop")
async def stop_robot():
    """Emergency stop."""
    try:
        client = _get_client()
        with _lock:
            res = client.stop()
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/speed")
async def set_speed(req: SpeedRequest):
    """Set global speed factor."""
    try:
        client = _get_client()
        with _lock:
            res = client.set_speed_factor(req.speed)
            _state["speed_factor"] = req.speed
        return {"success": True, "result": res}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/jog")
async def jog_robot(req: JogRequest):
    """Jog robot by relative distance."""
    try:
        client = _get_client()
        with _lock:
            # Set speed
            client.set_speed_factor(req.speed)
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
        with _lock:
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
        with _lock:
            client.gripper_init()
            _state["gripper_initialized"] = True
        return {"success": True}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/open")
async def gripper_open(speed: int = 50, force: int = 50):
    """Open the gripper."""
    try:
        client = _get_client()
        with _lock:
            state = client.gripper_open(force=force, speed=speed)
        state_names = {0: "moving", 1: "reached", 2: "caught", 3: "dropped", -1: "timeout"}
        return {"success": True, "state": state, "state_name": state_names.get(state, str(state))}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/close")
async def gripper_close(speed: int = 50, force: int = 50):
    """Close the gripper."""
    try:
        client = _get_client()
        with _lock:
            state = client.gripper_close(force=force, speed=speed)
        state_names = {0: "moving", 1: "reached", 2: "caught", 3: "dropped", -1: "timeout"}
        return {"success": True, "state": state, "state_name": state_names.get(state, str(state))}
    except Exception as e:
        return {"success": False, "error": str(e)}


@app.post("/api/gripper/move")
async def gripper_move(req: GripperMoveRequest):
    """Move gripper to position."""
    try:
        client = _get_client()
        with _lock:
            state = client.gripper_move(req.position, force=req.force, speed=req.speed)
        state_names = {0: "moving", 1: "reached", 2: "caught", 3: "dropped", -1: "timeout"}
        return {"success": True, "state": state, "state_name": state_names.get(state, str(state))}
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
            "robot_ip": _config.robot_ip,
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
                    1: "INIT", 2: "BRAKE_OPEN", 3: "DISABLED",
                    4: "ENABLE", 5: "BACKDRIVE", 6: "RUNNING",
                    7: "RECORDING", 8: "ERROR", 9: "PAUSE",
                    10: "JOG", 11: "TEACH",
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
