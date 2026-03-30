"""Vision client — queries the RealSense camera server REST API."""

import logging
from dataclasses import dataclass
from typing import List, Optional

import requests

logger = logging.getLogger(__name__)

DEFAULT_CAMERA_URL = "http://10.11.6.65:8080"


@dataclass
class DetectedObject:
    """Object detected by the camera."""
    id: int
    label: str
    bbox: List[int]          # [x, y, w, h] pixels
    center_px: List[int]     # [x, y] pixels
    depth_mean: float        # meters
    depth_min: float
    depth_max: float
    position_3d: List[float] # [x, y, z] meters in camera frame
    area_px: int
    rotation_deg: float      # orientation angle in degrees
    frames_tracked: int

    @property
    def x(self) -> float:
        return self.position_3d[0]

    @property
    def y(self) -> float:
        return self.position_3d[1]

    @property
    def z(self) -> float:
        return self.position_3d[2]


@dataclass
class RobotPoint:
    """A point in robot base frame."""
    x: float  # mm
    y: float  # mm
    z: float  # mm


class VisionClient:
    """Client for the RealSense camera server REST API."""

    def __init__(self, base_url: str = DEFAULT_CAMERA_URL, timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def _get(self, path: str, params: dict = None) -> dict:
        resp = requests.get(f"{self.base_url}{path}", params=params, timeout=self.timeout)
        resp.raise_for_status()
        return resp.json()

    def _post(self, path: str, json_data: dict = None) -> dict:
        resp = requests.post(f"{self.base_url}{path}", json=json_data, timeout=self.timeout)
        resp.raise_for_status()
        return resp.json()

    def get_objects(self) -> List[DetectedObject]:
        """Get all currently detected objects."""
        data = self._get("/api/detection/objects")
        return [DetectedObject(**obj) for obj in data.get("objects", [])]

    def get_object(self, object_id: int) -> Optional[DetectedObject]:
        """Get a specific object by ID. Returns None if not found."""
        objects = self.get_objects()
        for obj in objects:
            if obj.id == object_id:
                return obj
        return None

    def get_depth_at(self, x: int, y: int) -> dict:
        """Get depth and 3D position at a pixel."""
        return self._get("/api/depth/at", params={"x": x, "y": y})

    def transform_to_robot(self, camera_xyz: List[float]) -> RobotPoint:
        """Transform a camera-frame point to robot base frame.

        Requires calibration to be solved on the camera server.
        """
        data = self._get("/api/calibration/transform", params={
            "x": camera_xyz[0],
            "y": camera_xyz[1],
            "z": camera_xyz[2],
        })
        xyz = data["robot_xyz"]
        return RobotPoint(x=xyz[0], y=xyz[1], z=xyz[2])

    def get_calibration_status(self) -> dict:
        """Get calibration status."""
        return self._get("/api/calibration")

    def add_calibration_point(self, camera_xyz: List[float], robot_xyz: List[float]) -> dict:
        """Record a calibration correspondence point."""
        return self._post("/api/calibration/point", {
            "camera_xyz": camera_xyz,
            "robot_xyz": robot_xyz,
        })

    def solve_calibration(self) -> dict:
        """Solve the camera-to-robot transform."""
        return self._post("/api/calibration/solve")

    def clear_calibration(self) -> dict:
        """Clear all calibration points."""
        return self._post("/api/calibration/clear")

    def reset_detection(self) -> dict:
        """Reset the background model."""
        return self._post("/api/detection/reset")

    def is_connected(self) -> bool:
        """Check if camera server is reachable."""
        try:
            data = self._get("/api/status")
            return data.get("streaming", False)
        except Exception:
            return False
