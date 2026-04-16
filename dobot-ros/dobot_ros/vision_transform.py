"""Pixel-ray-to-table-plane projection for oblique camera setups.

Core insight: when the camera views the table from an angle, using depth to
compute XY introduces large errors (±38mm depth noise → ±20-38mm XY at 30-45°).
Instead, cast a ray from the camera through the pixel and intersect it with the
known table plane — this gives precise XY with no depth dependency.

Depth is used only as a SECONDARY signal for:
- Object presence verification (is something above the table at this pixel?)
- Object height estimation (for grasp Z refinement)
- Calibration drift detection

Coordinate frames:
- Camera frame: RealSense convention — Z forward, X right, Y down (meters)
- Robot frame: Dobot base frame — X forward, Y left, Z up (millimeters)
- Pixel: (px, py) from top-left, integer

Usage:
    vt = VisionTransform.solve_from_points(correspondences, intrinsics, table_z)
    robot_xy = vt.pixel_to_table(px, py)
    true_rot = vt.image_rot_to_table_rot(image_rot_deg, cx, cy)
    height = vt.object_height_mm(expected_depth, measured_depth)
    conf = vt.pick_confidence(obj_data)
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

log = logging.getLogger(__name__)


# ── Data classes ────────────────────────────────────────────────────────

@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters (from RealSense factory calibration)."""
    fx: float  # focal length X (pixels)
    fy: float  # focal length Y (pixels)
    cx: float  # principal point X (pixels)
    cy: float  # principal point Y (pixels)
    width: int = 640
    height: int = 360

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "CameraIntrinsics":
        return cls(**{k: d[k] for k in ("fx", "fy", "cx", "cy", "width", "height") if k in d})


@dataclass
class Extrinsics:
    """Camera-to-robot rigid transform: p_robot = R @ p_camera + t.

    R: 3x3 rotation matrix.
    t: 3x1 translation vector (camera origin in robot frame, mm).
    """
    R: np.ndarray  # (3, 3)
    t: np.ndarray  # (3,)

    def to_dict(self) -> dict:
        return {
            "R": self.R.tolist(),
            "t": self.t.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Extrinsics":
        return cls(R=np.array(d["R"]), t=np.array(d["t"]))


@dataclass
class PickConfidence:
    """Multi-signal confidence for a pick target."""
    score: int = 0
    max_score: int = 6
    level: str = "red"  # "green" | "yellow" | "red"
    signals: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"score": self.score, "max": self.max_score,
                "level": self.level, "signals": self.signals}


# ── VisionTransform ────────────────────────────────────────────────────

class VisionTransform:
    """Pixel-ray-to-table-plane projection with depth sanity checks."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        extrinsics: Extrinsics,
        table_z: float,
        min_clearance_mm: float = 25.0,
    ):
        self.intrinsics = intrinsics
        self.extrinsics = extrinsics
        self.table_z = table_z
        self.min_clearance_mm = min_clearance_mm

        # Precompute the camera origin in robot frame.
        self._cam_origin_robot = extrinsics.t.copy()

    # ── Primary: ray-plane projection (no depth needed) ─────────

    def _pixel_to_ray_camera(self, px: float, py: float) -> np.ndarray:
        """Convert a pixel to a unit direction vector in camera frame."""
        intr = self.intrinsics
        dx = (px - intr.cx) / intr.fx
        dy = (py - intr.cy) / intr.fy
        ray = np.array([dx, dy, 1.0])
        return ray / np.linalg.norm(ray)

    def _ray_camera_to_robot(self, ray_camera: np.ndarray) -> np.ndarray:
        """Rotate a direction from camera frame to robot frame."""
        return self.extrinsics.R @ ray_camera

    def pixel_to_table(self, px: float, py: float) -> Tuple[float, float]:
        """Project a pixel onto the table plane (Z = table_z in robot frame).

        Returns (robot_x, robot_y) in mm. No depth measurement needed —
        uses geometry of the known camera pose and table plane.

        Raises ValueError if the ray doesn't intersect the table (camera
        pointing away from the table plane).
        """
        ray_cam = self._pixel_to_ray_camera(px, py)
        ray_rob = self._ray_camera_to_robot(ray_cam)
        origin = self._cam_origin_robot

        # Intersect ray with plane Z = table_z.
        # origin.z + t * ray_rob.z = table_z → t = (table_z - origin.z) / ray_rob.z
        if abs(ray_rob[2]) < 1e-9:
            raise ValueError("ray is parallel to the table plane")

        t_param = (self.table_z - origin[2]) / ray_rob[2]
        if t_param < 0:
            raise ValueError("table plane is behind the camera")

        hit_x = origin[0] + t_param * ray_rob[0]
        hit_y = origin[1] + t_param * ray_rob[1]
        return float(hit_x), float(hit_y)

    def image_rot_to_table_rot(
        self, rot_deg: float, center_px: float, center_py: float,
    ) -> float:
        """Convert an image-plane rotation to a table-plane rotation.

        The image-plane rotation (from object detection) is distorted by
        perspective when the camera is oblique. We project two points on
        the object's principal axis to the table plane and compute the
        true angle in robot XY.

        Returns rotation in degrees, range [-180, 180].
        """
        r = math.radians(rot_deg)
        # Two points along the axis in pixel coords, 20px apart.
        offset = 20.0
        p1_px = center_px - offset * math.cos(r)
        p1_py = center_py - offset * math.sin(r)
        p2_px = center_px + offset * math.cos(r)
        p2_py = center_py + offset * math.sin(r)

        try:
            x1, y1 = self.pixel_to_table(p1_px, p1_py)
            x2, y2 = self.pixel_to_table(p2_px, p2_py)
        except ValueError:
            return rot_deg  # fallback to uncorrected if projection fails

        true_rot = math.degrees(math.atan2(y2 - y1, x2 - x1))
        return true_rot

    def robot_to_pixel(self, robot_x: float, robot_y: float, robot_z: Optional[float] = None) -> Tuple[float, float]:
        """Inverse: project a robot-frame point back to pixel coordinates.

        If robot_z is None, uses table_z. Useful for drawing overlays on
        the camera image showing where robot-frame positions map.
        """
        if robot_z is None:
            robot_z = self.table_z
        p_robot = np.array([robot_x, robot_y, robot_z])

        # p_camera = R^-1 @ (p_robot - t)
        R_inv = self.extrinsics.R.T
        p_camera = R_inv @ (p_robot - self.extrinsics.t)

        if abs(p_camera[2]) < 1e-9:
            return (0.0, 0.0)  # degenerate — point is at the camera

        intr = self.intrinsics
        px = intr.fx * (p_camera[0] / p_camera[2]) + intr.cx
        py = intr.fy * (p_camera[1] / p_camera[2]) + intr.cy
        return float(px), float(py)

    # ── Depth-based sanity checks ───────────────────────────────

    def expected_depth_at_pixel(self, px: float, py: float) -> float:
        """How far (meters) from the camera to the table plane at this pixel.

        Used to compare against measured depth for object presence / height.
        """
        ray_cam = self._pixel_to_ray_camera(px, py)
        ray_rob = self._ray_camera_to_robot(ray_cam)
        origin = self._cam_origin_robot

        if abs(ray_rob[2]) < 1e-9:
            return 0.0

        t_param = (self.table_z - origin[2]) / ray_rob[2]
        # t_param is in robot-frame scale (mm); the ray_cam is normalized.
        # Distance in camera frame = t_param * |ray_cam| / |ray_rob| but since
        # both are unit vectors after normalization... actually t_param is already
        # the distance along ray_rob. Convert from mm to meters.
        # The actual 3D distance in camera frame:
        hit_robot = origin + t_param * ray_rob
        p_camera_space = self.extrinsics.R.T @ (hit_robot - self.extrinsics.t)
        return float(np.linalg.norm(p_camera_space)) / 1000.0  # mm → meters

    def object_height_mm(self, expected_depth_m: float, measured_depth_m: float) -> float:
        """Estimate object height above the table from the depth delta.

        Objects closer to the camera than the table surface protrude upward.
        Positive = object sticks up. Negative = below table (shouldn't happen).
        """
        # Delta in meters along the camera's optical axis; convert to mm and
        # project onto the table normal (≈ vertical for a level table).
        # For a roughly vertical table normal, height ≈ delta * cos(viewing_angle).
        # Approximation: just use the raw delta in mm. Good enough for our
        # height-binning use case (flat / short / tall).
        return (expected_depth_m - measured_depth_m) * 1000.0

    def pick_confidence(
        self,
        object_present: bool,
        depth_consistency_mm: float,
        frames_tracked: int,
        in_workspace: bool,
        area_px: int,
        calibration_age_s: float = 0.0,
    ) -> PickConfidence:
        """Combine multiple signals into a pick confidence score."""
        signals = {}

        def signal(name, green_cond, yellow_cond):
            if green_cond:
                signals[name] = "green"
                return 1
            if yellow_cond:
                signals[name] = "yellow"
                return 0
            signals[name] = "red"
            return 0

        score = 0
        score += signal("presence", object_present, False)
        score += signal("depth_consistency", depth_consistency_mm < 15, depth_consistency_mm < 30)
        score += signal("tracking", frames_tracked > 5, frames_tracked > 2)
        score += signal("workspace", in_workspace, True)
        score += signal("size", area_px > 500, area_px > 200)
        score += signal("calibration", calibration_age_s < 3600, calibration_age_s < 28800)

        level = "green" if score >= 5 else ("yellow" if score >= 3 else "red")
        return PickConfidence(score=score, max_score=6, level=level, signals=signals)

    # ── Solve from correspondence points ────────────────────────

    @classmethod
    def solve_from_points(
        cls,
        correspondences: List[Dict],
        intrinsics: CameraIntrinsics,
        table_z: float,
        min_clearance_mm: float = 25.0,
    ) -> Tuple["VisionTransform", float]:
        """Solve the camera-to-robot extrinsic transform from correspondence
        points using SVD (Kabsch algorithm).

        Each correspondence: {"camera_xyz": [x,y,z] meters, "robot_xyz": [x,y,z] mm}

        Returns (VisionTransform, error_mm) where error_mm is the mean
        residual across all points.
        """
        if len(correspondences) < 3:
            raise ValueError(f"need ≥3 correspondence points, got {len(correspondences)}")

        # Extract point sets. Camera is in meters — convert to mm.
        cam_pts = np.array([c["camera_xyz"] for c in correspondences]) * 1000.0  # → mm
        rob_pts = np.array([c["robot_xyz"] for c in correspondences])

        # Kabsch: find R, t such that rob_pts ≈ R @ cam_pts + t
        cam_centroid = cam_pts.mean(axis=0)
        rob_centroid = rob_pts.mean(axis=0)
        cam_centered = cam_pts - cam_centroid
        rob_centered = rob_pts - rob_centroid

        H = cam_centered.T @ rob_centered
        U, S, Vt = np.linalg.svd(H)

        # Check for degenerate (near-collinear) points. For table-plane
        # calibration, points are inherently planar (one small singular value
        # is expected). Collinear means TWO small values — check the second.
        if len(S) >= 2 and S[-2] < 1e-6:
            raise ValueError(
                "correspondence points are near-collinear (degenerate). "
                "Record points that are NOT all on a straight line."
            )
        d = np.linalg.det(Vt.T @ U.T)
        sign_matrix = np.diag([1, 1, d])
        R = Vt.T @ sign_matrix @ U.T
        t = rob_centroid - R @ cam_centroid

        # Residual error.
        transformed = (R @ cam_pts.T).T + t
        errors = np.linalg.norm(transformed - rob_pts, axis=1)
        mean_error = float(errors.mean())

        extrinsics = Extrinsics(R=R, t=t)
        vt = cls(intrinsics, extrinsics, table_z, min_clearance_mm)
        return vt, mean_error

    # ── Serialization ───────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "intrinsics": self.intrinsics.to_dict(),
            "extrinsics": self.extrinsics.to_dict(),
            "table_z": self.table_z,
            "min_clearance_mm": self.min_clearance_mm,
        }

    def save(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2))
        log.info("Saved vision transform to %s", path)

    @classmethod
    def load(cls, path: Path) -> "VisionTransform":
        d = json.loads(Path(path).read_text())
        return cls(
            intrinsics=CameraIntrinsics.from_dict(d["intrinsics"]),
            extrinsics=Extrinsics.from_dict(d["extrinsics"]),
            table_z=d["table_z"],
            min_clearance_mm=d.get("min_clearance_mm", 25.0),
        )
