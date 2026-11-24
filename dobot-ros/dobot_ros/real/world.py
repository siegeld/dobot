"""RealWorld — World protocol over the camera server's object detection.

XY comes from projecting the detection centre onto the calibrated table
plane (VisionTransform.pixel_to_table), exactly as /api/vision/plan does;
camera depth is NOT trusted for Z. Colour is named from the mean hue of
the detection's bounding box in the colour frame when the detector does
not supply one.
"""

from __future__ import annotations

import colorsys
import io
import logging
import math
from typing import Any, Dict, List, Optional

import requests

from dobot_ros.policies.types import WorldObject

log = logging.getLogger(__name__)


def color_name_from_rgb(r: float, g: float, b: float) -> str:
    h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    if v < 0.18:
        return "black"
    if s < 0.18:
        return "white" if v > 0.7 else "gray"
    deg = h * 360.0
    if deg < 15 or deg >= 345: return "red"
    if deg < 45: return "orange"
    if deg < 70: return "yellow"
    if deg < 170: return "green"
    if deg < 200: return "cyan"
    if deg < 260: return "blue"
    if deg < 300: return "purple"
    return "pink"


class RealWorld:
    def __init__(
        self,
        camera_url: str,
        vision_transform,
        table_z: float,
        min_clearance_mm: float,
        tool_length_mm: float,
        default_height_mm: float = 40.0,
        timeout_s: float = 3.0,
    ):
        self.camera_url = camera_url.rstrip("/")
        self.vt = vision_transform
        self.table_z = float(table_z)
        self.min_clearance_mm = float(min_clearance_mm)
        self.tool_length_mm = float(tool_length_mm)
        self.default_height_mm = float(default_height_mm)
        self.timeout = timeout_s
        self._last_raw: List[dict] = []

    @property
    def surface_z(self) -> float:
        return self.table_z - self.tool_length_mm

    def objects(self) -> List[WorldObject]:
        resp = requests.get(f"{self.camera_url}/api/detection/objects", timeout=self.timeout).json()
        raw = resp.get("objects", []) or []
        self._last_raw = raw
        frame = None
        if any("color" not in o for o in raw):
            frame = self._color_frame()
        out = []
        for o in raw:
            try:
                out.append(self._to_object(o, frame))
            except Exception as e:
                log.debug("skip detection %s: %s", o.get("id"), e)
        return out

    def find(self, **query: Any) -> Optional[WorldObject]:
        best = None
        for o in self.objects():
            if all(_match(getattr(o, k, o.attrs.get(k)), v) for k, v in query.items()):
                if best is None or o.confidence > best.confidence:
                    best = o
        return best

    # ── internals ───────────────────────────────────────────────────
    def _to_object(self, o: dict, frame) -> WorldObject:
        px, py = o["center_px"]
        x, y = self.vt.pixel_to_table(float(px), float(py))
        yaw = self.vt.image_rot_to_table_rot(float(o.get("rotation_deg", 0.0)), float(px), float(py))
        # Footprint from the bbox corners projected onto the table.
        lx, ly = 40.0, 40.0
        bbox = o.get("bbox_px") or o.get("bbox")
        if bbox and len(bbox) == 4:
            bx, by, bw, bh = [float(v) for v in bbox]
            if bw > 0 and bh > 0:
                corners = [self.vt.pixel_to_table(bx + dx, by + dy)
                           for dx, dy in ((0, 0), (bw, 0), (0, bh), (bw, bh))]
                xs = [c[0] for c in corners]; ys = [c[1] for c in corners]
                diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                # Axis-aligned extents of a yawed box overestimate; split the
                # diagonal by the detector's aspect ratio.
                ar = max(bw, bh) / max(1.0, min(bw, bh))
                short = diag / math.sqrt(1 + ar * ar)
                lx, ly = max(10.0, short * ar), max(10.0, short)
        h = float(o.get("height_mm") or self.default_height_mm)
        color = o.get("color")
        if not color and frame is not None and bbox and len(bbox) == 4:
            color = self._mean_color(frame, bbox)
        conf = float(o.get("confidence", 0.8))
        return WorldObject(
            id=str(o.get("id", f"det_{px}_{py}")), label=str(o.get("label", "block")).lower(),
            color=(color or "unknown").lower(),
            position=[x, y, self.surface_z + h / 2.0], size=[lx, ly, h], yaw_deg=yaw,
            confidence=conf, support="table",
            attrs={"center_px": [px, py], "area_px": o.get("area_px"),
                   "frames_tracked": o.get("frames_tracked")},
        )

    def _color_frame(self):
        try:
            from PIL import Image
            r = requests.get(f"{self.camera_url}/api/frame/color", timeout=self.timeout)
            return Image.open(io.BytesIO(r.content)).convert("RGB")
        except Exception as e:
            log.debug("color frame unavailable: %s", e)
            return None

    @staticmethod
    def _mean_color(frame, bbox) -> str:
        bx, by, bw, bh = [int(v) for v in bbox]
        # Sample the central third of the box to avoid background bleed.
        cx0, cy0 = bx + bw // 3, by + bh // 3
        crop = frame.crop((cx0, cy0, max(cx0 + 1, bx + 2 * bw // 3), max(cy0 + 1, by + 2 * bh // 3)))
        px = list(crop.getdata())
        if not px:
            return "unknown"
        n = len(px)
        r = sum(p[0] for p in px) / n; g = sum(p[1] for p in px) / n; b = sum(p[2] for p in px) / n
        return color_name_from_rgb(r, g, b)


def _match(a: Any, b: Any) -> bool:
    if isinstance(a, str) and isinstance(b, str):
        return a.lower() == b.lower()
    return a == b
