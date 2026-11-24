#!/usr/bin/env python3
"""Validate the Python CR5 kinematics against the live robot.

Samples (joints, pose) pairs from the running web dashboard while you jog
the arm around (vary the wrist orientation!), then reports:
  * translational FK error (independent of any Euler convention)
  * which of the 12 Euler conventions matches the controller's RX/RY/RZ
  * the constant flange yaw offset that best explains the residual

Usage (host, robot powered on, dashboard at :7070):
    python3 scripts/validate_fk.py --seconds 60
Move the arm through 10+ distinct poses while it samples.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from pathlib import Path

import numpy as np
import requests

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dobot-ros"))
from dobot_ros.sim import kinematics as K  # noqa: E402


def euler_matrix(angles_deg, order: str, intrinsic: bool) -> np.ndarray:
    fns = {"x": K.rot_x, "y": K.rot_y, "z": K.rot_z}
    a = [math.radians(v) for v in angles_deg]
    mats = [fns[ax](ang) for ax, ang in zip(order, a)]
    R = np.eye(3)
    if intrinsic:
        for m in mats:
            R = R @ m
    else:
        for m in mats:
            R = m @ R
    return R


def rot_err_deg(Ra, Rb) -> float:
    c = max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def sample(base_url: str, seconds: float, period: float = 0.5):
    pairs = []
    last = None
    t_end = time.time() + seconds
    print(f"sampling for {seconds:.0f}s — jog the arm through varied poses…")
    while time.time() < t_end:
        try:
            d = requests.get(f"{base_url}/api/position", timeout=2).json()
            j = [d["joint"][k] for k in ("J1", "J2", "J3", "J4", "J5", "J6")]
            p = [d["cartesian"][k] for k in ("X", "Y", "Z", "RX", "RY", "RZ")]
            if any(abs(v) > 1e-6 for v in j) and (last is None or max(abs(a - b) for a, b in zip(j, last)) > 0.5):
                pairs.append((j, p))
                last = j
                print(f"  #{len(pairs)} J={[round(v,1) for v in j]} P={[round(v,1) for v in p]}")
        except Exception as e:
            print("  read failed:", e)
        time.sleep(period)
    return pairs


def analyse(pairs, tool_length_mm: float):
    if len(pairs) < 3:
        print("need at least 3 distinct poses"); return
    # 1. translation
    errs = []
    for j, p in pairs:
        f = K.fk(j, tool_length_mm)
        errs.append(math.dist(f[:3], p[:3]))
    print(f"\nTranslation error: max {max(errs):.2f} mm, mean {sum(errs)/len(errs):.2f} mm "
          f"(tool_length={tool_length_mm})")
    if max(errs) > 5:
        print("  >5 mm: wrong tool length (try --tool 0 / --tool 203) or the pose frame differs")

    # 2. Euler convention — angles are RX,RY,RZ; try all orders × intrinsic/extrinsic
    results = []
    for order in ["".join(o) for o in itertools.permutations("xyz")]:
        for intrinsic in (True, False):
            # Map letters to the RX/RY/RZ values.
            idx = {"x": 0, "y": 1, "z": 2}
            es = []
            for j, p in pairs:
                R_fk = K.fk_matrix(j, tool_length_mm)[:3, :3]
                R_rep = euler_matrix([p[3 + idx[c]] for c in order], order, intrinsic)
                es.append(rot_err_deg(R_fk, R_rep))
            results.append((max(es), order, intrinsic))
    results.sort()
    print("\nEuler convention fit (max rotation error, order, intrinsic?):")
    for r in results[:4]:
        print(f"  {r[0]:7.2f}°  order={r[1]}  {'intrinsic' if r[2] else 'extrinsic'}")
    best = results[0]
    print(f"\nkinematics.py assumes extrinsic xyz (R = Rz·Ry·Rx). "
          f"{'MATCHES' if (best[1], best[2]) == ('xyz', False) and best[0] < 2 else 'DOES NOT MATCH — update rpy_to_matrix/matrix_to_rpy'}")

    # 3. yaw offset for the best convention
    best_delta, best_e = 0.0, 1e9
    for delta in np.arange(-180, 180, 1.0):
        Rd = K.rot_z(math.radians(delta))
        es = []
        idx = {"x": 0, "y": 1, "z": 2}
        for j, p in pairs:
            R_fk = K.fk_matrix(j, tool_length_mm)[:3, :3] @ Rd
            R_rep = euler_matrix([p[3 + idx[c]] for c in best[1]], best[1], best[2])
            es.append(rot_err_deg(R_fk, R_rep))
        if max(es) < best_e:
            best_e, best_delta = max(es), delta
    print(f"\nBest flange yaw offset: {best_delta:+.0f}° (residual {best_e:.2f}°). "
          f"Set FLANGE_YAW_OFFSET_DEG in dobot_ros/sim/kinematics.py if ≠ 0.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:7070")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--tool", type=float, default=None,
                    help="tool length the controller currently reports in (0 or 203). Default: read /api/status")
    args = ap.parse_args()
    tl = args.tool
    if tl is None:
        try:
            st = requests.get(f"{args.url}/api/status", timeout=2).json()
            tl = 203.0 if int(st.get("tool_index", 0)) >= 1 else 0.0
        except Exception:
            tl = 0.0
    pairs = sample(args.url, args.seconds)
    analyse(pairs, tl)


if __name__ == "__main__":
    main()
