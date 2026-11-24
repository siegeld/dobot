"""PolicyRunner — runs a policy on the simulator or the real arm, records
the trace, stores it, and runs seeded batches for success-rate debugging."""

from __future__ import annotations

import json
import logging
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.types import PolicyAbort, PolicyError, PolicyResult, Trace, TraceEvent
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.robot import SimClock, SimRobot
from dobot_ros.sim.world import SimWorld

log = logging.getLogger(__name__)

EventFn = Callable[[str, dict], None]          # (run_id, event dict)
RobotFactory = Callable[..., Tuple[Any, Any]]  # (trace, stop_event, confirm_mode) -> (robot, world)


@dataclass
class RunRequest:
    slug: str
    target: str = "sim"                 # "sim" | "robot"
    params: Dict[str, Any] = field(default_factory=dict)
    scene: Any = "two_blocks"           # scene name or dict (sim only)
    seed: Optional[int] = None
    random_positions: bool = False
    noise: Optional[dict] = None
    pace: float = 1.0                   # sim realtime factor (0 = as fast as possible)
    confirm_mode: str = "step"          # robot only: "step" | "once"
    rehearse: bool = True               # robot only: dry-run in sim against the real scene first
    batch_id: Optional[str] = None

    def to_dict(self) -> dict:
        d = asdict(self)
        if isinstance(d["scene"], dict):
            d["scene"] = d["scene"].get("name", "custom")
        return d


class RehearsalFailed(PolicyError):
    """The sim dry-run of this policy against the live scene hit a guard."""


_GUARD_ERRORS = ("FloorViolation", "WorkspaceViolation", "TiltViolation", "TravelViolation",
                 "Unreachable", "Collision")


class PolicyRunner:
    def _rehearse(self, policy, params: dict, robot, world, trace: Trace) -> dict:
        """Run the policy on the simulator from the robot's *current* joints
        against the objects perception reports right now, perfect perception.
        A guard violation in the rehearsal blocks the real run — the policy
        would have asked for something the engine refuses, so don't start.
        Grasp misses etc. do not block (the sim cannot know the real scene
        that well); they are reported."""
        import copy
        from dobot_ros.sim.world import SimWorld
        trace.log("rehearsal: dry-running the policy in the simulator against the live scene")
        info: Dict[str, Any] = {"blocked": False}
        try:
            objs = world.objects()
        except Exception as e:
            trace.log(f"rehearsal skipped: perception unavailable ({e})")
            info.update({"skipped": True, "reason": f"perception unavailable: {e}"})
            return info
        g = robot.guards
        sim_world = SimWorld(table_z=world.table_z, tool_length_mm=robot.tool_length_mm,
                             min_clearance_mm=world.min_clearance_mm, workspace=g.workspace,
                             noise=NoiseModel(enabled=False))
        for o in objs:
            sim_world.add(copy.deepcopy(o))
        clock = SimClock(pace=0.0)
        rtrace = Trace(clock=clock)
        gp = robot.gripper_pos()
        sim_robot = SimRobot(sim_world, trace=rtrace, clock=clock, start_joints=robot.joints(),
                             tool_index=robot.tool_index(), gripper_pos=gp if gp >= 0 else 1000,
                             guards=copy.deepcopy(g))
        try:
            res = policy.run(sim_robot, sim_world, params, rtrace)
            info.update({"ok": res.ok, "reason": res.reason, "attempts": res.attempts})
            trace.log(f"rehearsal result: ok={res.ok} {res.reason}")
        except PolicyError as e:
            name = type(e).__name__
            info.update({"ok": False, "reason": f"{name}: {e}", "blocked": name in _GUARD_ERRORS})
            trace.log(f"rehearsal raised {name}: {e}")
        except Exception as e:
            info.update({"ok": False, "reason": f"crash: {e}", "blocked": True})
        info["events"] = len(rtrace.events)
        info["moves"] = sim_robot.motion_count
        return info

    def __init__(self, registry: PolicyRegistry, traces_dir: Path,
                 robot_factory: Optional[RobotFactory] = None):
        self.registry = registry
        self.traces_dir = Path(traces_dir)
        self.traces_dir.mkdir(parents=True, exist_ok=True)
        self.robot_factory = robot_factory
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._current: Optional[dict] = None
        self._last: Optional[dict] = None

    # ── building backends ───────────────────────────────────────────
    def build_sim(self, req: RunRequest, trace: Trace, stop_event: threading.Event,
                  clock: SimClock) -> Tuple[SimRobot, SimWorld]:
        noise = NoiseModel.from_dict(req.noise) if req.noise is not None else None
        world = SimWorld.from_scene(req.scene, seed=req.seed, noise=noise,
                                    random_positions=req.random_positions)
        robot = SimRobot(world, trace=trace, clock=clock, stop_event=stop_event)
        return robot, world

    # ── single run ──────────────────────────────────────────────────
    def run(self, req: RunRequest, on_event: Optional[EventFn] = None,
            stop_event: Optional[threading.Event] = None, save: bool = True) -> dict:
        """Blocking. Returns the run record (also saved to traces_dir)."""
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:6]
        stop_event = stop_event or threading.Event()
        policy = self.registry.get_policy(req.slug)
        params = self.registry.validate_params(req.slug, req.params)
        wall0 = time.monotonic()

        if req.target == "sim":
            clock = SimClock(pace=req.pace)
            trace = Trace(clock=clock)
        else:
            clock = None
            trace = Trace()
        if on_event:
            trace.add_sink(lambda ev: on_event(run_id, ev.to_dict()))

        record: Dict[str, Any] = {
            "id": run_id, "slug": req.slug, "target": req.target, "params": params,
            "request": req.to_dict(), "started_utc": datetime.now(timezone.utc).isoformat(),
            "result": None, "events": [], "initial_objects": None, "final_objects": None,
            "batch_id": req.batch_id,
        }
        with self._lock:
            self._current = {"id": run_id, "slug": req.slug, "target": req.target,
                             "started": time.time(), "trace": trace}

        world = None
        try:
            if req.target == "sim":
                robot, world = self.build_sim(req, trace, stop_event, clock)
                record["initial_objects"] = world.snapshot()
                record["scene"] = world.to_scene()
                record["noise"] = world.noise.to_dict()
                record["seed"] = world.seed
            elif req.target == "robot":
                if self.robot_factory is None:
                    raise RuntimeError("real-robot runs are not available in this process")
                robot, world = self.robot_factory(trace=trace, stop_event=stop_event,
                                                  confirm_mode=req.confirm_mode)
                try:
                    record["initial_objects"] = [o.to_dict() for o in world.objects()]
                except Exception as e:
                    trace.log(f"initial perception failed: {e}")
            else:
                raise ValueError(f"unknown target {req.target!r}")

            if req.target == "robot" and req.rehearse:
                record["rehearsal"] = self._rehearse(policy, params, robot, world, trace)
                if record["rehearsal"].get("blocked"):
                    raise RehearsalFailed(record["rehearsal"]["reason"])

            trace.log(f"policy {req.slug} start on {req.target} params={params}")
            try:
                result = policy.run(robot, world, params, trace)
            except PolicyAbort as e:
                result = PolicyResult(ok=False, reason=f"aborted: {e}")
            except PolicyError as e:
                trace.error("policy", e)
                result = PolicyResult(ok=False, reason=f"{type(e).__name__}: {e}")
            finally:
                try:
                    robot.stop() if req.target == "robot" else None
                except Exception:
                    pass
            if not result.elapsed_s:
                result.elapsed_s = trace.now()
            record["result"] = result.to_dict()
            trace.emit(TraceEvent(t=trace.now(), kind="result", result=record["result"]))
        except RehearsalFailed as e:
            trace.log(f"REFUSED: {e}")
            record["result"] = PolicyResult(ok=False, reason=str(e)).to_dict()
            trace.emit(TraceEvent(t=trace.now(), kind="result", result=record["result"]))
        except Exception as e:  # infrastructure failure, not a policy outcome
            log.exception("policy run %s crashed", run_id)
            record["result"] = PolicyResult(ok=False, reason=f"error: {e}").to_dict()
            record["traceback"] = traceback.format_exc()
            trace.emit(TraceEvent(t=trace.now(), kind="result", result=record["result"],
                                  message=str(e)))
        finally:
            record["events"] = trace.to_list()
            record["elapsed_wall_s"] = round(time.monotonic() - wall0, 3)
            if world is not None:
                try:
                    record["final_objects"] = (world.snapshot() if hasattr(world, "snapshot")
                                               else [o.to_dict() for o in world.objects()])
                except Exception:
                    pass
            if save:
                self._save(record)
            with self._lock:
                self._current = None
                self._last = {k: v for k, v in record.items() if k != "events"}
        return record

    # ── background run ──────────────────────────────────────────────
    def start(self, req: RunRequest, on_event: Optional[EventFn] = None,
              on_done: Optional[Callable[[dict], None]] = None) -> str:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("a policy run is already active")
            self._stop = threading.Event()
            stop = self._stop
        box: Dict[str, Any] = {}
        ready = threading.Event()

        def _target():
            def _ev(run_id, ev):
                box.setdefault("id", run_id)
                ready.set()
                if on_event:
                    on_event(run_id, ev)
            rec = self.run(req, on_event=_ev, stop_event=stop)
            box.setdefault("id", rec["id"])
            ready.set()
            if on_done:
                try:
                    on_done(rec)
                except Exception:
                    log.exception("on_done failed")

        t = threading.Thread(target=_target, daemon=True, name="policy-run")
        with self._lock:
            self._thread = t
        t.start()
        ready.wait(timeout=5.0)
        return box.get("id", "pending")

    def start_batch(self, req: RunRequest, n: int, on_event: Optional[EventFn] = None,
                    on_done: Optional[Callable[[dict], None]] = None) -> str:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("a policy run is already active")
            self._stop = threading.Event()
            stop = self._stop
        batch_id = "batch-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")

        def _target():
            summary = self.batch(req, n, batch_id=batch_id, on_event=on_event, stop_event=stop)
            if on_done:
                on_done(summary)

        t = threading.Thread(target=_target, daemon=True, name="policy-batch")
        with self._lock:
            self._thread = t
        t.start()
        return batch_id

    def stop(self) -> bool:
        self._stop.set()
        cur = self._current
        return cur is not None

    def is_running(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def status(self) -> dict:
        with self._lock:
            cur = self._current
            d = {"running": self.is_running(), "current": None, "last": self._last}
            if cur:
                d["current"] = {"id": cur["id"], "slug": cur["slug"], "target": cur["target"],
                                "elapsed_s": round(time.time() - cur["started"], 1),
                                "events": len(cur["trace"].events)}
            return d

    # ── batch ───────────────────────────────────────────────────────
    def batch(self, req: RunRequest, n: int, batch_id: Optional[str] = None,
              on_event: Optional[EventFn] = None,
              stop_event: Optional[threading.Event] = None) -> dict:
        """Run ``n`` sim episodes with seeds seed0..seed0+n-1, as fast as
        possible, randomizing positions. Returns a summary dict."""
        if req.target != "sim":
            raise ValueError("batch runs are sim-only")
        batch_id = batch_id or "batch-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        seed0 = req.seed if req.seed is not None else 0
        runs = []
        reasons: Dict[str, int] = {}
        stop_event = stop_event or threading.Event()
        t0 = time.monotonic()
        for i in range(int(n)):
            if stop_event.is_set():
                break
            r = RunRequest(**{**asdict(req), "seed": seed0 + i, "pace": 0.0,
                              "random_positions": True, "batch_id": batch_id})
            rec = self.run(r, stop_event=stop_event, save=True)
            res = rec["result"] or {}
            if not res.get("ok"):
                key = (res.get("reason") or "unknown").split(";")[0][:80]
                reasons[key] = reasons.get(key, 0) + 1
            runs.append({"seed": r.seed, "ok": bool(res.get("ok")), "reason": res.get("reason", ""),
                         "attempts": res.get("attempts"), "elapsed_s": res.get("elapsed_s"),
                         "trace_id": rec["id"]})
            if on_event:
                on_event(batch_id, {"kind": "batch_progress", "done": i + 1, "total": int(n),
                                    "ok": sum(1 for x in runs if x["ok"])})
        ok = sum(1 for x in runs if x["ok"])
        summary = {
            "batch_id": batch_id, "slug": req.slug, "n": len(runs), "ok": ok,
            "success_rate": round(ok / len(runs), 3) if runs else 0.0,
            "mean_attempts": round(sum((x["attempts"] or 0) for x in runs) / len(runs), 2) if runs else 0.0,
            "reasons": reasons, "runs": runs, "params": self.registry.validate_params(req.slug, req.params),
            "scene": req.scene if isinstance(req.scene, str) else "custom",
            "wall_s": round(time.monotonic() - t0, 2),
        }
        (self.traces_dir / f"{batch_id}.json").write_text(json.dumps(summary, indent=1))
        with self._lock:
            self._last = {"batch": summary}
        return summary

    # ── trace store ─────────────────────────────────────────────────
    def _save(self, record: dict) -> Path:
        p = self.traces_dir / f"{record['id']}.json"
        p.write_text(json.dumps(record, indent=None))
        return p

    def list_traces(self, limit: int = 50) -> List[dict]:
        out = []
        for p in sorted(self.traces_dir.glob("*.json"), reverse=True)[: limit * 2]:
            try:
                d = json.loads(p.read_text())
            except Exception:
                continue
            if "batch_id" in d and "events" in d:
                res = d.get("result") or {}
                out.append({"id": d["id"], "slug": d["slug"], "target": d["target"],
                            "started_utc": d.get("started_utc"), "ok": res.get("ok"),
                            "reason": res.get("reason", ""), "attempts": res.get("attempts"),
                            "events": len(d.get("events", [])), "batch_id": d.get("batch_id"),
                            "seed": d.get("seed")})
            elif "runs" in d:
                out.append({"id": d["batch_id"], "slug": d["slug"], "target": "sim-batch",
                            "ok": d["success_rate"] >= 0.95, "reason": f"{d['ok']}/{d['n']} ok",
                            "events": 0, "batch": True})
            if len(out) >= limit:
                break
        return out

    def get_trace(self, trace_id: str) -> Optional[dict]:
        p = self.traces_dir / f"{Path(trace_id).name}.json"
        if not p.exists():
            return None
        return json.loads(p.read_text())

    def delete_trace(self, trace_id: str) -> bool:
        p = self.traces_dir / f"{Path(trace_id).name}.json"
        if p.exists():
            p.unlink()
            return True
        return False
