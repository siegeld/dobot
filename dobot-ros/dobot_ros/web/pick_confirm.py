"""Pick-time modal confirmation broker.

A pick can pause mid-execution and ask the connected web UI to confirm a
specific step before proceeding. This is the safety primitive that backs
the pre-pick "switch tool / orient / lock" prompt and any per-waypoint
confirms a strategy attaches.

Flow:
  Python (threadpool worker)         Browser (WebSocket)
  ──────────────────────────         ───────────────────
  broker.ask("...summary...")  ─►    pick_confirm message  ──► modal
       (blocks on Event)                                          │
  broker.reply(id, "OK") ◄──   POST /api/pick/confirm  ◄──────  click
  returns "OK"                       pick_confirm_resolved ─►  closes
                                     other browser modals

The broker owns no transport itself; the web server wires the push and
reply endpoints. Tests can use the broker with a fake `push_fn`.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

log = logging.getLogger(__name__)


class PickConfirmTimeout(Exception):
    """No reply arrived within the timeout."""


class PickConfirmCancelled(Exception):
    """User clicked Cancel (or any non-OK choice)."""


@dataclass
class PendingConfirm:
    id: str
    summary: str
    choices: Tuple[str, ...]
    created_at: float
    event: threading.Event = field(default_factory=threading.Event)
    result: Optional[str] = None


PushFn = Callable[[dict], None]
"""Synchronously schedules a broadcast of `payload` to all WS clients.
The implementation is responsible for crossing the sync→async boundary
(typically via asyncio.run_coroutine_threadsafe)."""


class PickConfirmBroker:
    """Thread-safe ask/reply broker for pick-time confirmations.

    `push_fn` is set by the web layer once the FastAPI event loop is
    available. Until then `ask()` returns "OK" without prompting (so unit
    tests and headless callers don't block waiting for a UI that doesn't
    exist).
    """

    def __init__(self):
        self._pending: Dict[str, PendingConfirm] = {}
        self._lock = threading.Lock()
        self._push_fn: Optional[PushFn] = None

    def set_push_fn(self, push_fn: Optional[PushFn]) -> None:
        with self._lock:
            self._push_fn = push_fn

    def ask(
        self,
        summary: str,
        *,
        choices: Tuple[str, ...] = ("OK", "Cancel"),
        timeout_s: float = 60.0,
    ) -> str:
        """Block the caller until the user answers, times out, or cancels.

        Returns the chosen string. Raises PickConfirmCancelled if any
        non-affirmative choice was picked, PickConfirmTimeout on timeout.

        If no UI is connected (no push_fn), auto-returns the first choice.
        Caller is the one running in a threadpool worker — never call from
        the event loop.
        """
        with self._lock:
            push_fn = self._push_fn

        if push_fn is None:
            log.warning("pick_confirm: no UI attached, auto-accepting %r", summary[:60])
            return choices[0]

        pending = PendingConfirm(
            id=uuid.uuid4().hex,
            summary=summary,
            choices=tuple(choices),
            created_at=time.time(),
        )
        with self._lock:
            self._pending[pending.id] = pending

        try:
            push_fn({
                "type": "pick_confirm",
                "id": pending.id,
                "summary": summary,
                "choices": list(choices),
            })
            log.info("pick_confirm asked id=%s summary=%r", pending.id, summary[:80])
            got = pending.event.wait(timeout=timeout_s)
            if not got:
                raise PickConfirmTimeout(
                    f"no reply within {timeout_s:.0f}s — pick aborted")
            choice = pending.result or "Cancel"
            if choice != choices[0]:
                raise PickConfirmCancelled(f"user picked {choice!r}")
            return choice
        finally:
            with self._lock:
                self._pending.pop(pending.id, None)
            try:
                push_fn({
                    "type": "pick_confirm_resolved",
                    "id": pending.id,
                    "choice": pending.result,
                })
            except Exception:
                pass

    def reply(self, request_id: str, choice: str) -> bool:
        """Resolve a pending confirm. Returns False if the id isn't pending."""
        with self._lock:
            pending = self._pending.get(request_id)
            if pending is None:
                return False
            if pending.event.is_set():
                return False  # already answered
            pending.result = choice
            pending.event.set()
        log.info("pick_confirm reply id=%s choice=%s", request_id, choice)
        return True

    def cancel_all(self, reason: str = "cancelled") -> int:
        """Resolve all pending confirms with `Cancel`. Used by E-STOP."""
        n = 0
        with self._lock:
            items = list(self._pending.values())
        for p in items:
            if not p.event.is_set():
                p.result = "Cancel"
                p.event.set()
                n += 1
        if n:
            log.warning("pick_confirm: cancelled %d pending (%s)", n, reason)
        return n

    def list_pending(self) -> List[dict]:
        """Snapshot of pending confirms (for replay on WS reconnect)."""
        with self._lock:
            return [
                {
                    "type": "pick_confirm",
                    "id": p.id,
                    "summary": p.summary,
                    "choices": list(p.choices),
                }
                for p in self._pending.values()
                if not p.event.is_set()
            ]
