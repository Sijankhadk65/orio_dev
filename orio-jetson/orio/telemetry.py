"""What Orio heard, said and did, for anything that wants to watch.

The conversation loop and the LLM report into this hub; the app server
(`server.py`) subscribes to it. Producers never know whether anyone is
listening, and a listener can never slow them down or take them out: every
report is a dict appended to a ring and handed to each listener, whose
exceptions are swallowed. That is the same contract `fsm.StateMachine` gives
its subscribers, for the same reason — the robot must not depend on the app.

Deliberately imports nothing but the standard library and `config`, so the
laptop mock (`tools/mock_server.py`) can use it without the robot's stack.

Messages are already in the wire shape of `docs/app-protocol.md`, so the server
forwards them as they are.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Any, Callable

from . import config

log = logging.getLogger(__name__)

Listener = Callable[[dict[str, Any]], None]


class Telemetry:
    """A ring of recent transcript lines, plus fan-out of every report."""

    def __init__(self, backlog: int = config.APP_TRANSCRIPT_BACKLOG) -> None:
        self._lines: deque[dict[str, Any]] = deque(maxlen=backlog)
        self._listeners: list[Listener] = []
        self._lock = threading.Lock()

    def subscribe(self, listener: Listener) -> Callable[[], None]:
        """Call `listener` with every report from now on. Returns an unsubscribe."""
        with self._lock:
            self._listeners.append(listener)

        def _unsubscribe() -> None:
            with self._lock:
                if listener in self._listeners:
                    self._listeners.remove(listener)

        return _unsubscribe

    def backlog(self) -> list[dict[str, Any]]:
        """The most recent transcript lines, oldest first."""
        with self._lock:
            return list(self._lines)

    def _publish(self, message: dict[str, Any], keep: bool) -> None:
        with self._lock:
            if keep:
                self._lines.append(message)
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(message)
            except Exception:  # a broken watcher must not break the robot
                log.exception("telemetry listener failed")

    # ── reports ──────────────────────────────────────────────────────────────

    def transcript(self, kind: str, text: str, **extra: Any) -> None:
        """One conversation line: kind is wake, heard, said or tool."""
        self._publish({"type": "transcript", "t": time.time(), "kind": kind, "text": text, **extra},
                      keep=True)

    def event(self, kind: str, text: str) -> None:
        """Something that happened: arrived, halted, blocked, lost, ..."""
        self._publish({"type": "event", "t": time.time(), "kind": kind, "text": text}, keep=False)


# One per process, like body.py's: the reporters are scattered through modules
# that share no context of their own.
hub = Telemetry()


def wake(text: str) -> None:
    hub.transcript("wake", text)


def heard(text: str) -> None:
    hub.transcript("heard", text)


def said(text: str) -> None:
    hub.transcript("said", text)


def tool(name: str, args: dict[str, Any], result: str) -> None:
    hub.transcript("tool", name, name=name, args=args, result=result)


def event(kind: str, text: str) -> None:
    hub.event(kind, text)
