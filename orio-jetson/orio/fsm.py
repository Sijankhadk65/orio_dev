"""Robot operator state machine.

A single source of truth for *what Orio is doing right now* — idle, listening,
thinking, or speaking. The conversation loop drives the transitions; other
subsystems (the eyes display, status LEDs, motion safety) subscribe to state
changes instead of reaching into the loop. This decouples "what state are we in"
from "what each subsystem does about it", so adding the eyes later means writing
a subscriber, not threading calls through `conversation.py`.

Thread-safety: the conversation loop runs on the main thread while subscribers
(e.g. a future eyes renderer) may react from their own threads, so the current
state and the listener list are guarded by a lock. Listener callbacks are
invoked *outside* the lock, on whichever thread triggered the transition, so a
slow or reentrant subscriber can't deadlock the machine — but callbacks should
still be cheap and non-blocking (set a flag / post an event, don't compute).
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from enum import Enum, auto

log = logging.getLogger(__name__)


class State(Enum):
    """The mutually-exclusive things Orio can be doing."""

    ASLEEP = auto()     # dormant; only listening for the wake word ("Hey Orio")
    IDLE = auto()       # awake and waiting; nothing in flight
    LISTENING = auto()  # mic open, capturing a command phrase
    THINKING = auto()   # phrase captured, LLM generating a reply
    SPEAKING = auto()   # reply is being spoken aloud
    ERROR = auto()      # a subsystem failed; recoverable back to IDLE

    def __str__(self) -> str:  # readable logs / subscriber output
        return self.name


# Legal transitions. Keeping this explicit turns loop bugs into loud errors
# (e.g. jumping straight from LISTENING to SPEAKING without THINKING) instead of
# silently allowing any move. ERROR is reachable from every state; from ERROR we
# only recover to a resting state (IDLE or ASLEEP).
#
# The wake-gated flow is ASLEEP →(wake word)→ LISTENING → THINKING → SPEAKING,
# then either LISTENING again (follow-up window) or back to ASLEEP when it
# lapses. IDLE is the resting state when wake-gating is off (always-on) and in
# text mode.
_TRANSITIONS: dict[State, set[State]] = {
    State.ASLEEP: {State.LISTENING, State.IDLE, State.ERROR},
    State.IDLE: {State.ASLEEP, State.LISTENING, State.THINKING, State.ERROR},
    State.LISTENING: {State.THINKING, State.ASLEEP, State.IDLE, State.ERROR},
    State.THINKING: {State.SPEAKING, State.IDLE, State.ERROR},
    State.SPEAKING: {State.LISTENING, State.ASLEEP, State.IDLE, State.ERROR},
    State.ERROR: {State.IDLE, State.ASLEEP},
}

# Subscriber callback: receives (old_state, new_state) on every real transition.
Listener = Callable[[State, State], None]


class InvalidTransition(RuntimeError):
    """Raised when a transition isn't permitted from the current state."""


class StateMachine:
    """Thread-safe FSM with an observer hook."""

    def __init__(self, initial: State = State.IDLE) -> None:
        self._state = initial
        self._lock = threading.RLock()
        self._listeners: list[Listener] = []

    @property
    def state(self) -> State:
        with self._lock:
            return self._state

    def subscribe(self, listener: Listener) -> Callable[[], None]:
        """Register `listener`, fired on every transition as (old, new).

        Returns an unsubscribe callable. Callbacks run synchronously on the
        transitioning thread, so keep them quick.
        """
        with self._lock:
            self._listeners.append(listener)

        def _unsubscribe() -> None:
            with self._lock:
                if listener in self._listeners:
                    self._listeners.remove(listener)

        return _unsubscribe

    def to(self, target: State, *, force: bool = False) -> State:
        """Transition to `target`; return the resulting state.

        A no-op (target == current) is allowed and quietly ignored, so callers
        needn't track whether they're already in a state. Otherwise the move
        must be in the transition table unless `force=True` (reserved for
        recovery paths that must always succeed).

        Raises:
            InvalidTransition: if the move isn't allowed and `force` is False.
        """
        with self._lock:
            current = self._state
            if target is current:
                return current
            if not force and target not in _TRANSITIONS.get(current, set()):
                raise InvalidTransition(f"{current} → {target} not allowed")
            self._state = target
            listeners = list(self._listeners)  # snapshot for lock-free notify

        log.debug("state %s → %s", current, target)
        for fn in listeners:
            try:
                fn(current, target)
            except Exception:  # a buggy subscriber must not break the loop
                log.exception("state listener %r failed", fn)
        return target
