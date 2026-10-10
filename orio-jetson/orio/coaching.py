"""A live yoga session: the camera and pose model watch, the LLM coaches.

yoga.py decides WHAT is wrong and WHEN it is worth a word. This runs it, and
hands every such moment to the LLM to decide what to SAY — so Orio coaches the
way a person would, in its own words, aware of what it already said and of
anything the person asked, rather than playing one fixed sentence on repeat.

## Two threads

The **session** thread is perception. It reads the Gemini, finds the person
(pose.py), and runs `yoga.Framer` until they are properly in shot, then
`yoga.Coach` on the pose itself. Both return a `yoga.Cue` when something is
worth saying: the facts, plus a correct teaching cue as a hint.

The **narrator** thread is speech. It turns a Cue into an observation message
("[yoga camera] … their front knee is at 128 degrees, above the 80-110 range
…") and runs it as an ordinary turn of the SAME `llm.Conversation` the person
talks to, through the conversation loop's own `_handle_turn` — so it streams,
speaks, and can use tools (`center_on_person` while framing, `stop_yoga`) like
any other turn, and the history carries both sides of the session. Ask "how am
I doing?" and the model knows, because it has been saying so.

Cues queue newest-first: one waiting slot, replaced by anything newer, and a
waiting cue that goes stale is dropped — by the time it is said the person has
moved. The few that must not be lost (framed, praise, release) are kept over
ordinary ones.

## Taking turns with the person

There is one conversation, so there is one speaker at a time. `turn_lock` is
held by the conversation loop for as long as Orio is awake with the person (see
conversation._run_voice) and by the narrator for each coaching turn, and the
narrator only starts one while Orio is asleep or idle. A "Hey Orio" during a
coaching sentence waits for the sentence to end, then has Orio's full
attention; the coach picks up again once Orio goes back to sleep.

## Framing first, then parked

A session starts in the framing phase: no pose is taught until the whole person
is in view, near the middle, at a workable distance and facing the camera. In
that phase — and only then — the model may pivot Orio toward the person with
`center_on_person`; nothing drives, and once the pose starts Orio is parked
(the wheel and head tools refuse while a session runs; see tools.py).

## Stopping

`stop()` ends it from anywhere: the LLM's stop_yoga tool, or the conversation
loop's own check for "stop" and for pain words, which runs before the LLM so
that "ow, that hurts" ends the session even if the model would have dithered.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass

from . import config
from .fsm import State
from .yoga import POSES, Coach, Cue, Framer, bearing_deg

log = logging.getLogger(__name__)

# Conversation states in which the coach may take a turn. Everything else is a
# person talking to Orio, or Orio answering them.
_QUIET_OK = {State.ASLEEP, State.IDLE}

# What a person might call each pose, mapped to yoga.POSES keys.
POSE_ALIASES = {
    "mountain": "mountain", "mountain pose": "mountain", "tadasana": "mountain",
    "warrior": "warrior_ii", "warrior 2": "warrior_ii", "warrior two": "warrior_ii",
    "warrior ii": "warrior_ii", "warrior_ii": "warrior_ii", "virabhadrasana": "warrior_ii",
    "virabhadrasana ii": "warrior_ii",
    "tree": "tree", "tree pose": "tree", "vrksasana": "tree",
}

# Fallbacks for the conversation loop's stop/pain check, used only if the LLM
# cannot be reached to say it in its own words.
STOPPED_LINE = "Okay, we'll stop there. Come out of the pose slowly."
HURT_LINE = ("Okay, come out of the pose gently, right now. Stopping is always fine. "
             "If the pain doesn't settle, please check with a doctor.")

# Checked by the conversation loop BEFORE the LLM, while a session runs.
# Pain is any mention of it; stopping is a short utterance that is about
# stopping, so "stop telling me about my knee" still reaches the model.
_PAIN = re.compile(r"\b(hurts?|hurting|pain|painful|ouch|ow+|injur\w*|cramp\w*)\b")
_STOP = re.compile(
    r"^(please )?(stop|stop (it|now|yoga|the yoga|the session|coaching)|"
    r"i'?m done|i am done|that'?s enough|enough|finish|end( the)? (session|yoga)|"
    r"no more( yoga)?|let'?s stop)( please)?$"
)


def resolve_pose(name: str) -> str | None:
    """A yoga.POSES key for what the model or person called the pose, or None."""
    key = re.sub(r"\s+", " ", str(name).strip().lower().replace("-", " "))
    return POSE_ALIASES.get(key) or (key if key in POSES else None)


def classify(text: str) -> str | None:
    """"hurt", "stop", or None — what an utterance means for a running session."""
    t = re.sub(r"[^\w' ]", "", text.lower()).strip()
    if _PAIN.search(t):
        return "hurt"
    if _STOP.match(t):
        return "stop"
    return None


# ── what the debug window draws ───────────────────────────────────────────────


@dataclass
class View:
    """One frame of a session, as the debug window shows it."""

    frame: object
    people: list
    subject: object
    angles: list
    infer_ms: float
    phase: str
    pose_name: str
    framer: Framer
    coach: Coach
    said: str  # the last thing handed to the narrator, as facts
    t: float


# ── the session ───────────────────────────────────────────────────────────────


class Session:
    """One pose: framed, then coached until held, stopped, or broken."""

    def __init__(self, pose_key: str, estimator, emit=None, camera=None,
                 estimate_lock: threading.Lock | None = None) -> None:
        self.pose = POSES[pose_key]
        self.framer = Framer(view=self.pose.view)
        self.coach = Coach(self.pose)
        self.phase = "framing"
        self._estimator = estimator
        self._lock = estimate_lock or threading.Lock()
        self._emit = emit  # callable(Cue, Session); None = nobody listening
        self._camera = camera
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.ended_by: str | None = None  # "done", "stopped", "error"
        self.view: View | None = None
        self.said = ""

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="yoga", daemon=True)
        self._thread.start()

    def stop(self, wait: bool = True) -> None:
        self._stop.set()
        if wait and self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _camera_or_shared(self):
        if self._camera is None:
            from .gemini import shared

            self._camera = shared()
        return self._camera

    def status(self) -> str:
        c = self.coach
        if not self.running:
            return f"the {self.pose.name} session has ended ({self.ended_by or 'not started'})"
        if self.phase == "framing":
            issues = "; ".join(w for _, w in self.framer.issues) or "nearly in position"
            return f"{self.pose.name}: still getting them into view — {issues}"
        if c.state == "waiting":
            return f"{self.pose.name}: they are in view; waiting for them to get into the pose"
        if c.state == "adjusting":
            return (f"{self.pose.name}: in the pose, working on {c.failing}; "
                    f"held {c.held_s:.0f} of {self.pose.hold_s:.0f} s")
        return f"{self.pose.name}: holding it well, {c.held_s:.0f} of {self.pose.hold_s:.0f} s"

    def _run(self) -> None:
        from .pose import joint_angles, lift

        try:
            camera = self._camera_or_shared()
            seq = 0
            while not self._stop.is_set():
                frames = camera.read(after=seq)
                seq = frames.seq
                t0 = time.monotonic()
                with self._lock:
                    people = self._estimator.estimate(frames.color)
                infer_ms = (time.monotonic() - t0) * 1000
                person = people[0] if people else None
                if person is not None:
                    lift(person, frames.depth, camera.intrinsics)
                h, w = frames.color.shape[:2]

                cue = self.framer.update(person, w, h)
                if self.phase == "framing" and self.framer.framed:
                    self.phase = "pose"
                elif self.phase == "pose" and not self.framer.framed:
                    self.phase = "framing"  # lost them: frame again, hold kept
                if self.phase == "pose" and cue is None:
                    cue = self.coach.update(person, w, h)
                if cue is not None:
                    log.info("yoga [%s] %s", cue.kind, cue.detail or cue.text)
                    self.said = f"{cue.kind}: {cue.detail or cue.text}"
                    if self._emit is not None:
                        self._emit(cue, self)

                self.view = View(frames.color, people, person,
                                 joint_angles(person) if person is not None else [],
                                 infer_ms, self.phase, self.pose.name, self.framer,
                                 self.coach, self.said, time.monotonic())
                if self.coach.state == "done":
                    self.phase = "done"
                    self.ended_by = "done"
                    return
            self.ended_by = "stopped"
        except Exception as exc:  # noqa: BLE001 - reported through status, never raised
            log.exception("yoga session failed")
            self.error = str(exc)
            self.ended_by = "error"

    # ── framing help ─────────────────────────────────────────────────────────

    def center(self, tries: int = 3) -> str:
        """Pivot Orio until the person sits near the middle of the picture."""
        if self.phase != "framing":
            return "they are already in position — Orio stays still during the pose"
        from .body import get as get_body
        from .seek import pivot

        body = get_body()
        if body is None or not body.can_drive:
            return "Orio can't turn right now — ask them to step toward the middle instead"
        for _ in range(tries):
            seen = self.view
            if seen is None or seen.subject is None:
                return "can't see them, so there is nothing to turn toward"
            x1, _, x2, _ = seen.subject.bbox
            off = bearing_deg((x1 + x2) / 2.0, seen.frame.shape[1])
            if abs(off) <= config.YOGA_CENTRE_DEG:
                return f"turned to face them; they are {abs(off):.0f} degrees off centre now"
            pivot(body, "left" if off < 0 else "right", min(abs(off), config.SEEK_TURN_DEG))
            time.sleep(config.SEEK_SETTLE_S)  # let a fresh frame through the model
        return "turned toward them, but they are still not quite in the middle"


# ── the narrator ──────────────────────────────────────────────────────────────

# Cues that must survive a newer, ordinary one arriving while they wait.
_KEEP = {"framed", "praise", "release"}
# A waiting ordinary cue older than this (s) is dropped, not said late.
_STALE_S = 4.0

_WHAT = {
    "framing": "They are not properly in view yet.",
    "framed": "They are now fully in view, so the pose can begin.",
    "square": "In the pose, but their body has turned away from you.",
    "prompt": "They are in view but have not got into the pose yet.",
    "correction": "They are in the pose, and one thing needs adjusting.",
    "praise": "Their pose is now right, and the hold has started.",
    "progress": "They are halfway through the hold.",
    "release": "They have finished the hold.",
    "left": "They came out of the pose early.",
}


def observation(cue: Cue, session: Session) -> str:
    """The message a Cue becomes: facts for the model, not words for the person."""
    pose = session.pose
    lead = session.coach.lead if session.phase != "framing" else None
    side = ""
    if pose.roles and lead:
        side = f" ({pose.roles['lead']} side: their {lead})"
    lines = [
        "[Yoga camera update. This is what your camera and pose measurements show — "
        "it is NOT something the person said. Reply with only what you say to them "
        "out loud.]",
        f"Pose: {pose.name}{side}. {_WHAT.get(cue.kind, '')}",
    ]
    if cue.detail:
        lines.append(f"Measured: {cue.detail}.")
    if cue.kind == "framing" and "off-centre" in (code for code, _ in session.framer.issues):
        lines.append("You can call center_on_person to turn toward them yourself, "
                     "instead of asking them to move.")
    if cue.kind == "framed":
        lines.append(f"Now guide them into {pose.name}. How it is done: {pose.how_to}")
    elif cue.kind == "release":
        lines.append("Tell them to release, and congratulate them. You can offer another "
                     "pose (mountain, warrior two, tree) or ask how that felt.")
    else:
        lines.append(f'A correct teaching cue for this: "{cue.text}"')
    if cue.repeat > 1:
        lines.append(f"You have already raised this {cue.repeat - 1} time(s) in a row — "
                     "say it a different way, or break it into one smaller step.")
    lines.append("One or two short, warm sentences. Only mention a number if it helps.")
    return "\n".join(lines)


class _Narrator:
    """Speaks cues through the LLM, one at a time, only when Orio is free."""

    def __init__(self, fsm, run_turn) -> None:
        self._fsm = fsm
        self._run_turn = run_turn  # callable(text): one full LLM turn, spoken
        self._pending: tuple[Cue, Session, float] | None = None
        self._cond = threading.Condition()
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="yoga-narrator", daemon=True)
        self._thread.start()

    def post(self, cue: Cue, session: Session) -> None:
        with self._cond:
            old = self._pending
            if old is not None and old[0].kind in _KEEP and cue.kind not in _KEEP:
                return  # "framed" waiting beats a later ordinary correction
            self._pending = (cue, session, time.monotonic())
            self._cond.notify()

    def cancel(self) -> None:
        with self._cond:
            self._pending = None

    def _take(self):
        with self._cond:
            while self._pending is None and not self._closed:
                self._cond.wait(0.5)
            if self._closed:
                return None
            item, self._pending = self._pending, None
            return item

    def _run(self) -> None:
        while True:
            item = self._take()
            if item is None:
                return
            cue, session, at = item
            # Wait for Orio to be free — the person has the floor until then.
            while not self._closed:
                if not turn_lock.acquire(timeout=0.2):
                    continue
                if self._fsm.state in _QUIET_OK:
                    break
                turn_lock.release()
                time.sleep(0.2)
            else:
                return
            try:
                stale = cue.kind not in _KEEP and time.monotonic() - at > _STALE_S
                gone = not session.running and cue.kind != "release"
                if stale or gone:
                    continue
                rest = self._fsm.state
                try:
                    self._run_turn(observation(cue, session))
                finally:
                    if self._fsm.state is not rest:
                        self._fsm.to(rest, force=True)
            except Exception:  # noqa: BLE001 - a failed turn must not end coaching
                log.exception("yoga narrator turn failed")
            finally:
                turn_lock.release()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify()
        self._thread.join(timeout=1.0)


# ── the one session ───────────────────────────────────────────────────────────
# There is one camera and one person on the mat, so at most one session. The
# conversation loop wires in the state machine and its turn runner once
# (configure()); the LLM tools then start and stop sessions by name.

# One speaker at a time — see "Taking turns with the person" above.
turn_lock = threading.RLock()

_lock = threading.Lock()
_estimate_lock = threading.Lock()  # the pose model is not thread-safe
_session: Session | None = None
_narrator: _Narrator | None = None
_estimator = None


def configure(fsm, run_turn) -> None:
    """Give sessions the conversation: its state, and a way to take a turn in it.

    `run_turn(text)` must run one complete LLM turn on `text` and speak the
    reply, leaving the state machine in SPEAKING (conversation._handle_turn).
    """
    global _narrator
    if _narrator is not None:
        _narrator.close()
    _narrator = _Narrator(fsm, run_turn)


def active() -> bool:
    with _lock:
        return _session is not None and _session.running


def current() -> Session | None:
    with _lock:
        return _session


def estimator():
    """The shared pose model, and the lock to hold while using it."""
    global _estimator
    with _lock:
        if _estimator is None:
            from .pose import PoseEstimator

            _estimator = PoseEstimator()
        return _estimator, _estimate_lock


def _park() -> None:
    """Wheels stopped, head centred: the robot holds still for the session."""
    from .body import get as get_body

    body = get_body()
    if body is None:
        return
    if body.can_drive:
        body.stop()
    if body.can_look and not body.head_is_driving_pose:
        body.restore_head()


def _emit(cue: Cue, session: Session) -> None:
    if _narrator is not None:
        _narrator.post(cue, session)


def start(pose_key: str) -> Session:
    """Start a session for `pose_key`, replacing any session already running."""
    global _session
    with _lock:
        old = _session
    if old is not None:
        old.stop()
    if _narrator is not None:
        _narrator.cancel()  # nothing about the old pose is worth saying now
    _park()
    est, est_lock = estimator()
    with _lock:
        _session = Session(pose_key, est, _emit, estimate_lock=est_lock)
        _session.start()
        return _session


def stop() -> Session | None:
    """End the running session, if any; returns it (or None)."""
    with _lock:
        s = _session
    if s is not None and s.running:
        s.stop()
        if _narrator is not None:
            _narrator.cancel()  # a correction queued just before "stop" stays unsaid
        return s
    return None


def shutdown() -> None:
    global _narrator
    stop()
    if _narrator is not None:
        _narrator.close()
        _narrator = None
