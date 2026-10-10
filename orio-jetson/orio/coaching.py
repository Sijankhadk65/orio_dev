"""A live yoga session: the camera, the pose model, the coach, and its voice.

yoga.py decides what to say about one frame; this runs it. A session is a
background thread that reads the Gemini, finds the person, hands them to a
`yoga.Coach`, and speaks what comes back through `cues.CoachVoice` — until the
pose is held long enough, or something stops it.

## Alongside the conversation, not instead of it

A session is not a state of `fsm.StateMachine`. The machine tracks the
conversation, and the conversation carries on during yoga: the person can wake
Orio mid-pose to ask how long is left, or to stop. So the session runs next to
it and takes one rule from it — **the coach is silent unless Orio is asleep or
idle.** Listening, thinking and speaking all mean someone is talking to Orio
or Orio is answering, and a correction in the middle of either is noise at
best and, played into an open mic, a command at worst. Cues that come up while
muted are dropped, not queued; the coach repeats what still matters.

## Parked

The robot does not move during a session. `start()` stops the wheels and
re-centres the head (the driving tilt is already the neck's highest), and the
tools that move the wheels or the head refuse while a session is running (see
tools.py) — someone on a mat with their eyes closed is not something to drive
at. The camera is shared (`gemini.shared`), so obstacle sensing keeps running
and nothing has to be handed back afterwards.

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

from . import config
from .fsm import State
from .yoga import POSES, Coach, phrases

log = logging.getLogger(__name__)

# Conversation states in which the coach may speak. Everything else is a person
# talking to Orio, or Orio answering them.
_QUIET_OK = {State.ASLEEP, State.IDLE}

# What a person might call each pose, mapped to yoga.POSES keys.
POSE_ALIASES = {
    "mountain": "mountain", "mountain pose": "mountain", "tadasana": "mountain",
    "warrior": "warrior_ii", "warrior 2": "warrior_ii", "warrior two": "warrior_ii",
    "warrior ii": "warrior_ii", "warrior_ii": "warrior_ii", "virabhadrasana": "warrior_ii",
    "virabhadrasana ii": "warrior_ii",
    "tree": "tree", "tree pose": "tree", "vrksasana": "tree",
}

# Said when a session is stopped by the person rather than finished.
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


class Session:
    """One pose, coached until held, stopped, or broken. Runs its own thread."""

    def __init__(self, pose_key: str, estimator, voice, fsm=None, camera=None) -> None:
        self.pose = POSES[pose_key]
        self.coach = Coach(self.pose)
        self._estimator = estimator
        self._voice = voice
        self._fsm = fsm
        self._camera = camera
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.ended_by: str | None = None  # "done", "stopped", "error"

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

    def _may_speak(self) -> bool:
        return self._fsm is None or self._fsm.state in _QUIET_OK

    def _say(self, text: str) -> None:
        if self._voice is not None and self._may_speak():
            self._voice.say(text)

    def status(self) -> str:
        c = self.coach
        if not self.running:
            return f"the {self.pose.name} session has ended ({self.ended_by or 'not started'})"
        if c.state == "waiting":
            return f"coaching {self.pose.name}: waiting for them to get into the pose"
        if c.state == "adjusting":
            last = c.last_cue.text if c.last_cue else ""
            return (f"coaching {self.pose.name}: in the pose, being corrected "
                    f"(last said: {last!r}); held {c.held_s:.0f} of {self.pose.hold_s:.0f} s")
        return f"coaching {self.pose.name}: holding it well, {c.held_s:.0f} of {self.pose.hold_s:.0f} s"

    def _run(self) -> None:
        from .pose import lift

        try:
            if self._voice is not None:
                intro = f"Let's do {self.pose.name}. {self.pose.how_to}"
                # First run for a voice synthesizes every line (seconds); later
                # runs load them from disk. Either way before the camera loop,
                # so no cue waits on the network mid-pose.
                self._voice.prewarm([intro, *phrases(self.pose)])
                self._wait_until_quiet()
                self._say(intro)
            camera = self._camera
            if camera is None:
                from .gemini import shared

                camera = shared()
            seq = 0
            while not self._stop.is_set():
                frames = camera.read(after=seq)
                seq = frames.seq
                people = self._estimator.estimate(frames.color)
                person = people[0] if people else None
                if person is not None:
                    lift(person, frames.depth, camera.intrinsics)
                h, w = frames.color.shape[:2]
                cue = self.coach.update(person, w, h)
                if cue is not None:
                    log.info("coach [%s] %s", cue.kind, cue.text)
                    self._say(cue.text)
                if self.coach.state == "done":
                    self.ended_by = "done"
                    return
            self.ended_by = "stopped"
        except Exception as exc:  # noqa: BLE001 - reported through status, never raised
            log.exception("yoga session failed")
            self.error = str(exc)
            self.ended_by = "error"

    def _wait_until_quiet(self, limit_s: float = 30.0) -> None:
        """Hold the intro until Orio has finished its own reply."""
        deadline = time.monotonic() + limit_s
        while not self._may_speak() and not self._stop.is_set() and time.monotonic() < deadline:
            time.sleep(0.1)


# ── the one session ───────────────────────────────────────────────────────────
# There is one camera and one person on the mat, so at most one session. The
# conversation loop wires in the TTS and the state machine once (configure());
# the LLM tools then start and stop sessions by name.

_lock = threading.Lock()
_session: Session | None = None
_tts = None
_fsm = None
_voice = None
_estimator = None


def configure(tts, fsm) -> None:
    """Give sessions a voice and the conversation's state. Call once at startup."""
    global _tts, _fsm
    _tts, _fsm = tts, fsm


def active() -> bool:
    with _lock:
        return _session is not None and _session.running


def current() -> Session | None:
    with _lock:
        return _session


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


def start(pose_key: str) -> Session:
    """Start coaching `pose_key`, replacing any session already running."""
    global _session, _voice, _estimator
    with _lock:
        old = _session
    if old is not None:
        old.stop()
    _park()
    with _lock:
        if _estimator is None:
            from .pose import PoseEstimator

            _estimator = PoseEstimator()
        if _voice is None and _tts is not None:
            from .cues import CoachVoice

            _voice = CoachVoice(_tts)
        _session = Session(pose_key, _estimator, _voice, _fsm)
        _session.start()
        return _session


def stop() -> Session | None:
    """End the running session, if any; returns it (or None)."""
    with _lock:
        s = _session
    if s is not None and s.running:
        s.stop()
        if _voice is not None:
            _voice.cancel()  # a correction queued just before "stop" stays unsaid
        return s
    return None


def shutdown() -> None:
    global _voice
    stop()
    with _lock:
        if _voice is not None:
            _voice.close()
            _voice = None
