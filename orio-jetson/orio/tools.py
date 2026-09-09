"""LangChain-compatible tools the LLM can call.

Kept separate from llm.py so the tool set can grow without touching the
conversation wrapper. Each tool is a plain `@tool`-decorated function.

Two kinds live here now. The query tools answer questions about the world and
change nothing. The drive tools *move the robot*, and they are a different
proposition: they go through `body.py`, which owns the boards, runs every move
under the obstacle-avoidance policy in `avoid.py`, and stops the wheels itself
when the hop ends. Nothing here can reach the wheels any other way — there is no
unguarded path and no flag that makes one. A confused model can waste a few
seconds; it cannot aim the robot at a wall and be obeyed.

Missing deps or hardware degrade to a smaller tool list rather than breaking
the conversation loop — no camera means no vision tool, a drivetrain that did
not open means no drive tools, and the system prompt is chosen to match (see
llm.py) so Orio never offers to do something it currently cannot.
"""

from __future__ import annotations

import logging
import time

from langchain_core.tools import tool

from . import config
from .body import NO_WHEELS, get as get_body
from .knowledge import search as search_knowledge

log = logging.getLogger(__name__)

_detector = None  # lazily-built ObjectDetector, shared across calls


def get_detector():
    """The shared ObjectDetector (built on first use). Also used by
    conversation.py to hand the same instance to VisionDebugWindow, so the
    debug preview and the LLM's tool calls never open a second camera handle.
    """
    global _detector
    if _detector is None:
        from .vision import ObjectDetector

        _detector = ObjectDetector()
    return _detector


def _band(tilt: float) -> str:
    """Where a tilt is looking, relative to the pose Orio drives on."""
    if tilt < config.NECK_TILT_DEG - 6:
        return "above"
    if tilt > config.NECK_TILT_DEG + 6:
        return "below"
    return ""


def _sweep(body, pan: float, tilts) -> dict:
    """Tilt through `tilts` at `pan`, detecting at each stop; merge what's found.

    One tilt is one horizontal slice of the room — the cameras' vertical field
    is narrow, so a single frame misses anything above or below it. Merged by
    label, keeping the most confident sighting of each, with the band it was
    seen in so "a mug up on the shelf" doesn't read the same as one underfoot.

    Restores the driving tilt on the way out but leaves `pan` alone: a look to
    the left should stay looking left until something drives.
    """
    detector = get_detector()
    best: dict[str, tuple] = {}
    for tilt in tilts:
        if body.look(pan, tilt) is not None:
            continue
        time.sleep(config.SEEK_SETTLE_S)
        _reading, frame = body.snapshot()
        if frame is None:
            continue
        for d in detector.detect_in(frame):
            found = best.get(d.label)
            if found is None or d.confidence > found[0].confidence:
                best[d.label] = (d, _band(tilt))
    body.look(pan, config.NECK_TILT_DEG)
    return best


def _report(best: dict) -> str:
    if not best:
        return "no recognizable objects in view right now"
    parts = []
    for d, band in best.values():
        where = f"{d.position}, {band}" if band else d.position
        parts.append(f"{d.label} ({where}, {d.confidence:.0%} confidence)")
    return "; ".join(parts)


@tool
def what_do_you_see() -> str:
    """Look around in front of Orio and report what objects are there.

    Use this whenever asked what you can see, what's in front of you, or to
    describe your surroundings. Takes no arguments. Orio tilts its head up and
    down while looking, so this takes a few seconds and catches things above and
    below its normal eyeline, not just straight ahead.
    """
    body = get_body()
    # Whenever Orio can drive, the avoidance thread owns both CSI sensors and
    # Argus will not give the detector a handle of its own — so the picture comes
    # from the stereo pair. detect_once() is the fallback for a no-driving run,
    # where nothing holds the cameras and the head may not even be connected.
    try:
        if body is not None and body.can_look:
            return _report(_sweep(body, body.head_pan, config.LOOK_TILT_SWEEP_DEG))
        detector = get_detector()
        detections = detector.detect_once()
    except Exception as exc:
        return f"camera error: {exc}"
    return _report({d.label: (d, "") for d in detections})


@tool
def search_knowledge_base(question: str) -> str:
    """Search Orio's knowledge base for facts relevant to the question.

    Covers what Orio itself is, what it can do, and who built it, plus
    whatever venue- or domain-specific knowledge has been loaded for this
    deployment (e.g. store info, product details) — not for vision ("what
    do you see") or anything outside what's actually in the knowledge base.
    """
    facts = search_knowledge(config.KB_PROFILE, question, top_k=config.KB_TOP_K)
    if not facts:
        return "nothing in the knowledge base matches that question"
    return " ".join(facts)


# ── driving ──────────────────────────────────────────────────────────────────
# One tool per direction rather than one `drive(direction)`: the small local
# models this also runs on pick a named tool far more reliably than they fill in
# an enum argument, and the docstrings are what they steer by.


def _seconds(value) -> float | None:
    """Coerce a model-supplied duration to a float, or None to use the default.

    Models hand this back as a string often enough ("2"), and occasionally as
    something that is not a duration at all. A junk value falls back to the
    default hop rather than failing the call — the value is clamped downstream
    either way, so a wrong one can only ever be short.
    """
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _drive(direction: str, seconds) -> str:
    body = get_body()
    if body is None or not body.can_drive:
        return NO_WHEELS
    try:
        return body.move(direction, _seconds(seconds))
    except Exception as exc:  # a dying serial link must not kill the turn
        log.exception("drive %s failed", direction)
        return f"the wheels didn't respond: {exc}"


@tool
def move_forward(seconds: float | None = None) -> str:
    """Drive Orio forward a short distance, then stop.

    Use when asked to move forward, go ahead, or come closer. `seconds` is how
    long to drive and is optional — leave it out for a normal short hop, and
    only pass a bigger number if asked for a longer move.

    Orio steers around obstacles by itself while doing this, and stops if it
    cannot find a way through. The result says what actually happened — it may
    report steering around something, turning on the spot, or stopping early.
    Report what it says; never assume the move went as asked.
    """
    return _drive("forward", seconds)


@tool
def move_backward(seconds: float | None = None) -> str:
    """Drive Orio backward a short distance, then stop.

    Use when asked to back up, reverse, or move away. `seconds` is optional, as
    for move_forward.

    Orio's cameras only face forward, so reversing is BLIND — nothing can steer
    it away from what is behind. Keep these short. It still refuses to move at
    all if the cameras have stopped working.
    """
    return _drive("backward", seconds)


@tool
def turn_left(seconds: float | None = None) -> str:
    """Turn Orio to the left in place, then stop.

    Use when asked to turn or look left. The wheels spin opposite ways, so Orio
    pivots where it stands rather than driving anywhere. `seconds` is optional
    and controls how far it swings round; leave it out for a normal turn.
    Refuses to turn if the cameras have stopped working.
    """
    return _drive("left", seconds)


@tool
def turn_right(seconds: float | None = None) -> str:
    """Turn Orio to the right in place, then stop.

    Use when asked to turn or look right. Pivots in place like turn_left.
    `seconds` is optional and controls how far it swings round.
    """
    return _drive("right", seconds)


@tool
def stop_moving() -> str:
    """Stop Orio's wheels right now. Takes no arguments.

    Use whenever asked to stop, halt, hold still, or wait. Safe to call at any
    time, including when Orio is already stopped.
    """
    body = get_body()
    if body is None or not body.can_drive:
        return NO_WHEELS
    try:
        return body.stop()
    except Exception as exc:
        log.exception("stop failed")
        return f"the wheels didn't respond: {exc}"


@tool
def set_speed(percent: float) -> str:
    """Set how fast Orio drives, as a percent of full power.

    Use when asked to go faster, slower, or at some particular speed. Pass a
    number: higher is faster. Anything outside the range Orio is allowed to
    drive at is pulled back to the nearest end and the result says so, so report
    the speed that comes back rather than the one that was asked for. This only
    changes later moves — it does not make Orio move.
    """
    body = get_body()
    if body is None or not body.can_drive:
        return NO_WHEELS
    value = _seconds(percent)
    if value is None:
        return f"{percent!r} isn't a speed — give a number of percent"
    return body.set_speed(value)


# ── looking, and going to things ──────────────────────────────────────────────
# Moving the head is its own component and its own tool; walking up to something
# is a behaviour that uses the head, the detector, and the wheels together, and
# lives in seek.py rather than being chained together by the model one blind hop
# at a time.

# What the model is likely to say, mapped to what the detector actually knows.
_LABEL_ALIASES = {
    "people": "person", "human": "person", "someone": "person", "man": "person",
    "woman": "person", "guy": "person", "me": "person", "you": "person",
    "sofa": "couch", "table": "dining table", "plant": "potted plant",
    "tv": "tv", "television": "tv", "monitor": "tv", "screen": "tv",
    "cup": "cup", "mug": "cup", "bin": "vase",
}

def _seeker():
    from .seek import Seeker

    detector = get_detector()
    return Seeker(get_body(), detector.detect_in)


@tool
def look_around(direction: str) -> str:
    """Turn Orio's head to look somewhere, and report what is there.

    `direction` is "left", "right", "ahead", "up" or "down". Use it when asked
    to look somewhere, or to check whether something is off to one side before
    deciding what to do. This moves only the head — the robot stays where it is.

    Looking left or right pans the head that way and then tilts up and down to
    see the whole of what is over there, so it takes a few seconds. The head
    stays where you pointed it until Orio next drives, which straightens it out.
    """
    body = get_body()
    if body is None or not body.can_look:
        return "Orio can't move its head right now"

    where = str(direction).strip().lower()
    pan, tilts = config.NECK_PAN_DEG, config.LOOK_TILT_SWEEP_DEG
    if where in ("left", "l"):
        pan += config.LOOK_PAN_DEG        # positive pan turns the head left
    elif where in ("right", "r"):
        pan -= config.LOOK_PAN_DEG
    elif where in ("up", "u"):
        tilts = config.LOOK_TILT_UP_DEG
    elif where in ("down", "d"):
        tilts = config.LOOK_TILT_DOWN_DEG
    elif where not in ("ahead", "forward", "centre", "center", "straight"):
        return f"can't look {direction!r} — try left, right, ahead, up or down"

    return f"looked {where}; " + _report(_sweep(body, pan, tilts))


@tool
def go_to(target: str) -> str:
    """Walk over to something and stop in front of it.

    Use this whenever asked to go to, approach, come to, or move toward a thing
    or a person — "come here", "go to the person", "go to the chair". Pass what
    to walk to as a plain word, like "person" or "chair".

    Orio does the whole job itself: it looks around for the target, turns to
    face it, and drives to it, checking every step and steering around anything
    in the way. It stops about a metre short, because that is as close as its
    obstacle avoidance will take it to anything. This takes several seconds and
    the answer says what really happened — it may report arriving, losing sight
    of the target, or not finding one at all. Don't call the move or turn tools
    to do this yourself; they cannot see where they are going.
    """
    body = get_body()
    if body is None or not body.can_drive:
        return NO_WHEELS

    label = str(target).strip().lower()
    label = _LABEL_ALIASES.get(label, label)
    try:
        known = set(get_detector().labels())
    except Exception as exc:
        return f"camera error: {exc}"
    if known and label not in known:
        return (
            f"Orio doesn't know how to recognise a {target!r}, so it can't walk to one"
        )
    try:
        return _seeker().approach(label)
    except Exception as exc:
        log.exception("go_to(%r) failed", target)
        body.stop()
        return f"something went wrong on the way: {exc}"


# The drive half of the tool set, by name. llm.py picks the movement half of
# the system prompt from whether these actually bound this run, so Orio's claim
# about being able to move always matches the tools it really has.
DRIVE_TOOLS = [
    move_forward, move_backward, turn_left, turn_right, stop_moving, set_speed, go_to,
]
DRIVE_TOOL_NAMES = frozenset(t.name for t in DRIVE_TOOLS)


def get_tools() -> list:
    """Build the enabled tool list.

    Each capability is added only if it is actually there: the drive tools need
    a drivetrain that opened (body.start() must already have run), and the
    vision tool needs the CV deps and a camera. Anything missing drops out of
    the list rather than failing the run.
    """
    if not config.TOOLS_ENABLED:
        return []
    tools = [search_knowledge_base]

    body = get_body()
    if body is not None and body.can_drive:
        tools.extend(DRIVE_TOOLS)
    else:
        log.warning("drive tools unavailable; Orio cannot move this run")
    if body is not None and body.can_look:
        tools.append(look_around)

    try:
        import cv2  # noqa: F401
        import ultralytics  # noqa: F401
    except Exception as exc:
        log.warning("vision tool unavailable (%s); Orio runs without it", exc)
        return tools
    tools.append(what_do_you_see)
    return tools


def close_tools() -> None:
    """Release any held hardware (camera). Call on shutdown."""
    global _detector
    if _detector is not None:
        _detector.close()
        _detector = None
