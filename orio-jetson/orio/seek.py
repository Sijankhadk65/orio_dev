"""Going to something Orio can see.

"Go to the person in front of you" is not a move, it is a behaviour, and the
difference is the whole reason this file exists. A move is open-loop: point the
wheels, run them for a bounded time, stop. Asking an LLM to reach a person by
emitting moves makes it guess a distance it cannot measure and a heading it
cannot see, one blind hop at a time — which is exactly what it did, and why the
robot "just moved a bit" and stopped.

So the model picks the target and nothing else. Everything below is
deterministic and closed-loop:

    look for it        sweep the head across `config.SEEK_SCAN_OFFSETS_DEG`,
                       running the detector at each stop, nearest instance wins
    point at it        pivot the chassis in short bursts until the target sits
                       within `SEEK_CENTRE_DEG` of straight ahead
    close the distance  one guarded drive, re-detecting while it rolls
    stop               when the target is inside `SEEK_ARRIVE_M`, or lost, or
                       the policy will not go nearer, or the clock runs out

Every step re-checks, because both ends of the problem move: the robot drifts
off heading (no odometry, and a pivot is timed rather than measured) and a
person does not stand still. Nothing here integrates; each decision is made
from the picture in front of it.

## The approach cruises; the pivots still do not

Closing the distance used to be a run of bounded hops, which meant the wheels
were zeroed once per look — the robot walked up to people in a stutter, rolling
for `SEEK_HOP_S` and then standing still for as long as the detector took. It
now latches `body.Cruise` forward and leaves it latched while it looks, so
looking no longer costs motion and the guard keeps ticking through the pass
(see `Cruise`, which is where that safety argument lives).

Pivots are deliberately NOT cruised, and the reason is the same one that makes
them short: a pivot is timed, not measured, and the correction comes from
looking again. Held down, a pivot would keep swinging through each detector
pass, so the robot would answer every look with a turn made partly stale by the
turn itself — the classic way to oscillate around `SEEK_CENTRE_DEG` instead of
settling into it. They stay bursts: latch, `SEEK_TURN_BURST_S`, stop, look.
Turns are also where the guard is thinnest, since nothing senses the sides.

## Where the bearing comes from

`stereo.obstacles()` splits the frame into equal-width columns and charges each
one a distance, so the sector a detection's bounding-box centre falls into is
just `int(cx / width * n)`. That is an exact correspondence rather than an
approximation, and it means the target's RANGE comes from the same depth map
the avoidance policy is steering on. It also means the two are only consistent
if the box and the depth came from the same frame — hence `Sensor.snapshot()`.

The BEARING does not come from the sector. A sector is 13 deg wide, far too
coarse to aim with (see `_bearing`), so the angle is taken from the box centre
and each pivot turns no further than the target is off.

## Approaching and avoiding are the same manoeuvre

A person is an obstacle. The policy steers around anything nearer than
`AVOID_CLEAR_M` and refuses to drive forward at all inside `AVOID_STOP_M`, so
past that range "go to them" and "don't hit them" are opposite instructions and
the guard wins — it is not negotiable, and this behaviour does not get an
exemption. `SEEK_ARRIVE_M` therefore defaults to `AVOID_CLEAR_M`: Orio arrives
where the two agree, roughly a metre away, which is where you would stop in
front of someone anyway.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass

from . import config
from .body import NO_WHEELS

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Sighting:
    """One detection, placed in the world the avoidance policy sees."""

    label: str
    confidence: float
    bearing_deg: float        # box centre; negative is left of straight ahead
    distance_m: float | None  # that sector's range, or None if it reads unknown
    head_pan_deg: float       # where the head was pointing when this was taken

    @property
    def side(self) -> str:
        return "left" if self.bearing_deg < 0 else "right"


def _bearing(centre_x: float, width: int) -> float:
    """Degrees off straight ahead of pixel column `centre_x`, negative left.

    From the box centre itself rather than its sector's centre: with 7 sectors
    over the Gemini's 94 deg a sector is 13 deg wide, so the sector angle can
    only say 0 or +-13 — and +-13 is past SEEK_CENTRE_DEG on either side of a
    target sitting 10 deg off, which is how the approach ended up swinging
    left-right-left without ever driving. Pinhole, principal point assumed
    central (the D2C colour frame's is within a few pixels of it).
    """
    half = math.radians(config.STEREO_HFOV_DEG) / 2.0
    offset = (2.0 * centre_x / width - 1.0) * math.tan(half)
    return math.degrees(math.atan(offset))


def _describe_distance(metres: float | None) -> str:
    if metres is None:
        return "an unknown distance away"
    return f"about {metres:.1f} m away"


class Seeker:
    """Finds a labelled thing and closes the distance to it."""

    def __init__(self, body, detect) -> None:
        self._body = body
        self._detect = detect  # callable(frame) -> list[Detection]

    # ── perception ───────────────────────────────────────────────────────────

    def sight(self, label: str) -> Sighting | None:
        """Look once, from wherever the head is now.

        The nearest instance wins, measured by bounding-box area — with several
        people in frame, walking to the furthest is never what was meant.
        """
        reading, frame = self._body.snapshot()
        if reading is None or frame is None or not reading.sectors:
            return None
        try:
            found = [d for d in self._detect(frame) if d.label == label]
        except Exception:
            log.exception("detector failed while looking for %r", label)
            return None
        if not found:
            return None

        best = max(found, key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]))
        width = frame.shape[1]
        n = len(reading.sectors)
        centre_x = (best.bbox[0] + best.bbox[2]) / 2.0
        index = min(n - 1, max(0, int(centre_x / width * n)))
        _, distance = reading.sectors[index]
        return Sighting(label, best.confidence, _bearing(centre_x, width), distance,
                        self._body.head_pan)

    def scan(self, label: str) -> Sighting | None:
        """Sweep the head over pan AND tilt; leave it where driving needs it.

        The head moves rather than the robot because turning the chassis to look
        around costs floor space and risks a pivot into something off to the
        side that nothing can see. The head is free.

        Both axes, because one tilt is one horizontal slice of the room — a
        person two metres off reads fine at the driving tilt, the same person at
        arm's length is a pair of legs below the frame, and anything on a shelf
        is above it. The grid is walked tilt-major with the driving tilt first,
        so the usual case (someone standing in front of the robot) costs one
        stop rather than the whole sweep.
        """
        try:
            for tilt in config.SEEK_SCAN_TILTS_DEG:
                for offset in config.SEEK_SCAN_OFFSETS_DEG:
                    refusal = self._body.look(config.NECK_PAN_DEG + offset, tilt)
                    if refusal is not None:
                        log.info("scan skipping pan %+g tilt %g: %s", offset, tilt, refusal)
                        continue
                    time.sleep(config.SEEK_SETTLE_S)
                    seen = self.sight(label)
                    if seen is not None:
                        return seen
            return None
        finally:
            # Whatever happened, driving must not inherit the scan's aim.
            self._body.restore_head()

    # ── behaviour ────────────────────────────────────────────────────────────

    def approach(self, label: str) -> str:
        """Find `label` and walk up to it. Returns what happened, in words."""
        if not self._body.can_drive:
            return "Orio can't move right now, so it can't go to anything"

        seen = self.sight(label) or self.scan(label)
        if seen is None:
            return f"couldn't find a {label} anywhere in view"

        # Which way the head was turned when it found the target is the only
        # hint about where the target is once the head comes back to centre.
        hint = "left" if seen.head_pan_deg > config.NECK_PAN_DEG else "right"
        if abs(seen.head_pan_deg - config.NECK_PAN_DEG) < 1.0:
            hint = seen.side
        self._body.restore_head()

        deadline = time.monotonic() + config.SEEK_TIMEOUT_S
        lost = 0
        blocked = 0
        last: Sighting | None = seen

        # Every exit from here — arrived, lost, blocked, timed out, or an
        # exception nobody expected — leaves through the context manager, which
        # stops the wheels. That is the whole of why the latch is safe to leave
        # standing across a look.
        with self._body.cruise() as cruise:
            while time.monotonic() < deadline:
                looked_at = time.monotonic()
                seen = self.sight(label)

                if seen is None:
                    lost += 1
                    if lost > config.SEEK_MAX_LOST:
                        if last is not None and last.distance_m is not None:
                            return (
                                f"lost sight of the {label} — last saw it "
                                f"{_describe_distance(last.distance_m)} to the {last.side}"
                            )
                        return f"lost sight of the {label}"
                    # Turn toward where it last was and look again.
                    self._turn(cruise, hint, config.SEEK_TURN_DEG)
                    continue

                lost = 0
                last = seen
                hint = seen.side

                if abs(seen.bearing_deg) > config.SEEK_CENTRE_DEG:
                    # Never further than the target is off: a fixed turn wider
                    # than the centre window jumps straight over it, and the
                    # next look sends the robot back the other way, forever.
                    self._turn(cruise, seen.side,
                               min(config.SEEK_TURN_DEG, abs(seen.bearing_deg)))
                    continue

                if seen.distance_m is not None and seen.distance_m <= config.SEEK_ARRIVE_M:
                    return (
                        f"went to the {label} — standing {seen.distance_m:.1f} m away, "
                        f"facing them"
                    )

                # Renew rather than start: on every pass but the first the
                # wheels are already turning, and have been throughout the look
                # that just happened.
                cruise.go("forward")

                # What the policy did over the stretch just driven — which is
                # the look, not a hop, but the question asked of it is the one
                # a hop used to answer.
                drive = cruise.drain()
                if drive.halted == NO_WHEELS:
                    return drive.halted
                if drive.blocked:
                    blocked += 1
                    # The policy would not take it any nearer. Once is a moment
                    # of noise and worth retrying; three times running is an
                    # answer.
                    if blocked >= 3:
                        where = _describe_distance(seen.distance_m)
                        reason = drive.halted or "something is in the way"
                        return (
                            f"got as close to the {label} as it could — {where}, "
                            f"and then {reason}"
                        )
                else:
                    blocked = 0

                # Pace the detector, not the robot: it keeps rolling through
                # this, and a look any sooner would be at much the same picture.
                remaining = config.SEEK_LOOK_PERIOD_S - (time.monotonic() - looked_at)
                if remaining > 0:
                    time.sleep(remaining)

        where = _describe_distance(last.distance_m) if last else ""
        return f"gave up going to the {label} after {config.SEEK_TIMEOUT_S:g} seconds — {where}"

    def _turn(self, cruise, side: str, degrees: float) -> None:
        """One short pivot toward `side`, by ANGLE when the IMU is answering.

        With an IMU the turn ends when the body has turned `degrees`,
        so the same command means the same heading change on carpet as on
        lino, at a flat battery as at a full one. Without one it falls back to
        the timed burst this used to be — unmeasured, but exactly the
        behaviour that existed before the sensor was fitted.

        Two stops besides the angle: `SEEK_TURN_MAX_S`, for a robot that is
        commanded to pivot and does not move (a wheel against a skirting
        board), and the policy itself, which can halt the cruise mid-turn.

        Through the cruise rather than a hop of its own, so the control loop
        this behaviour is driving stays the one loop throughout — but still a
        burst that ends in `hold()`, because a pivot held down through a look
        oscillates (see the module docstring).
        """
        tracker = self._body.turn_tracker()
        cruise.go(side)
        if tracker is None:
            # The burst is calibrated to SEEK_TURN_DEG; a smaller turn gets a
            # proportionally shorter one.
            time.sleep(config.SEEK_TURN_BURST_S * min(1.0, degrees / config.SEEK_TURN_DEG))
        else:
            deadline = time.monotonic() + config.SEEK_TURN_MAX_S
            while time.monotonic() < deadline:
                turned = tracker.turned
                if turned is not None and abs(turned) >= degrees:
                    break
                time.sleep(0.01)
            else:
                log.info(
                    "pivot %s gave up after %.1fs having turned %.0f deg of %.0f",
                    side, config.SEEK_TURN_MAX_S, abs(tracker.turned or 0.0), degrees,
                )
        cruise.hold()
        # A pivot's states are not the approach's. Dropping them keeps the next
        # window purely forward, so `blocked` still answers the question it
        # answered when turns were hops of their own and their states went
        # nowhere: is the policy refusing to take the robot any NEARER.
        cruise.drain()
