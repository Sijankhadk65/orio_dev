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
    close the distance  guarded forward hops, re-detecting between each one
    stop               when the target is inside `SEEK_ARRIVE_M`, or lost, or
                       the policy will not go nearer, or the clock runs out

Every step re-checks, because both ends of the problem move: the robot drifts
off heading (no odometry, and a pivot is timed rather than measured) and a
person does not stand still. Nothing here integrates; each decision is made
from the picture in front of it.

## Where the bearing comes from

`stereo.obstacles()` splits the frame into equal-width columns and charges each
one a distance, so the sector a detection's bounding-box centre falls into is
just `int(cx / width * n)`. That is an exact correspondence rather than an
approximation, and it means the target's range comes from the same depth map
the avoidance policy is steering on, with no field-of-view arithmetic in
between to get wrong. It also means the two are only consistent if the box and
the depth came from the same frame — hence `Sensor.snapshot()`.

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
import time
from dataclasses import dataclass

from . import config
from .body import DIRECTIONS

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Sighting:
    """One detection, placed in the world the avoidance policy sees."""

    label: str
    confidence: float
    bearing_deg: float        # sector centre; negative is left of straight ahead
    distance_m: float | None  # that sector's range, or None if it reads unknown
    head_pan_deg: float       # where the head was pointing when this was taken

    @property
    def side(self) -> str:
        return "left" if self.bearing_deg < 0 else "right"


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
        angle, distance = reading.sectors[index]
        return Sighting(label, best.confidence, angle, distance, self._body.head_pan)

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

        while time.monotonic() < deadline:
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
                self._turn(hint)
                continue

            lost = 0
            last = seen
            hint = seen.side

            if abs(seen.bearing_deg) > config.SEEK_CENTRE_DEG:
                self._turn(seen.side)
                continue

            if seen.distance_m is not None and seen.distance_m <= config.SEEK_ARRIVE_M:
                return (
                    f"went to the {label} — standing {seen.distance_m:.1f} m away, "
                    f"facing them"
                )

            hop = self._body.hop("forward", DIRECTIONS["forward"], config.SEEK_HOP_S)
            if hop.halted == "the wheels aren't connected right now":
                return hop.halted
            if hop.blocked:
                blocked += 1
                # The policy would not take it any nearer. Once is a moment of
                # noise and worth retrying; three times running is an answer.
                if blocked >= 3:
                    where = _describe_distance(seen.distance_m)
                    reason = hop.halted or "something is in the way"
                    return (
                        f"got as close to the {label} as it could — {where}, "
                        f"and then {reason}"
                    )
            else:
                blocked = 0

        where = _describe_distance(last.distance_m) if last else ""
        return f"gave up going to the {label} after {config.SEEK_TIMEOUT_S:g} seconds — {where}"

    def _turn(self, side: str) -> None:
        """One short pivot toward `side`. Timed, not measured — hence "short":
        the correction comes from looking again, not from turning accurately."""
        self._body.hop(side, DIRECTIONS[side], config.SEEK_TURN_BURST_S)
