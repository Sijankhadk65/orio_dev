"""A jammed wheel, read as an obstacle sensor.

The premise is that a stall is not a fault to be reported — it is the most
certain obstacle reading the robot will ever get. Every other sensor here
answers "is something there?" with a probability attached; a wheel that will
not turn under duty answers it with contact. The only questions worth asking
are which side, and how to say so in a shape the policy already understands.

The shape is the one `orio/tof.py` uses. `sectors.Sector` is
`(index, angle_deg, distance_m, valid_frac, source, clear_m)` with nothing in
it about how the distance was obtained, so a bump is a sector reading at zero
range and fusion is `sectors.fuse()`'s existing `min()`. **`avoid.Avoider` needs
no changes at all** — the geometric corridor, the Schmitt trigger on the stop
floor and the back-off logic all keep working on a map that has better numbers
in it. Nothing in this module decides anything about motion.

## Why this is not a camera behaviour

The obvious version of this feature points the head at whatever stopped the
robot. It cannot work, for two independent reasons, both measured rather than
argued:

* The neck's pan window is 165..185 deg — `kJointLimits[]` in
  `orio-stm-motion/Core/Src/servo_joint.c`, and the firmware NACKs anything
  outside it. The head can turn +/-10 deg, against a stereo pair that already
  sees +/-36.5. Looking "to the left" is not a motion this robot has.
* `STEREO_MIN_RANGE_M` is 0.25 m. A stall means contact, so the obstacle is
  a quarter of a metre inside the range where stereo can triangulate at all.
  Aimed perfectly, the cameras return unknown.

So the stall is not a trigger to go and perceive. It is the perception.

## Three traps

1. **The map must be stamped NOW, not when the bump happened.** `fuse()` takes
   the OLDEST timestamp of its inputs, deliberately, so a frozen sensor cannot
   hide behind a fresh one. A memory stamped at contact time would therefore
   age the whole fused map as it decayed, and `AVOID_STALE_S` (0.5 s) would
   halt the robot on arithmetic alone while every sensor was working — exactly
   the bug the ToF fan hit and the reason its staleness is handled the way it
   is. A memory's age belongs in whether it is emitted at all, never in the
   timestamp it is emitted with.
2. **Unknown is not stalled.** `WheelTelemetry.valid` is false when the board
   has nothing current for that wheel, and a missing or stale `Status` means
   the same thing. Neither may confirm a stall: inventing contact stops the
   robot dead for as long as the memory lasts, and the failure is silent.
3. **Only a FORWARD stall is an obstacle ahead.** Stalling while reversing is
   real information about something behind, and there is nowhere to put it —
   the sector grid spans the forward view only, and `avoid.py` says outright
   that back-off reverses blind. Recording a reverse stall in forward sectors
   would put a phantom obstacle in front of a robot whose actual problem is
   behind it. It is dropped here, and stays a gap.

## Why a side bump marks the whole side

`Avoider._roomier_side()` scores each side by its BEST known distance, so a
single zeroed sector on the left changes nothing while any other left sector
still reports clear road — the robot would pivot left, into the thing it just
hit. Marking every sector on that side is what makes the escape correct, and
it is honest: a stalled wheel locates the obstacle to a side, not to a 10 deg
slice. The centre sector (angle exactly 0) is left alone for a one-sided bump
and included for a two-sided one, which is the same distinction `_roomier_side`
itself draws.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from . import config
from .sectors import ObstacleMap, Sector, sector_angles

LEFT = "left"
RIGHT = "right"
BOTH = "both"


@dataclass(frozen=True)
class Bump:
    """A confirmed stall: which side, when, and what it was pulling."""

    side: str  # "left" | "right" | "both"
    at: float
    current_a: float

    def describe(self) -> str:
        return f"{self.side} wheel stalled at {self.current_a:.1f} A"


class StallDetector:
    """Wheel telemetry plus the duty that was asked for -> a confirmed stall.

    Pure and synchronous: no thread, no clock of its own, no serial. `update()`
    is called once per control tick by whoever has both halves of the evidence,
    which is the one place a duty pair reaches the board (`body._drive_tick`).

    A stall is duty commanded and the wheel not turning, held for `confirm_s`.

    There was a third condition — current being drawn — and measuring it on
    2026-09-18 retired it. At the 5% duty this robot is limited to, driving
    draws 0.04 A and a jam draws 0.06, two adjacent readings at the bottom of
    the ESC's range, while eRPM separates 447 from 0. `min_current_a` is
    therefore 0.0 by default, which skips the check; it stays configurable
    because at a duty this robot is not allowed to use it would work.

    That puts all the weight on eRPM, and makes `Status.estopped` load-bearing:
    a board that is refusing to drive looks exactly like a board driving into a
    wall, and only the e-stop flag tells them apart.

    `confirm_s` is not noise filtering. A hub motor takes real milliseconds to
    come up from rest, and the whole of that looks exactly like a stall: duty
    high, eRPM ~0, current at its peak. Confirming instantly would report a
    bump on every start.
    """

    def __init__(
        self,
        *,
        min_duty: int = config.BUMP_MIN_DUTY,
        max_erpm: int = config.BUMP_STALL_ERPM,
        min_current_a: float = config.BUMP_STALL_CURRENT_A,
        confirm_s: float = config.BUMP_CONFIRM_S,
        status_stale_s: float = config.BUMP_STATUS_STALE_S,
    ) -> None:
        self.min_duty = min_duty
        self.max_erpm = max_erpm
        self.min_current_a = min_current_a
        self.confirm_s = confirm_s
        self.status_stale_s = status_stale_s
        # A confirmation timer only means anything if the ticks feeding it were
        # continuous. Between two hops the wheels stop, the loop stops calling
        # this, and the robot may be carried somewhere else entirely; resuming
        # against a timer started before all that would confirm a stall on the
        # first tick of the new hop, from evidence about the old one. Any gap
        # longer than this starts the timers over. Kept here rather than asked
        # of every caller, because a caller that forgets gets a phantom bump
        # and no indication of where it came from.
        self.max_gap_s = status_stale_s
        self._last_update: float | None = None
        self._since: dict[str, float | None] = {LEFT: None, RIGHT: None}
        # The side-set currently confirmed, or None. Exposed so a caller can log
        # the edge without having to diff `update()`'s return value itself.
        self.active: str | None = None

    def reset(self) -> None:
        self._since = {LEFT: None, RIGHT: None}
        self._last_update = None
        self.active = None

    def update(self, now: float, commanded: tuple[int, int], status) -> Bump | None:
        """One tick. Returns a `Bump` for as long as a stall stays confirmed.

        `commanded` is the duty pair actually sent to the board, per-mille,
        `(left, right)`. `status` is `drivetrain.Status` or None.
        """
        gapped = self._last_update is not None and (now - self._last_update) > self.max_gap_s
        self._last_update = now
        if gapped:
            self._since = {LEFT: None, RIGHT: None}
            self.active = None

        if status is None or (now - status.timestamp) > self.status_stale_s:
            # Trap 2: no current evidence is not evidence of contact. Drop the
            # timers too, so a link that comes back does not immediately
            # confirm on the strength of a stall it could not see.
            self.reset()
            return None

        if getattr(status, "estopped", False):
            # An e-stopped board answers CMD_SET_DRIVE with NACK_ESTOPPED and
            # otherwise ignores it (drivetrain.py's module docstring), so the
            # wheels do not turn while duty is being commanded at them. That is
            # duty + zero eRPM, which since the current check was measured
            # useless and disabled is EXACTLY the signature of contact. Without
            # this, an e-stopped robot reports an obstacle in every sector it
            # is pointed at.
            self.reset()
            return None

        wheels = getattr(status, "wheels", None) or {}
        stalled: list[str] = []
        current = 0.0
        for side, duty in ((LEFT, commanded[0]), (RIGHT, commanded[1])):
            tel = wheels.get(side)
            if tel is None or not tel.valid or duty < self.min_duty:
                # Trap 3 lives in `duty < min_duty`: a reverse duty is negative
                # and can never clear the floor, so backing off into something
                # is never recorded as an obstacle ahead.
                self._since[side] = None
                continue
            if abs(tel.erpm) > self.max_erpm:
                self._since[side] = None
                continue
            # Zero or below disables the current condition rather than making
            # it trivially true. It matters that this is a skip and not a
            # comparison: a free-running wheel logged -0.01 A, so `>= 0.0`
            # would veto a real stall on a sign of measurement noise.
            if self.min_current_a > 0.0 and tel.current_a < self.min_current_a:
                self._since[side] = None
                continue
            if self._since[side] is None:
                self._since[side] = now
            if (now - self._since[side]) >= self.confirm_s:
                stalled.append(side)
                current = max(current, tel.current_a)

        if not stalled:
            self.active = None
            return None
        side = BOTH if len(stalled) == 2 else stalled[0]
        self.active = side
        return Bump(side=side, at=now, current_a=current)


class BumpMemory:
    """The last confirmed bump, for as long as it is still true.

    Thread-safe and deliberately tiny: the control loop writes it and the
    sensor thread reads it, and those are different threads at similar rates.

    The decay is the whole design. A bump is a fact about a moment of contact,
    not a landmark — the robot backs off, the obstacle is no longer under the
    wheel, and the memory has to stop claiming it is. Left permanent it would
    wall off one side of the map forever. `memory_s` is therefore long enough
    to survive the back-off it triggers (`AVOID_BACKOFF_S`, plus the pivot that
    follows) and no longer.
    """

    def __init__(
        self,
        *,
        memory_s: float = config.BUMP_MEMORY_S,
        distance_m: float = config.BUMP_DISTANCE_M,
        sectors: int = config.STEREO_SECTORS,
        hfov_deg: float = config.STEREO_HFOV_DEG,
    ) -> None:
        self.memory_s = memory_s
        self.distance_m = distance_m
        self.angles = sector_angles(sectors, hfov_deg)
        self._lock = threading.Lock()
        self._bump: Bump | None = None

    def record(self, bump: Bump | None) -> None:
        """Remember a bump. Called every tick a stall stays confirmed, so the
        memory stays fresh while the wheel is still jammed and starts decaying
        the moment it is not."""
        if bump is None:
            return
        with self._lock:
            self._bump = bump

    def forget(self) -> None:
        with self._lock:
            self._bump = None

    @property
    def last(self) -> Bump | None:
        with self._lock:
            return self._bump

    def fresh(self, now: float | None = None) -> Bump | None:
        """The remembered bump if it is still inside `memory_s`, else None."""
        now = time.monotonic() if now is None else now
        with self._lock:
            bump = self._bump
        if bump is None or (now - bump.at) > self.memory_s:
            return None
        return bump

    def fresh_map(self, now: float | None = None) -> ObstacleMap | None:
        """The bump as a sector map, or None when there is nothing to say.

        Returning None — rather than a map of unknowns — is what makes this
        degrade instead of interfere: `fuse()` skips a None source entirely, so
        for the whole of the time the robot is not stalled this contributes
        nothing at all.
        """
        now = time.monotonic() if now is None else now
        bump = self.fresh(now)
        if bump is None:
            return None

        def marked(angle: float) -> bool:
            if bump.side == BOTH:
                return True
            if bump.side == LEFT:
                return angle < 0
            return angle > 0

        return ObstacleMap(
            sectors=tuple(
                Sector(
                    index=i,
                    angle_deg=angle,
                    distance_m=self.distance_m if marked(angle) else None,
                    valid_frac=1.0 if marked(angle) else 0.0,
                    source="bump" if marked(angle) else "",
                    clear_m=None,
                )
                for i, angle in enumerate(self.angles)
            ),
            # Trap 1. `now`, never `bump.at` — see the module docstring.
            timestamp=now,
            # A contact reading needs no calibration to be true, and saying
            # otherwise would drag `calibrated` false across the fused map and
            # relabel every stereo reading in it as approximate.
            calibrated=True,
        )
