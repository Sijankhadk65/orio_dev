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
    """A confirmed stall: which side, when, and the evidence for it."""

    side: str  # "left" | "right" | "both"
    at: float
    current_a: float
    erpm: tuple[int, int] = (0, 0)  # (left, right) at the moment it confirmed
    duty: tuple[int, int] = (0, 0)  # what was being commanded

    def describe(self) -> str:
        return (f"{self.side} stalled — erpm L{self.erpm[0]:+d} R{self.erpm[1]:+d} "
                f"at duty L{self.duty[0]:+d} R{self.duty[1]:+d}, {self.current_a:.2f} A")


class StallDetector:
    """Wheel telemetry plus the duty that was asked for -> a confirmed stall.

    Pure and synchronous: no thread, no clock of its own, no serial. `update()`
    is called once per control tick by whoever has both halves of the evidence,
    which is the one place a duty pair reaches the board (`body._drive_tick`).

    A stall is BOTH wheels commanded forward, one or both turning far slower
    than that duty should turn them, held for `confirm_s`.

    "Far slower", not "stopped": a wheel jammed against a wall reads a clean
    zero, but a wheel pushing a real obstacle creeps. Measured on the robot,
    three separate contacts reported both wheels between 0 and 23% of the 447
    eRPM they free-run at, and a flat 50 threshold caught only one wheel of
    each pair — so the map was marked on one side, `_roomier_side` kept picking
    a side that was not open, and the robot drove back into the obstacle.

    And the wheel must be drawing real current (`min_current_a`, 3.5 A). A
    slow wheel alone is also every start: from rest the wheel takes ~0.3 s to
    pass the eRPM limit, longer than `confirm_s`, and without current this
    confirmed a bump on nearly every press of W. Spin-up draws 3-4 A for under
    0.2 s while a jam holds 4-6 A (config.BUMP_STALL_CURRENT_A has the data).
    Current was briefly retired on readings that turned out to be 100x low —
    see drivetrain._current_scale.

    It also separates contact from an ESC that is not driving at all (0 A),
    alongside `Status.estopped`, which remains the explicit check.

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
        erpm_per_mille: float = config.BUMP_STALL_ERPM_PER_MILLE,
        min_current_a: float = config.BUMP_STALL_CURRENT_A,
        confirm_s: float = config.BUMP_CONFIRM_S,
        status_stale_s: float = config.BUMP_STATUS_STALE_S,
    ) -> None:
        self.min_duty = min_duty
        self.max_erpm = max_erpm
        self.erpm_per_mille = erpm_per_mille
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
        self._last_commanded: tuple[int, int] | None = None
        self._since: dict[str, float | None] = {LEFT: None, RIGHT: None}
        # The side-set currently confirmed, or None. Exposed so a caller can log
        # the edge without having to diff `update()`'s return value itself.
        self.active: str | None = None

    def reset(self) -> None:
        self._since = {LEFT: None, RIGHT: None}
        self._last_update = None
        self._last_commanded: tuple[int, int] | None = None
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

        # BOTH wheels must be driving forward. Not one of them — both.
        #
        # Found on the robot 2026-09-18, and it made the feature unusable: an
        # in-place pivot commands one wheel forward and one reverse, and at the
        # 4.5% that leaves, the forward wheel cannot scrub the robot round on a
        # smooth floor. It reports zero eRPM with nothing in front of it, the
        # bump pins `ahead` to 0.00 m, that keeps the policy pivoting, and the
        # pivot stalls it again. The robot ping-ponged between a left bump and
        # a right one and never drove at all.
        #
        # The rule that fixes it is also the honest one: this sensor answers
        # "is something in front of me", and that question only means anything
        # while the robot is trying to go forward. Scrubbing round on the spot
        # is not driving, and a wheel that will not scrub is not a wheel that
        # has hit something. Trap 3 (reversing) falls out of the same test.
        if commanded[0] < self.min_duty or commanded[1] < self.min_duty:
            self._since = {LEFT: None, RIGHT: None}
            self._last_commanded = commanded
            self.active = None
            return None

        # A new duty pair starts a new episode. Without this the timer keeps
        # running across a change of command, so the tail of one manoeuvre can
        # confirm a stall that belongs to the next.
        if commanded != self._last_commanded:
            self._since = {LEFT: None, RIGHT: None}
            self._last_commanded = commanded

        commanded_side = {LEFT: commanded[0], RIGHT: commanded[1]}
        wheels = getattr(status, "wheels", None) or {}
        stalled: list[str] = []
        current = 0.0
        erpm = {LEFT: 0, RIGHT: 0}
        for side in (LEFT, RIGHT):
            tel = wheels.get(side)
            if tel is None or not tel.valid:
                self._since[side] = None
                continue
            erpm[side] = tel.erpm
            # Scaled to the command, not a flat floor. A wheel jammed against a
            # wall reads a clean 0, but a wheel pushing a REAL obstacle creeps —
            # measured at 0 to 23% of free-running, where a flat 50 caught only
            # one wheel of each stalled pair. `Avoider` also throttles duty down
            # while steering, so the same fixed number cannot be both tight
            # enough at cruise and loose enough at a crawl.
            limit = max(self.max_erpm, self.erpm_per_mille * commanded_side[side])
            if abs(tel.erpm) > limit:
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
        return Bump(side=side, at=now, current_a=current,
                    erpm=(erpm[LEFT], erpm[RIGHT]), duty=tuple(commanded))


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
