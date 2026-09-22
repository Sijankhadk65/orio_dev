"""Stop the wheels when the body is not sitting on them properly.

Two things this catches, both of which end with the wheels zeroed:

  * **Tipped** — pitch or roll past a limit. A robot on its way over should not
    still be driving itself further over.
  * **Lifted** — someone picked it up. Wheels spinning in the air are no use to
    anyone and are unpleasant to be holding.

This is the one IMU feature with no fallback. Everything else the IMU does —
measured turns, heading hold — degrades to the timed behaviour that existed
before the part was fitted. A guard cannot degrade: with no reading it simply
is not guarding, which is exactly where the robot was last week, so a missing
IMU must not stop the robot driving (see `config.IMU_ENABLED`).

## Why the limits are what they are

**Roll is the trustworthy angle and pitch is not.** Measured on this chassis
2026-09-22: rest pitch wandered 1.7 deg across one run with nothing touching
the robot, because the castor swivels and the body pitches with it, while roll
held to 0.1 deg. Every threshold here has to clear that slop, so a pitch limit
is necessarily coarse. It is not a fine instrument; it is the difference
between "on a ramp" and "going over".

**A lift is only visible while it is happening.** An accelerometer held
perfectly still in the air reads 1 g, exactly like one on the floor — gravity
is all it can see. What it does see is the *transient*: being picked up pushes
the total above 1 g, setting down or a stumble drops it below, and a genuine
drop approaches zero. So this catches the moment of lifting, not the state of
being held, and `LIFT_CONFIRM_S` is short for that reason. Wheels that spin
freely at no current are the other half of this evidence and live in
`bump.py`'s telemetry, not here.

## Latching

An alarm is sticky: once tipped, the robot stays stopped until the readings
say level again for `CLEAR_S`, and a reading that never comes keeps it
stopped. The asymmetry is deliberate — starting to drive again is the
dangerous direction, and a half-second of chatter around the limit would
otherwise have the wheels stuttering on and off while the robot is balanced on
something.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from . import config

TIPPED = "tipped"
LIFTED = "lifted"


@dataclass(frozen=True)
class Alarm:
    """Why the robot must not be driving, in words a caller can report."""

    kind: str  # TIPPED or LIFTED
    reason: str
    at: float


class TiltGuard:
    """Watches body attitude, and says when the wheels must be still.

    One instance per `Body`, ticked wherever drive commands are sent. Reads
    only: it never commands anything itself, it returns a verdict and lets the
    caller stop the wheels, the same division of labour `bump.py` uses.
    """

    def __init__(
        self,
        pitch_limit_deg: float = config.IMU_TILT_PITCH_DEG,
        roll_limit_deg: float = config.IMU_TILT_ROLL_DEG,
        lift_low_mg: float = config.IMU_LIFT_LOW_MG,
        lift_high_mg: float = config.IMU_LIFT_HIGH_MG,
        tilt_confirm_s: float = config.IMU_TILT_CONFIRM_S,
        lift_confirm_s: float = config.IMU_LIFT_CONFIRM_S,
        clear_s: float = config.IMU_TILT_CLEAR_S,
    ) -> None:
        self.pitch_limit_deg = pitch_limit_deg
        self.roll_limit_deg = roll_limit_deg
        self.lift_low_mg = lift_low_mg
        self.lift_high_mg = lift_high_mg
        self.tilt_confirm_s = tilt_confirm_s
        self.lift_confirm_s = lift_confirm_s
        self.clear_s = clear_s

        self._bad_since: float | None = None  # when the current fault started
        self._bad_kind: str | None = None
        self._good_since: float | None = None  # when it last started looking fine
        self._alarm: Alarm | None = None

    @property
    def alarm(self) -> Alarm | None:
        """The standing alarm, without ticking anything."""
        return self._alarm

    def reset(self) -> None:
        """Forget everything. For a caller that has stopped driving entirely."""
        self._bad_since = self._good_since = self._bad_kind = None
        self._alarm = None

    def update(self, reading, now: float | None = None) -> Alarm | None:
        """One tick. Returns the standing alarm, or None when driving is fine.

        `reading` is a calibrated `imu.Reading`, or None when there is nothing
        fresh. None never *raises* an alarm — no reading is no evidence — but
        it will not clear a standing one either: proving the robot is level
        again takes a reading that says so.
        """
        now = time.monotonic() if now is None else now

        if reading is None:
            return self._alarm

        fault = self._fault(reading)
        if fault is None:
            self._bad_since = self._bad_kind = None
            if self._alarm is None:
                return None
            # Latched: hold the alarm until it has looked fine for a while.
            if self._good_since is None:
                self._good_since = now
            elif now - self._good_since >= self.clear_s:
                self._alarm = None
                self._good_since = None
            return self._alarm

        self._good_since = None
        kind, reason = fault
        confirm = self.lift_confirm_s if kind == LIFTED else self.tilt_confirm_s
        if self._bad_since is None or self._bad_kind != kind:
            self._bad_since = now
            self._bad_kind = kind
        elif now - self._bad_since >= confirm:
            # Re-made every tick so the numbers in the message stay current.
            self._alarm = Alarm(kind=kind, reason=reason, at=now)
        return self._alarm

    def _fault(self, reading) -> tuple[str, str] | None:
        """What is wrong with this reading, or None if nothing is.

        Lift is checked first: a robot being carried is usually also tilted,
        and "someone picked me up" is the more useful thing to say about it.
        """
        accel = reading.accel_mg
        if accel < self.lift_low_mg:
            return LIFTED, (
                f"the robot is being lifted or is falling ({accel:.0f} mg, "
                f"below {self.lift_low_mg:.0f})"
            )
        if accel > self.lift_high_mg:
            return LIFTED, (
                f"the robot is being lifted or knocked ({accel:.0f} mg, "
                f"above {self.lift_high_mg:.0f})"
            )
        if abs(reading.roll_deg) > self.roll_limit_deg:
            side = "right" if reading.roll_deg > 0 else "left"
            return TIPPED, (
                f"the robot is leaning {abs(reading.roll_deg):.0f} degrees to the {side}"
            )
        if abs(reading.pitch_deg) > self.pitch_limit_deg:
            way = "back" if reading.pitch_deg > 0 else "forward"
            return TIPPED, (
                f"the robot is tipped {abs(reading.pitch_deg):.0f} degrees {way}"
            )
        return None
