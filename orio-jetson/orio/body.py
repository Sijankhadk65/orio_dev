"""Orio's body: the boards it moves through, and the guard it moves under.

One object owns the hardware for the lifetime of the app — the motion board
holding the neck where this run needs it, the drivetrain turning the wheels, and
the stereo pair watching the ground in front of them. `tools.py` exposes the
wheels to the LLM through here; nothing else in the app talks to a board.

## Avoidance is not a feature, it is the only way to drive

Every move goes through the policy in `avoid.py`. There is no flag to turn it
off, no direction that bypasses it, and no path from a tool call to
`set_drive()` that does not pass a decision from the `Avoider` first. It is not
optional in a stronger sense too: if the cameras will not open, or the neck will
not hold the pose those distances are measured through, then `can_drive` is
False, the drive tools are never offered to the model, and Orio says it cannot
move (see `config.NO_DRIVE_PROMPT`). Blind and moving is not a state this can
reach — you get sighted and moving, or talking and still.

What the policy can and cannot cover is worth being exact about, because a
forward-facing sensor is all there is:

  * **Forward** is fully guarded. The `Avoider` picks the heading every tick —
    cruise, steer around, pivot to find a way through, back off when boxed in,
    or halt — and this loop just carries out what it decided.
  * **Turns** pivot in place, so they translate the robot nowhere. Nothing on
    the sides is sensed, and pivoting does sweep the corners of a 0.70 m chassis.
  * **Reverse is blind. There is no rear sensor at all.** The policy's own
    back-off reverses blind for the same reason; nothing can be done about it
    here beyond keeping it brief.

The one guard that *does* apply to all four is sight itself: a move refuses to
start, and stops mid-hop, whenever the reading is missing, failed, or staler
than `config.AVOID_STALE_S`. A wedged camera stops the robot in every direction
rather than leaving it driving on a frozen picture of an empty corridor.

## A move is a bounded hop, not a latch

The teleop tool holds a direction down at 30 Hz while a key is pressed; the LLM
issues one command and goes back to talking. So `move()` runs the same control
loop the teleop tool runs, for a bounded span, and stops the wheels in a
`finally` — the wheels cannot outlive the tool call that started them.
`config.DRIVE_MAX_STEP_S` bounds how far a single command can carry the robot.

## The head is aimed first, and held

Before anything drives, the neck goes to `config.NECK_PAN_DEG` / `NECK_TILT_DEG`
and *stays* there. Aiming matters because every threshold in `avoid.py` is a
distance measured through this head: a head pointing somewhere else measures
somewhere else while reporting the same numbers. Holding matters because the
motion board's e-stop cuts the servo PWM rather than freezing it, and the board
e-stops itself 500 ms after the last heartbeat, so a released neck sags. The
head stays put only while the link stays open — hence the motion link lives as
long as the app does, and is released, deliberately, on the way out.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from . import config
from .avoid import Sensor, avoider_from_config
from .drivetrain import Drivetrain
from .motion import JOINT_NECK, Motion, travel_time_s

log = logging.getLogger(__name__)

# (left_sign, right_sign) — the firmware has one primitive, Wheel_SetSpeeds(),
# and every direction is a sign pair for it. "left" is left wheel reverse, right
# wheel forward: a pivot, not an arc.
DIRECTIONS: dict[str, tuple[int, int]] = {
    "forward": (1, 1),
    "backward": (-1, -1),
    "left": (-1, 1),
    "right": (1, -1),
}

FORWARD = DIRECTIONS["forward"]

# Past tense: these are reported after the hop has finished.
_MOVED = {
    "forward": "moved forward",
    "backward": "backed up",
    "left": "turned left",
    "right": "turned right",
}

# How the policy's states read out loud. Only the ones that mean something
# happened the asker did not ask for; "cruise" is the unremarkable case.
_STATE_WORDS = {
    "steer": "steering around something in the way",
    "pivot": "turning on the spot to find a way past something",
    "backoff": "backing away from something it got too close to",
}

# A hop that ends before this has not really moved, so it is reported as a
# refusal rather than as a move that stopped early.
_BARELY_MOVED_S = 0.15

NO_WHEELS = "the wheels aren't connected right now"

# A cruise whose caller stopped renewing the latch — it wedged, or died
# somewhere its context manager could not see. See Cruise's deadman.
NOT_RENEWED = "nothing was steering any more, so Orio stopped"


@dataclass(frozen=True)
class Hop:
    """What one guarded hop actually did.

    `states` is the policy's states in the order they occurred (deduped), so
    `("cruise",)` is an untroubled hop and `("cruise", "steer", "pivot")` is one
    that met something. `halted` is the reason the hop ended early, if it did.
    """

    states: tuple[str, ...]
    halted: str | None
    rejection: object | None
    elapsed: float

    @property
    def blocked(self) -> bool:
        """The robot did not get anywhere: it halted, or spent the hop turning."""
        if self.halted is not None:
            return True
        return bool(self.states) and set(self.states) <= {"pivot", "backoff"}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Body:
    """The robot's boards and its avoidance policy, held open for the session."""

    def __init__(
        self,
        drive_port: str = config.DRIVETRAIN_PORT,
        motion_port: str = config.MOTION_PORT,
    ) -> None:
        self._drive_port = drive_port
        self._motion_port = motion_port
        self._drive: Drivetrain | None = None
        self._neck: Motion | None = None
        self._sensor: Sensor | None = None
        self._avoider = avoider_from_config()
        # The cruise currently running, if any — at most one, since there is one
        # set of wheels. See cruise().
        self._cruise: "Cruise | None" = None
        # Where the head actually is, as far as this object knows: the pose the
        # last accepted move_to() commanded. None until one has been accepted.
        self._head: tuple[float, float] | None = None
        # Re-entrant so stop() is callable from inside a move's own thread.
        self._lock = threading.RLock()
        self.speed_percent = _clamp(
            config.DRIVE_SPEED_PERCENT,
            config.DRIVE_SPEED_MIN_PERCENT,
            config.DRIVE_SPEED_MAX_PERCENT,
        )
        # Human-readable startup lines for the UI layer to print. Collected
        # rather than printed so this module stays quiet like vision/eyes do.
        self.notes: list[str] = []

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> "Body":
        """Bring the body up, in the order the parts depend on each other.

        The head is aimed before the cameras open, because it is what they look
        through; the cameras open before the wheels, because nothing may turn a
        wheel until something is watching. Any of the three failing costs the
        robot its wheels for the session — never the conversation.
        """
        if not config.DRIVE_ENABLED:
            if config.NECK_ENABLED:
                self._open_neck()
            self.notes.append("driving disabled (ORIO_DRIVE=0) — Orio will say it can't move")
            return self

        if config.NECK_ENABLED:
            self._open_neck()
            if self._neck is None:
                self.notes.append(
                    "⚠ not driving: the head could not be placed, and every distance "
                    "the avoidance policy uses is measured through it. Fix the neck, "
                    "or set ORIO_NECK=0 to drive with the head wherever it happens "
                    "to be lying."
                )
                return self
        else:
            self.notes.append(
                "⚠ ORIO_NECK=0: driving with the head unaimed and unheld. The "
                "avoidance thresholds describe whatever the cameras happen to be "
                "pointing at, which is not necessarily the ground ahead."
            )

        self._open_sensor()
        if self._sensor is None:
            self.notes.append("⚠ not driving: no obstacle sensing, so no driving")
            return self

        self._open_drivetrain()
        return self

    @property
    def can_drive(self) -> bool:
        """Wheels AND eyes. Never one without the other."""
        return self._drive is not None and self._sensor is not None

    def _open_neck(self) -> None:
        try:
            link = Motion(self._motion_port).connect()
        except Exception as exc:
            self.notes.append(
                f"⚠ could not open the motion board on {self._motion_port} ({exc}). "
                f"The head stays wherever it is — slack, if nothing else holds it. "
                f"Check `ls -l /dev/orio_*`."
            )
            return

        try:
            rejection = link.move_to(JOINT_NECK, config.NECK_PAN_DEG, config.NECK_TILT_DEG)
        except Exception as exc:
            link.close()
            self.notes.append(f"⚠ neck pose failed: {exc}")
            return

        if rejection is not None:
            # OUT_OF_RANGE means the angles are outside kJointLimits[] and
            # NOTHING moved. That window is still being characterised, so check
            # the firmware's limits rather than the defaults here.
            link.close()
            self.notes.append(
                f"⚠ the motion board refused the neck pose (pan "
                f"{config.NECK_PAN_DEG:g}°, tilt {config.NECK_TILT_DEG:g}°): "
                f"{rejection.reason} — nothing moved"
            )
            return

        self._neck = link
        self._head = (config.NECK_PAN_DEG, config.NECK_TILT_DEG)
        # Read the pose back, but do not over-read what that proves. The board
        # echoes the angle it was commanded, not one it measured: sweeping tilt
        # past its mechanical stop (~50 deg) had STATUS reporting 80 deg happily
        # while the view through the cameras did not change at all. So this
        # confirms the command reached the joint controller and was not
        # rejected — it does NOT confirm the head physically got there. The only
        # check for that is looking at what the cameras see.
        status = link.read_status()
        angles = None if status is None else status.joints.get("neck")
        if angles is None:
            self.notes.append(
                f"neck posed at pan {config.NECK_PAN_DEG:g}°, tilt "
                f"{config.NECK_TILT_DEG:g}° ({link.identity}) — no STATUS came back, "
                f"so the pose is commanded but unconfirmed"
            )
        else:
            self.notes.append(f"neck holding at {angles} ({link.identity})")

    def _open_sensor(self) -> None:
        sensor = Sensor()
        try:
            # The first reading opens both Argus pipelines and takes ~2 s.
            sensor.start()
        except Exception as exc:
            sensor.close()
            self.notes.append(
                f"⚠ stereo failed to start: {exc}\n"
                f"    Stereo needs BOTH sensors, so nothing else may hold one — "
                f"check that no other run of the app, the vision debug window, or "
                f"a bench tool is using a camera."
            )
            return
        self._sensor = sensor
        if sensor.calibrated:
            self.notes.append("obstacle avoidance armed (stereo calibrated)")
        else:
            self.notes.append(
                "⚠ obstacle avoidance armed, but stereo is UNCALIBRATED — no "
                "models/stereo/calibration.npz, so depth falls back to published "
                f"optics. Obstacles still rank correctly, but the "
                f"{config.AVOID_STOP_M:.2f} m stop distance is only as accurate as "
                "those metres are. Fix with: uv run python tools/calibrate_stereo.py"
            )

    def _open_drivetrain(self) -> None:
        try:
            link = Drivetrain(self._drive_port).connect()
        except Exception as exc:
            self.notes.append(
                f"⚠ driving disabled: could not open the drivetrain on "
                f"{self._drive_port} ({exc}). Check `ls -l /dev/orio_*`; do not "
                f"guess between ttyACM0 and ttyACM1, the wrong one is the motion "
                f"board and it takes a velocity float as a joint angle."
            )
            return
        self._drive = link
        # connect() has already refused a link to the wrong board; naming what
        # answered makes a swapped board visible in a session that DID start.
        self.notes.append(
            f"drivetrain ready at {self.speed_percent:g}% speed ({link.identity})"
        )

    @property
    def sensor(self) -> Sensor | None:
        """The running stereo thread, for anything that needs a camera frame.

        With avoidance always on, this thread holds both sensors for the whole
        session. Argus will not hand a third handle to the same sensor, so the
        vision tool takes its picture from here rather than opening its own.
        """
        return self._sensor

    def close(self) -> None:
        """Stop the wheels, release the cameras, and let the head go."""
        self._end_cruise()  # before the lock, for the reason stop() gives
        with self._lock:
            if self._drive is not None:
                self._drive.close()  # zeroes the wheels and latches the e-stop
                self._drive = None
            if self._sensor is not None:
                self._sensor.close()
                self._sensor = None
            if self._neck is not None:
                # There is no way to leave the board holding a pose: it releases
                # every servo when it e-stops, and it e-stops on its own 500 ms
                # after the last heartbeat. The head goes slack here either way.
                self._neck.close()
                self._neck = None

    # ── commands ─────────────────────────────────────────────────────────────

    @property
    def can_look(self) -> bool:
        """The head can be pointed and something can be seen through it."""
        return self._neck is not None and self._sensor is not None

    @property
    def head_pan(self) -> float:
        """Where the head is pointing, in vendor degrees. Higher is further LEFT."""
        return config.NECK_PAN_DEG if self._head is None else self._head[0]

    @property
    def head_is_driving_pose(self) -> bool:
        return self._head == (config.NECK_PAN_DEG, config.NECK_TILT_DEG)

    def look(self, pan_deg: float, tilt_deg: float | None = None) -> str | None:
        """Point the head. Returns None on success, or why the board refused.

        Tilt defaults to `config.NECK_TILT_DEG` and should usually stay there:
        it is the avoidance policy's aim, and the window that sees the ground
        ahead is narrow (see the sweep in config.py). Pan is the free axis.
        """
        if self._neck is None:
            return "the head isn't connected right now"
        tilt = config.NECK_TILT_DEG if tilt_deg is None else tilt_deg
        try:
            rejection = self._neck.move_to(JOINT_NECK, pan_deg, tilt)
        except Exception as exc:
            return f"the head didn't respond: {exc}"
        if rejection is not None:
            return f"that angle was refused: {rejection.reason}"
        was = self._head
        self._head = (pan_deg, tilt)
        # The ACK above means "accepted", not "arrived": the board starts a
        # motion profile and answers straight away. Wait the travel out here,
        # where how far the head just went is known, so no caller has to know
        # it — they are left waiting only on the camera (config.SEEK_SETTLE_S).
        # A head whose pose was never recorded could be anywhere, so it costs
        # the width of the pan window.
        if was is None:
            time.sleep(travel_time_s(270.0))
        else:
            time.sleep(travel_time_s(max(abs(pan_deg - was[0]), abs(tilt - was[1]))))
        return None

    def restore_head(self) -> str | None:
        """Put the head back where driving needs it."""
        return self.look(config.NECK_PAN_DEG, config.NECK_TILT_DEG)

    def _ensure_driving_pose(self) -> None:
        """Never drive on a view the policy was not calibrated against.

        Anything may have pointed the head elsewhere — a look_around, a scan for
        something to walk to — and every threshold in `avoid.py` is a distance
        measured through this head. Re-aiming here means a move can never
        inherit somebody else's aim, and the sensor gets a moment to publish a
        reading from the pose that will actually be driven on.
        """
        if self._neck is None or self.head_is_driving_pose:
            return
        if self.restore_head() is None:
            time.sleep(config.SEEK_SETTLE_S)

    def snapshot(self):
        """`(Reading, frame)` from the same sensor tick, or `(None, None)`."""
        if self._sensor is None:
            return None, None
        return self._sensor.snapshot()

    def _blind(self, reading) -> str | None:
        """Why this reading cannot be driven on, or None if it can.

        Applied to every direction, including the ones the policy cannot steer.
        Being unable to see is a reason not to move at all, not merely a reason
        not to move forward.
        """
        if reading is None:
            return "the cameras haven't given a reading yet"
        if reading.error is not None:
            return f"the cameras are failing: {reading.error}"
        age = time.monotonic() - reading.timestamp
        if age > config.AVOID_STALE_S:
            return f"the cameras stopped updating ({age:.1f} s stale)"
        return None

    def move(self, direction: str, seconds: float | None = None) -> str:
        """Drive one guarded hop and stop. Returns what happened, in words.

        Blocking, for the length of the hop, and running the avoidance policy
        every `config.AVOID_TICK_S` throughout. What comes back describes what
        the robot actually did — steered, pivoted, backed off, stopped early —
        not what was asked for, because those differ often enough to matter.
        """
        signs = DIRECTIONS.get(direction)
        if signs is None:
            return f"error: unknown direction {direction!r}"

        default_s = (
            config.DRIVE_STEP_S if direction in ("forward", "backward") else config.TURN_STEP_S
        )
        span = _clamp(default_s if not seconds else float(seconds), 0.1, config.DRIVE_MAX_STEP_S)
        hop = self.hop(direction, signs, span)
        return self._describe(direction, span, hop)

    def hop(self, direction: str, signs: tuple[int, int], span: float) -> "Hop":
        """One guarded hop, as structured facts rather than a sentence.

        `move()` wraps this for the LLM; `seek.py` drives it directly, because a
        behaviour deciding whether to keep approaching needs to know the policy
        halted rather than read that out of English.
        """
        self._ensure_driving_pose()

        with self._lock:
            link, sensor = self._drive, self._sensor
            if link is None or sensor is None:
                return Hop((), NO_WHEELS, None, 0.0)
            duty = round(self.speed_percent * 10)  # permille, what the board takes
            # Each hop is its own episode: the commitment and stuck timers from
            # the last one describe a situation the robot may have been carried
            # away from by hand between turns.
            self._avoider.reset()
            link.take_rejection()  # drop anything stale so we report this hop's

            states: list[str] = []
            halted: str | None = None
            rejection = None
            last_sent: tuple[int, int] | None = None
            started = time.monotonic()
            deadline = started + span
            try:
                while time.monotonic() < deadline:
                    state, halted, rejection, last_sent = self._drive_tick(
                        link, sensor, signs, duty, last_sent
                    )
                    if state is not None and (not states or states[-1] != state):
                        states.append(state)
                    if halted is not None or rejection is not None:
                        break
                    time.sleep(config.AVOID_TICK_S)
            finally:
                # The one thing that must always happen. A hop that is not
                # explicitly ended keeps rolling: the firmware re-sends the last
                # duty to the FSESCs so its own timeout never cuts the motor,
                # and the heartbeat thread keeps the board armed.
                link.set_drive(0, 0)
            elapsed = time.monotonic() - started

        return Hop(tuple(states), halted, rejection, elapsed)

    def _drive_tick(self, link, sensor, signs, duty, last_sent):
        """One pass of the guarded control loop: look, decide, send.

        The whole of what `hop()` and `Cruise` have in common, and the only
        place a duty pair reaches the board. Returns
        `(state, halted, rejection, last_sent)` — `state` is the policy's state
        this tick or None if it never got as far as deciding, and `halted` is
        set when the robot must not be moving right now.

        Caller-agnostic on purpose: it neither sleeps nor zeroes the wheels, so
        a bounded hop can break out of its loop and a cruise can keep ticking
        through the same condition. Both must be holding `self._lock`.
        """
        reading = sensor.reading
        halted = self._blind(reading)
        if halted is not None:
            # Sight failed before the policy was consulted, so there is no state
            # to report — just the reason nothing may move.
            return None, halted, None, last_sent

        decision = self._avoider.decide(signs, duty, reading)
        if decision.state == "halted":
            return decision.state, decision.reason, None, last_sent

        command = (decision.left, decision.right)
        if command != last_sent:
            link.set_drive(*command)
            last_sent = command
        return decision.state, None, link.take_rejection(), last_sent

    def cruise(self) -> "Cruise":
        """Start a guarded drive that outlives the call which started it.

        `hop()` is the right shape for the LLM, which issues one command and
        goes back to talking. It is the wrong shape for a closed-loop behaviour
        like `seek.py`, where it means the wheels are zeroed once per look and
        the robot walks in a stutter. A cruise keeps the control loop running on
        its own thread while the caller perceives, so the latch survives a
        detector pass instead of being ended by one.

        Only one cruise runs at a time; starting a second ends the first. Use it
        as a context manager — leaving the block stops the wheels.
        """
        self._end_cruise()
        self._ensure_driving_pose()
        with self._lock:
            self._avoider.reset()
            if self._drive is not None:
                self._drive.take_rejection()  # this cruise reports its own
        cruise = Cruise(self)
        self._cruise = cruise
        cruise.start()
        return cruise

    def _cruise_tick(self, signs, last_sent):
        """One locked tick on a cruise's behalf, including the held case.

        Separate from `_drive_tick` because a cruise, unlike a hop, has to keep
        ticking through the conditions that end a hop: held with no direction
        latched, and halted. Both mean the wheels are zeroed and the loop goes
        round again, because the situation can clear — a stale reading comes
        back, or the caller latches a new direction — and the caller's own
        deadline is what ends the drive.
        """
        with self._lock:
            link, sensor = self._drive, self._sensor
            if link is None or sensor is None:
                return None, NO_WHEELS, None, last_sent
            if signs is None:
                return None, None, None, self._zero(link, last_sent)
            duty = round(self.speed_percent * 10)  # permille, what the board takes
            state, halted, rejection, last_sent = self._drive_tick(
                link, sensor, signs, duty, last_sent
            )
            if halted is not None:
                last_sent = self._zero(link, last_sent)
            return state, halted, rejection, last_sent

    @staticmethod
    def _zero(link, last_sent):
        """Stop the wheels, without re-sending a stop that is already standing."""
        if last_sent != (0, 0):
            link.set_drive(0, 0)
        return (0, 0)

    def _end_cruise(self) -> None:
        """Stop whatever cruise is running, if any. Never call holding the lock:
        `Cruise.close()` joins a thread that takes it every tick."""
        cruise, self._cruise = self._cruise, None
        if cruise is not None:
            cruise.close()

    def _stop_wheels(self) -> None:
        """Zero the wheels, for a cruise winding itself up."""
        with self._lock:
            if self._drive is not None:
                self._drive.set_drive(0, 0)

    def _describe(self, direction: str, span: float, hop: "Hop") -> str:
        """Turn a hop into the sentence the model will read back to the person."""
        states, halted, rejection, elapsed = hop.states, hop.halted, hop.rejection, hop.elapsed
        if rejection is not None:
            return f"the drive board refused that: {rejection}"
        if halted == NO_WHEELS:
            return NO_WHEELS

        moved = _MOVED[direction]
        # What the policy did that the asker did not ask for, in the order it
        # happened, minus the states that mean "carrying on normally".
        detours = [_STATE_WORDS[s] for s in states if s in _STATE_WORDS]

        if halted is not None:
            if elapsed < _BARELY_MOVED_S:
                return f"didn't move — {halted}"
            return f"{moved} for {elapsed:.1f} seconds and then stopped: {halted}"

        # Forward that spent the whole hop pivoting never actually advanced.
        if direction == "forward" and states and set(states) <= {"pivot", "backoff"}:
            return (
                f"couldn't go forward — something was in the way, so Orio spent the "
                f"{elapsed:.1f} seconds {detours[0] if detours else 'looking for a way past'}"
            )

        summary = f"{moved} for {span:.1f} seconds at {self.speed_percent:g}% speed"
        if detours:
            summary += ", " + " then ".join(dict.fromkeys(detours))
        return summary

    def stop(self) -> str:
        """Zero the wheels now.

        Mostly belt-and-braces — every hop stops itself — but it is what "stop"
        has to mean, and it covers a hop left running by a crash.

        A cruise is ended first, and deliberately before the lock is taken: its
        thread commands the wheels every tick, so zeroing them underneath one
        would last until its next tick, and joining it from inside the lock it
        wants would not finish at all.
        """
        self._end_cruise()
        with self._lock:
            if self._drive is None:
                return NO_WHEELS
            self._drive.set_drive(0, 0)
            self._drive.stop()  # e-stop; the heartbeat clears it within 200 ms,
            # but the firmware zeroes the commanded speeds before resuming, so
            # the wheels stay stopped.
        return "stopped"

    def set_speed(self, percent: float) -> str:
        """Set the duty every later hop runs at, clamped to the safe window."""
        low = config.DRIVE_SPEED_MIN_PERCENT
        high = config.DRIVE_SPEED_MAX_PERCENT
        wanted = float(percent)
        self.speed_percent = _clamp(wanted, low, high)
        if self.speed_percent != wanted:
            return (
                f"speed set to {self.speed_percent:g}% — {wanted:g}% is outside "
                f"the {low:g}–{high:g}% range Orio is allowed to drive at"
            )
        return f"speed set to {self.speed_percent:g}%"


class Cruise:
    """A guarded drive that keeps running while its caller looks at something.

    The wheels are latched by direction rather than commanded for a span:
    `go()` says which way and `hold()` says not right now, and the control loop
    on this thread keeps deciding at `config.AVOID_TICK_S` regardless of what
    the caller is doing. That is the whole point — a behaviour that re-detects
    between moves stutters if perceiving ends the move, and detection is slow
    enough to be visible in the robot's gait.

    ## The guard runs more, not less

    Nothing about the avoidance policy is relaxed here. Every tick goes through
    the same `Body._drive_tick` a hop does, so the same staleness check stops
    the robot on a wedged camera and the same `Avoider` picks every heading. The
    difference is that it now also runs *during* the caller's detector pass,
    where a hop-shaped approach has the robot standing still and nothing
    watching at all.

    One thing does change behaviourally: the `Avoider`'s commitment and stuck
    timers now survive a look. A hop reset them on every call, so an approach
    that looked every 0.8 s wiped them far more often than the manoeuvres they
    exist to hold together, and a policy already committed to steering around
    something could forget it was mid-detour. A cruise resets them once, when it
    starts. Pivots still clear them — `Avoider.decide` resets on any latch that
    is not straight forward — so what is preserved is commitment across a run of
    forward looks, which is exactly where it was being lost.

    ## The deadman

    A latch that outlives its caller is the failure this shape introduces: if
    the caller wedges — a detector that never returns, a behaviour that throws
    somewhere the context manager cannot see — the last direction would stand
    and the robot would keep driving, guarded but unsupervised. So a latch
    expires `config.CRUISE_DEADMAN_S` after the `go()` that set it, and the
    wheels stop until something renews it. Callers re-latch every pass around
    their own loop, which costs them nothing and makes the renewal the proof
    that they are still there.

    It is the same bargain twice more below this: the firmware re-sends the duty
    to the FSESCs every 200 ms or their own timeout cuts the motor, and the
    Jetson heartbeats the board every 200 ms or its watchdog e-stops. Motion
    here is something that has to be continuously re-earned.
    """

    def __init__(self, body: "Body") -> None:
        self._body = body
        # Guards the latch and everything drain() reports. Held only to read or
        # write those fields, never across a tick: the tick takes Body's lock,
        # and close() is called by a thread holding neither.
        self._lock = threading.Lock()
        self._signs: tuple[int, int] | None = None
        self._renewed = 0.0
        self._direction: str | None = None
        self._states: list[str] = []
        self._halted: str | None = None
        self._rejection = None
        self._since = time.monotonic()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "Cruise":
        self._thread = threading.Thread(target=self._loop, daemon=True, name="cruise")
        self._thread.start()
        return self

    # ── the latch ────────────────────────────────────────────────────────────

    def go(self, direction: str) -> None:
        """Drive `direction` until told otherwise, or until the deadman expires.

        Idempotent and cheap: call it every pass around a perception loop. The
        wheels do not restart on each call — the loop is already running and the
        board is already holding the duty — so this only renews the latch.
        """
        signs = DIRECTIONS.get(direction)
        if signs is None:
            log.warning("cruise asked for unknown direction %r; holding", direction)
            self.hold()
            return
        with self._lock:
            self._signs = signs
            self._direction = direction
            self._renewed = time.monotonic()

    def hold(self) -> None:
        """Stop the wheels, keep the loop running. The robot stands still until
        the next `go()`, with the policy still ticking underneath it."""
        with self._lock:
            self._signs = None
            self._direction = None

    @property
    def direction(self) -> str | None:
        """What is latched right now, or None if the robot is holding."""
        with self._lock:
            return self._direction

    # ── what happened ────────────────────────────────────────────────────────

    def drain(self) -> Hop:
        """Everything the policy did since the last drain, and clear it.

        Reported as a `Hop` so a caller can ask the same questions of a window
        of a cruise that it used to ask of one hop — `blocked` in particular
        means exactly what it did before, over the stretch between two looks
        rather than over a 0.8 s move.
        """
        with self._lock:
            now = time.monotonic()
            window = Hop(tuple(self._states), self._halted, self._rejection, now - self._since)
            self._states = []
            self._halted = None
            self._rejection = None
            self._since = now
            return window

    def _record(self, state, halted, rejection) -> None:
        with self._lock:
            if state is not None and (not self._states or self._states[-1] != state):
                self._states.append(state)
            if halted is not None:
                self._halted = halted
            if rejection is not None:
                self._rejection = rejection

    def _latched(self) -> tuple[int, int] | None:
        """The current direction, or None if held or the deadman has expired."""
        with self._lock:
            if self._signs is None:
                return None
            if time.monotonic() - self._renewed <= config.CRUISE_DEADMAN_S:
                return self._signs
            self._signs = None
            self._direction = None
        # Reported outside the lock, which _record takes for itself. The caller
        # is gone or stuck, so this is the last thing said about the drive.
        log.warning("cruise latch expired after %gs unrenewed", config.CRUISE_DEADMAN_S)
        self._record(None, NOT_RENEWED, None)
        return None

    # ── the loop ─────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        last_sent: tuple[int, int] | None = None
        try:
            while not self._stop.wait(config.AVOID_TICK_S):
                state, halted, rejection, last_sent = self._body._cruise_tick(
                    self._latched(), last_sent
                )
                self._record(state, halted, rejection)
                if halted == NO_WHEELS:
                    break
        except Exception:
            log.exception("cruise loop died; the wheels are being stopped")
            self._record(None, "the drive loop failed", None)
        finally:
            # The same promise hop() makes in its own finally, and for the same
            # reason: nothing below this layer ever stops on its own while the
            # heartbeat keeps the board armed.
            self._body._stop_wheels()

    def close(self) -> None:
        """End the cruise and leave the wheels stopped. Safe to call twice."""
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                log.error("cruise thread would not stop; stopping the wheels anyway")
        self._body._stop_wheels()

    def __enter__(self) -> "Cruise":
        return self

    def __exit__(self, *exc_info) -> bool:
        self.close()
        return False


# ── the one body this process has ────────────────────────────────────────────
# A module-level singleton for the same reason tools.py keeps one detector: the
# boards and cameras are physical, only one link to each can exist, and the tool
# functions the LLM calls take no context of their own. run() owns its lifetime.

_body: Body | None = None


def start() -> Body:
    """Open the body (idempotent) and return it."""
    global _body
    if _body is None:
        _body = Body().start()
    return _body


def get() -> Body | None:
    """The open body, or None if one was never started."""
    return _body


def close() -> None:
    """Release the boards and cameras. Call on shutdown."""
    global _body
    if _body is not None:
        _body.close()
        _body = None
