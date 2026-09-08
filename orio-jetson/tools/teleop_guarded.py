#!/usr/bin/env python3
"""WASD teleop that steers around obstacles instead of stopping at them.

    uv run python tools/teleop_guarded.py                      # /dev/orio_drive
    uv run python tools/teleop_guarded.py --port /dev/ttyACM1
    uv run python tools/teleop_guarded.py --duty 40 --clear-m 1.5

Tap W and the robot cruises forward on its own, steering around what it sees
and carrying on. It is the operator who says "go" and "stop"; between those,
the heading is chosen here, from `orio/stereo.py`'s sector map. A/S/D remain
manual overrides for the times you want to place the robot by hand.

## Choosing a heading

Straight on is preferred and only given up when it has to be. Each tick:

    ahead clearer than --clear-m      drive straight, full duty
    ahead blocked but a sector is     steer toward the best sector, forward
      open beyond --stop-m              speed reduced by how hard it turns
    nothing open beyond --stop-m      pivot in place toward the roomier side
    nothing KNOWN anywhere            stop; blind is not a heading

"Best" is the most distant sector, penalised for how far it sits off straight
ahead (`--turn-penalty`), so a marginally roomier route 30 deg off-axis loses
to a good-enough one dead ahead. The robot commits to a turn once it starts —
a same-side bonus plus a low-pass on the steering output — because a policy
that re-picks freely will happily oscillate between two equally good gaps and
make no progress at all.

## Unknown is not an obstacle, and not a target either

A sector with too few valid pixels reports `None`, and the distinction matters
more here than it did for a stop-only guard. **Unknown is never steered
toward**: it means a blank wall, a dark corner, or something closer than the
~0.25 m the cameras can triangulate at, and committing to a heading you cannot
see is how a robot finds furniture. But unknown is deliberately NOT treated as
a repulsive obstacle either, because sector 0 reads UNKNOWN permanently at 0%
valid pixels — it is the undefined black border rectification leaves at the
frame edge, measured still 0% after calibration. Scoring that as a wall would
put a phantom obstacle on the far left of every frame and bias every turn to
the right forever. Only *known* sectors vote.

## What has not changed

Forward motion is still floored: with the path ahead nearer than --stop-m the
robot pivots, it never drives. Reverse and manual turns are never gated, so a
cornered robot can always be recovered by hand. A stale reading (older than
--stale-s, against a ~30 Hz sensor thread) halts forward motion, so a wedged
camera stops the robot rather than leaving it driving on a frozen picture of
an empty corridor.

Distances are only as good as `models/stereo/calibration.npz`; without it the
script says so at startup and every threshold below is approximate.

Stereo needs BOTH sensors, so nothing else may hold a camera: stop the vision
tool and the main app (`ORIO_STEREO=0`, `ORIO_VISION_DEBUG=0`) first.

Controls:
  W          cruise forward, avoiding    [ / ]    duty -/+ 5%
  S          reverse (manual, ungated)   space    stop
  A / D      turn in place (manual)      G        toggle avoidance off/on
  Q / Esc    quit (stops first)

POSIX only: it wants a cbreak terminal, and the CSI cameras mean it only runs
on the Jetson anyway.
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.drivetrain import Drivetrain
from orio.stereo import ObstacleDetector

# The drivetrain board, by its stable udev name — never a raw /dev/ttyACM*,
# whose number is enumeration order and can point at the motion board
# instead. See config.DRIVETRAIN_PORT for why and how to override.
DEFAULT_PORT = config.DRIVETRAIN_PORT
DEFAULT_DUTY_PERCENT = 30.0
DUTY_STEP_PERCENT = 5.0
LOOP_TICK_S = 0.03

# (left_sign, right_sign) — every direction is just a sign pair for the
# firmware's one primitive, Wheel_SetSpeeds().
DIRECTION_KEYS = {
    b"w": (1, 1),
    b"s": (-1, -1),
    b"a": (-1, 1),
    b"d": (1, -1),
}
DIRECTION_NAMES = {(1, 1): "forward", (-1, -1): "reverse", (-1, 1): "left", (1, -1): "right"}


class KeyReader:
    """Non-blocking single-key reads, via cbreak mode plus a zero-timeout
    select() on stdin. Restores the terminal on the way out — without that the
    shell is left unusable. cbreak keeps ISIG enabled, so Ctrl+C still works."""

    def __enter__(self) -> "KeyReader":
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc_info) -> bool:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        return False

    def kbhit(self) -> bool:
        return bool(select.select([sys.stdin], [], [], 0)[0])

    def getch(self) -> bytes:
        return sys.stdin.read(1).encode()


@dataclass(frozen=True)
class Reading:
    """The latest stereo reading, or the error that prevented one.

    `sectors` is (angle_deg, distance_m or None) per sector, left to right —
    the whole map, because choosing a heading needs the sectors either side of
    straight ahead, not just the clearance in front.
    """

    sectors: tuple[tuple[float, float | None], ...]
    clearance_m: float | None
    describe: str
    timestamp: float
    error: str | None = None

    @staticmethod
    def failed(exc: Exception) -> "Reading":
        return Reading(
            sectors=(),
            clearance_m=None,
            describe="stereo read failed",
            timestamp=time.monotonic(),
            error=f"{type(exc).__name__}: {exc}",
        )


class Sensor:
    """Runs the stereo pipeline on its own thread and publishes the latest
    reading.

    Threaded rather than inline because a reading costs ~33 ms (measured, and
    frame-rate bound), which would be felt as keyboard lag on a 30 ms tick — and
    more importantly because a camera that wedges must not also wedge the loop
    that is holding the robot's watchdog alive. A stalled thread simply stops
    republishing, the reading goes stale, and the guard blocks. Failure ends up
    pointing the safe way.
    """

    def __init__(self, guard_sectors: int) -> None:
        self._guard_sectors = guard_sectors
        self._detector = ObstacleDetector()
        self._reading: Reading | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.calibrated = False

    def start(self) -> None:
        # The first reading opens both Argus pipelines and takes ~2 s. Do it
        # here, before the terminal goes into cbreak mode, so any camera error
        # is readable and lands before the operator can press a key.
        omap = self._detector.sense()
        self.calibrated = omap.calibrated
        self._publish(omap)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _publish(self, omap) -> None:
        with self._lock:
            self._reading = Reading(
                sectors=tuple((s.angle_deg, s.distance_m) for s in omap.sectors),
                clearance_m=omap.clearance_ahead(self._guard_sectors),
                describe=omap.describe(),
                timestamp=time.monotonic(),
            )

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._publish(self._detector.sense())
            except Exception as exc:
                with self._lock:
                    self._reading = Reading.failed(exc)
                time.sleep(0.5)  # a failing camera should not spin the CPU

    @property
    def reading(self) -> Reading | None:
        with self._lock:
            return self._reading

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._detector.close()


@dataclass(frozen=True)
class Decision:
    """What the avoider decided, and why."""

    left: int
    right: int
    state: str  # "cruise" | "steer" | "pivot" | "halted" | "off" | "manual"
    reason: str
    heading_deg: float = 0.0


class Avoider:
    """Turns a sector map into a steering command.

    A reactive policy, deliberately: it has no map, no memory of where it has
    been, and no goal beyond "keep going forward without hitting anything".
    That is the right amount of machinery for a bench tool, but it does set the
    limits worth knowing:

    * It wanders rather than travels. Having avoided something it carries on in
      whatever direction it ended up facing; nothing pulls it back to its
      original heading, because nothing here knows what that was.
    * A concave dead end is escaped only by luck and the back-off timer, not by
      reasoning. In simulation it does get out — but by reversing and trying
      another way, which is a different thing from knowing it is in a trap.
    * There is no rear or side sensing. The corridor test covers what the
      cameras can see ahead; the back-off reverses blind.
    """

    def __init__(
        self,
        stop_m: float,
        clear_m: float,
        min_scale: float,
        stale_s: float,
        turn_penalty: float,
        turn_gain: float,
        smooth: float,
        hysteresis_m: float,
        half_width: float,
        release_m: float,
        commit_clear_s: float,
        pivot_timeout_s: float,
        backoff_s: float,
    ) -> None:
        self.stop_m = stop_m
        self.clear_m = clear_m
        self.min_scale = min_scale
        self.stale_s = stale_s
        self.turn_penalty = turn_penalty
        self.turn_gain = turn_gain
        self.smooth = smooth
        self.hysteresis_m = hysteresis_m
        self.half_width = half_width
        self.release_m = release_m
        self.commit_clear_s = commit_clear_s
        self.enabled = True
        self.pivot_timeout_s = pivot_timeout_s
        self.backoff_s = backoff_s
        self._turn = 0.0  # smoothed steering state, permille
        self._committed_side = 0  # -1 left, +1 right, 0 none — damps oscillation
        self._blocked = False
        self._stuck_s = 0.0
        self._clear_s = 0.0
        self._must_clear = False
        self._last_tick: float | None = None
        self._backoff_until = 0.0

    def reset(self) -> None:
        self._turn = 0.0
        self._committed_side = 0
        self._blocked = False
        self._stuck_s = 0.0
        self._clear_s = 0.0
        self._must_clear = False
        self._backoff_until = 0.0

    # ── helpers ──────────────────────────────────────────────────────────────

    def _ahead(self, reading: Reading) -> float | None:
        """Nearest KNOWN obstacle inside the robot's forward corridor, or None
        if nothing in it can be seen (which is not the same as clear).

        The corridor is geometric, not a fixed count of centre sectors, and the
        difference is the difference between avoiding an obstacle and driving a
        shoulder into it. A sector counts when the point it reports lies within
        `half_width` of the centre line — `distance * sin(angle)`. That window
        widens in angle as things get closer, which is exactly the behaviour
        wanted: at 3 m a reading 21 deg off-axis is 1.1 m to the side and no
        threat, while at 0.5 m the same angle is 0.18 m away and squarely in
        the robot's path.

        Using the middle three sectors instead, as a stop-only guard could get
        away with, spans just +/-0.1 m at 0.5 m range — narrower than the robot.
        An obstacle being steered around would leave that window while still
        physically ahead, the clearance would jump to whatever lay beyond it,
        and the policy would accelerate into it. That is not hypothetical: it
        collided in every obstacle scenario in simulation before this changed.
        """
        near = None
        for angle, distance in reading.sectors:
            if distance is None:
                continue
            if abs(distance * math.sin(math.radians(angle))) <= self.half_width:
                near = distance if near is None else min(near, distance)
        return near

    def _pick(self, reading: Reading) -> tuple[float, float] | None:
        """Best (angle, distance) to head for, or None if nothing qualifies.

        Only KNOWN sectors are candidates. Unknown ones are skipped rather than
        scored as obstacles: sector 0 is permanently unknown (the rectification
        border), and treating that as a wall would bias every turn rightward.
        """
        best = None
        best_score = float("-inf")
        for angle, distance in reading.sectors:
            if distance is None or distance <= self.stop_m:
                continue
            # Distance, minus a penalty for how far off-axis it is, plus a
            # bonus for staying on the side already committed to.
            score = distance - self.turn_penalty * abs(angle) / 45.0
            if self._committed_side and (angle > 0) == (self._committed_side > 0):
                score += self.hysteresis_m
            if score > best_score:
                best_score, best = score, (angle, distance)
        return best

    def _mix(self, linear: int, turn: float, duty: int) -> tuple[int, int]:
        """Differential mix. `turn` > 0 steers LEFT, matching the sign of the
        firmware's wheel pair for an in-place left turn (left reverse, right
        forward)."""
        left = max(-duty, min(duty, round(linear - turn)))
        right = max(-duty, min(duty, round(linear + turn)))
        return left, right

    # ── policy ───────────────────────────────────────────────────────────────

    def decide(self, latch, duty: int, reading: Reading | None) -> Decision:
        now = time.monotonic()
        dt = 0.0 if self._last_tick is None else min(0.2, now - self._last_tick)
        self._last_tick = now

        if not self.enabled:
            return Decision(*_signs(latch, duty), "off", "avoidance disabled")

        # Only forward cruise is planned. Reverse and manual turns pass through
        # untouched so a cornered robot can always be recovered by hand.
        if latch is None or latch != (1, 1):
            self.reset()
            return Decision(*_signs(latch, duty), "manual", "manual command")

        if reading is None:
            return Decision(0, 0, "halted", "no stereo reading yet")
        age = now - reading.timestamp
        if age > self.stale_s:
            return Decision(0, 0, "halted", f"stereo reading stale ({age:.1f}s)")
        if reading.error is not None:
            return Decision(0, 0, "halted", reading.error)

        ahead = self._ahead(reading)

        # Whether forward motion is allowed is a Schmitt trigger, not a bare
        # comparison: `ahead` sits within a few centimetres of --stop-m for as
        # long as the robot is working around an obstacle, and depth noise
        # alone then flips the branch every other tick. That chatter is not
        # cosmetic — it reversed the wheels several times a second and the
        # robot dithered in place instead of turning. Blocking latches at
        # --stop-m and only releases --release-m beyond it.
        if ahead is None:
            blocked = True  # nothing KNOWN in the corridor: blind, not clear
        elif self._blocked:
            blocked = ahead <= self.stop_m + self.release_m
        else:
            blocked = ahead <= self.stop_m
        self._blocked = blocked

        # "Stuck" is measured as time spent pinned near the floor, not as
        # consecutive pivot ticks. The two differ exactly when it matters: a
        # robot alternating pivot/steer on the boundary is making no progress
        # at all, yet every steer tick would reset a consecutive-tick counter
        # and the escape would never fire.
        if ahead is not None and ahead < self.stop_m + self.release_m:
            self._stuck_s += dt
        else:
            self._stuck_s = max(0.0, self._stuck_s - 2.0 * dt)

        # After backing off, turn to face clear road BEFORE driving again.
        # Without this the robot spends the space it just bought driving
        # straight back into the same obstacle: back off, steer, re-approach,
        # back off, forever. In simulation that limit cycle held it at the same
        # 2 m mark for the entire run. Forward motion stays suspended until the
        # way ahead is genuinely open, not merely past the stop floor.
        if self._must_clear and ahead is not None and ahead >= self.clear_m:
            self._must_clear = False

        if not blocked and not self._must_clear:
            # 1. Road clear: go straight, and unwind any turn in progress.
            if ahead is not None and ahead >= self.clear_m:
                # Commitment survives a moment of clear road. Clearing it here
                # unconditionally is wrong for the same reason the raw
                # comparison was: cruise and steer alternate while the robot is
                # still working past an obstacle, so every clear tick wiped the
                # side and the next steer re-picked freely. It flip-flopped
                # between the left and right gap and passed neither.
                self._clear_s += dt
                if self._clear_s > self.commit_clear_s:
                    self._committed_side = 0
                self._stuck_s = 0.0
                self._turn *= 1.0 - self.smooth
                left, right = self._mix(duty, self._turn, duty)
                return Decision(left, right, "cruise", f"{ahead:.2f} m clear ahead")

            # 2. Something is in the way but a sector is open: steer for it.
            target = self._pick(reading)
            if target is not None:
                angle, distance = target
                self._clear_s = 0.0
                # Only a genuinely off-axis heading changes the committed side.
                # Letting a 0 deg target clear it lets the side flip freely on
                # the next pivot, which is half of the dithering above.
                if angle:
                    self._committed_side = 1 if angle > 0 else -1
                desired = -self.turn_gain * (angle / 45.0) * duty
                self._turn += self.smooth * (desired - self._turn)
                near = min(ahead if ahead is not None else distance, distance)
                span = max(self.clear_m - self.stop_m, 1e-6)
                by_range = self.min_scale + (1.0 - self.min_scale) * min(
                    1.0, max(0.0, (near - self.stop_m) / span)
                )
                by_turn = 1.0 - 0.5 * min(1.0, abs(self._turn) / max(duty, 1))
                linear = round(duty * by_range * by_turn)
                left, right = self._mix(linear, self._turn, duty)
                return Decision(
                    left, right, "steer",
                    f"gap at {angle:+.0f} deg, {distance:.2f} m", angle,
                )

        # 3. Backing off, because turning alone cannot always clear the corridor.
        #
        #    Close in, the angle an obstacle must reach to leave the corridor
        #    can exceed the cameras' field of view: at 0.48 m against a 0.25 m
        #    half-width it is 31.4 deg, while the outermost sector centre is
        #    31 deg. The obstacle then cannot leave the corridor however far the
        #    robot turns, and it pivots on the spot indefinitely — which it did
        #    in simulation, for 927 of 1200 ticks. Reversing is the only move
        #    that changes that geometry, since the angle required shrinks as
        #    distance grows.
        #
        #    NOTE there is no rear sensor. This reverses BLIND, which is why it
        #    is slow, brief, and entered only once genuinely stuck.
        if now < self._backoff_until or self._stuck_s > self.pivot_timeout_s:
            if now >= self._backoff_until:
                self._backoff_until = now + self.backoff_s
                self._stuck_s = 0.0
                self._committed_side = 0
                self._must_clear = True
            speed = round(duty * self.min_scale)
            self._turn = 0.0
            return Decision(-speed, -speed, "backoff", "stuck — backing off to turn")

        # 4. Boxed in: pivot in place toward whichever side has more room. No
        #    forward component at all — this is the floor the stop-only guard
        #    used to enforce, and it still holds.
        # Latch the turn direction for the whole pivot episode. Re-deciding it
        # every tick produces a perfect limit cycle: turning left moves the
        # obstacle rightward through the frame, which makes the right side look
        # more open, which reverses the turn, which moves it back. Measured in
        # simulation as a +4.6/-9.2 degree oscillation that ran forever, and no
        # per-tick margin fixes it — the evidence really does swing from 0.5 m
        # to 4 m as the obstacle crosses the centre line.
        #
        # Latching is only safe because the episode is bounded: the stuck timer
        # above forces a back-off, and that clears the commitment so the next
        # attempt is free to choose the other way. An unbounded latch is what
        # made an earlier version pivot into a wall indefinitely.
        side = self._committed_side or self._roomier_side(reading)
        if side is not None:
            self._committed_side = side
            self._turn = -side * self.turn_gain * duty
            left, right = self._mix(0, self._turn, duty)
            where = "left" if side < 0 else "right"
            why = "turning to clear" if self._must_clear else "boxed in"
            return Decision(left, right, "pivot", f"{why}, pivoting {where}")

        # 5. Nothing known anywhere. Blind is not a heading.
        self.reset()
        return Decision(0, 0, "halted", "no known clearance in any sector")

    def _roomier_side(self, reading: Reading) -> int | None:
        """-1 (left) or +1 (right), by the best KNOWN distance on each side.
        None when neither side can be seen at all.

        Commitment biases this choice by `hysteresis_m` but must never lock it.
        Returning the committed side unconditionally — which this did at first —
        means a robot that commits toward a wall pivots into that wall forever,
        because the evidence that the other side is now wide open can no longer
        change its mind. In simulation it spent over half its time pivoting on
        the spot against a corridor wall.
        """
        left = [d for a, d in reading.sectors if a < 0 and d is not None]
        right = [d for a, d in reading.sectors if a > 0 and d is not None]
        if not left and not right:
            return None
        best_left = max(left, default=0.0)
        best_right = max(right, default=0.0)
        if self._committed_side < 0:
            best_left += self.hysteresis_m
        elif self._committed_side > 0:
            best_right += self.hysteresis_m
        return -1 if best_left > best_right else 1


def _signs(latch, duty: int) -> tuple[int, int]:
    """The raw wheel pair a latched direction asks for, ungated."""
    if latch is None:
        return (0, 0)
    return (latch[0] * duty, latch[1] * duty)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="WASD teleop that steers around obstacles instead of stopping.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="STM32 drivetrain serial port")
    parser.add_argument("--duty", type=float, default=DEFAULT_DUTY_PERCENT, help="cruise duty %%")
    parser.add_argument("--stop-m", type=float, default=0.50, help="never drive forward inside this")
    parser.add_argument("--clear-m", type=float, default=1.20, help="straight on is good beyond this")
    parser.add_argument("--min-scale", type=float, default=0.35, help="duty scale at --stop-m")
    parser.add_argument("--stale-s", type=float, default=0.50, help="reading older than this halts")
    parser.add_argument(
        "--turn-penalty", type=float, default=1.0,
        help="metres of clearance a 45 deg detour must be worth to be taken",
    )
    parser.add_argument("--turn-gain", type=float, default=0.9, help="steering strength")
    parser.add_argument(
        "--smooth", type=float, default=0.35,
        help="steering low-pass, 0-1; lower is smoother and slower to react",
    )
    parser.add_argument(
        "--hysteresis-m", type=float, default=0.30,
        help="bonus for staying on the side already turning toward",
    )
    parser.add_argument(
        "--half-width", type=float, default=0.25,
        help="half the robot's width plus clearance margin, in metres — sets how "
             "wide the forward corridor is that must stay clear",
    )
    parser.add_argument(
        "--release-m", type=float, default=0.12,
        help="how far past --stop-m the way must clear before driving again "
             "(hysteresis; stops the steer/pivot branch chattering on noise)",
    )
    parser.add_argument(
        "--commit-clear-s", type=float, default=0.8,
        help="seconds of clear road before the robot stops favouring the side "
             "it was turning toward",
    )
    parser.add_argument(
        "--pivot-timeout-s", type=float, default=2.0,
        help="pivoting longer than this without clearing triggers a back-off",
    )
    parser.add_argument(
        "--backoff-s", type=float, default=1.0,
        help="how long to reverse when stuck — REVERSES BLIND, there is no rear sensor",
    )
    parser.add_argument("--no-avoid", action="store_true", help="start with avoidance off")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.duty <= 100.0:
        print("--duty must be between 0 and 100")
        return 1
    if args.stop_m >= args.clear_m:
        print("--stop-m must be less than --clear-m")
        return 1
    if not 0.0 < args.smooth <= 1.0:
        print("--smooth must be in (0, 1]")
        return 1
    if args.stop_m < config.STEREO_MIN_RANGE_M:
        print(
            f"warning: --stop-m {args.stop_m:.2f} is inside the cameras' minimum range "
            f"({config.STEREO_MIN_RANGE_M:.2f} m). Closer than that reads UNKNOWN, which "
            "is never steered toward anyway."
        )

    avoider = Avoider(
        stop_m=args.stop_m,
        clear_m=args.clear_m,
        min_scale=args.min_scale,
        stale_s=args.stale_s,
        turn_penalty=args.turn_penalty,
        turn_gain=args.turn_gain,
        smooth=args.smooth,
        hysteresis_m=args.hysteresis_m,
        half_width=args.half_width,
        release_m=args.release_m,
        commit_clear_s=args.commit_clear_s,
        pivot_timeout_s=args.pivot_timeout_s,
        backoff_s=args.backoff_s,
    )
    avoider.enabled = not args.no_avoid

    print(__doc__)
    print("--- opening stereo (both sensors, ~2 s) ---")
    sensor = Sensor(3)
    try:
        sensor.start()
    except Exception as exc:
        # StereoCamera's own message already explains sensor contention, which
        # is the usual cause; don't bury it under a traceback.
        print(f"\nstereo failed to start: {exc}")
        sensor.close()
        return 1
    if not sensor.calibrated:
        print(
            "\n*** UNCALIBRATED STEREO — distances are approximate ***\n"
            "    No models/stereo/calibration.npz, so depth uses published optics.\n"
            "    Obstacles rank correctly, but the metres carry real error and\n"
            f"    --stop-m {args.stop_m:.2f} is only as accurate as they are.\n"
            "    Fix with: uv run python tools/calibrate_stereo.py\n"
        )

    print(f"--- opening drivetrain on {args.port} ---")
    duty_percent = args.duty
    latch: tuple[int, int] | None = None
    last_sent: tuple[int, int] | None = None
    last_state: str | None = None
    last_hud = 0.0

    try:
        try:
            link = Drivetrain(args.port).connect()
        except Exception as exc:
            print(
                f"\ncould not open the drivetrain on {args.port}: {exc}\n"
                "  Is the STM32 plugged in? Check `ls -l /dev/orio_* /dev/ttyACM*`.\n"
                "  If /dev/orio_drive is missing, the udev rule is not installed on\n"
                "  this machine (see 99-orio-stm32.rules) — find the drivetrain board\n"
                "  in `ls /dev/serial/by-id/` by its ST-LINK serial and pass it with\n"
                "  --port. Do not guess between ttyACM0 and ttyACM1: the wrong one is\n"
                "  the motion board, and it accepts drive frames without complaint."
            )
            return 1

        with link as dt, KeyReader() as keys:
            print(f"duty={duty_percent:g}%  avoidance={'ON' if avoider.enabled else 'OFF'} "
                  f"(pivot<{args.stop_m:g}m, cruise>{args.clear_m:g}m)\n")
            while True:
                while keys.kbhit():
                    key = keys.getch().lower()
                    if key in (b"q", b"\x1b"):
                        raise KeyboardInterrupt
                    if key == b"[":
                        duty_percent = max(0.0, duty_percent - DUTY_STEP_PERCENT)
                        print(f"\nduty={duty_percent:g}%")
                    elif key == b"]":
                        duty_percent = min(100.0, duty_percent + DUTY_STEP_PERCENT)
                        print(f"\nduty={duty_percent:g}%")
                    elif key == b" ":
                        latch = None
                        avoider.reset()
                        print("\nstop")
                    elif key == b"g":
                        avoider.enabled = not avoider.enabled
                        latch = None  # never hand back an already-moving robot
                        avoider.reset()
                        print(
                            f"\navoidance "
                            f"{'ON' if avoider.enabled else 'OFF — nothing will stop you'}"
                        )
                    elif key in DIRECTION_KEYS:
                        latch = DIRECTION_KEYS[key]
                        avoider.reset()
                        print(f"\n{DIRECTION_NAMES.get(latch, latch)}")

                reading = sensor.reading
                decision = avoider.decide(latch, round(duty_percent * 10), reading)

                # A halt is terminal: the robot is blind or boxed in with
                # nowhere known to go, so drop the latch rather than sit there
                # re-deciding. Steering and pivoting are progress and keep it.
                if decision.state == "halted" and latch is not None:
                    print(f"\nHALTED: {decision.reason}")
                    latch = None
                    avoider.reset()
                elif decision.state != last_state and decision.state in (
                    "steer", "pivot", "backoff",
                ):
                    print(f"\n{decision.state.upper()}: {decision.reason}")
                last_state = decision.state

                command = (decision.left, decision.right)
                if command != last_sent:
                    dt.set_drive(*command)
                    last_sent = command

                rejection = dt.take_rejection()
                if rejection is not None:
                    print(f"\nBOARD: {rejection}")
                    last_sent = None  # resend after a rejection rather than dedupe it away

                now = time.monotonic()
                if now - last_hud > 0.2:
                    last_hud = now
                    ahead = None if reading is None else reading.clearance_m
                    clear = "----" if ahead is None else f"{ahead:.2f}"
                    print(
                        f"\r ahead {clear:>5} m │ {decision.state:<7} │ "
                        f"head {decision.heading_deg:+3.0f}° │ "
                        f"L{command[0]:+5d} R{command[1]:+5d} │ duty {duty_percent:3.0f}% │ "
                        f"avoid {'ON ' if avoider.enabled else 'OFF'} ",
                        end="",
                        flush=True,
                    )

                time.sleep(LOOP_TICK_S)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n--- stopping ---")
        sensor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
