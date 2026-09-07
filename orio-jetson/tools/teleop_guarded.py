#!/usr/bin/env python3
"""WASD teleop with a stereo depth guard that stops the robot before it hits things.

    uv run python tools/teleop_guarded.py                      # /dev/ttyACM0
    uv run python tools/teleop_guarded.py --port /dev/ttyTHS1
    uv run python tools/teleop_guarded.py --duty 40 --stop-m 0.6

This is the first thing in this repo that both senses and moves. It joins the
two halves that each deliberately stopped at the same seam: `orio/stereo.py`
reports what is in the way and refuses to decide anything about motion, and the
STM32 accepts wheel duties and knows nothing about cameras. The policy in
between lives here, in a bench tool, where it can be watched and argued with
before it is promoted into the robot proper.

## What the guard actually does

Only *forward* motion is gated. Reverse and turning in place are always allowed
— a guard that blocks everything when you are nose-first against a wall has
trapped you, and the operator's only remaining move would be to power off.

Straight ahead is `ObstacleMap.clearance_ahead(3)`, the nearest obstacle in the
middle three sectors. The outermost sectors are deliberately not consulted:
rectification leaves an undefined black border at the frame edges, so sector 0
reads UNKNOWN permanently, and folding that into the gate would mean the robot
was never allowed to move.

    clearance > --slow-m         full commanded duty
    --stop-m .. --slow-m         duty scaled down linearly toward --min-scale
    < --stop-m, or UNKNOWN,      forward refused, latch cleared
    or reading stale

**Unknown is not clear.** A sector with too few valid pixels reports `None`,
and that is treated exactly like an obstacle. It usually means a blank wall, a
dark room, or something closer than the ~0.25 m the cameras can triangulate at
— all things you want to stop for, not drive into.

**A stale reading is not clear either.** The sensing thread republishes at
~30 Hz; if the newest reading is older than --stale-s the guard blocks forward
motion. A crashed or wedged camera thread therefore stops the robot instead of
leaving it driving on a frozen picture of an empty corridor.

## Latching, and why the two zones behave differently

Direction is latched, as in the drivetrain repo's `teleop_wasd.py`: tap W and it
keeps going. The caution zone only *scales* the latched command, so backing away
from an obstacle smoothly restores speed. A hard stop, by contrast, clears the
latch — you have to press W again. Auto-resuming into an obstacle the moment it
moves out of view is exactly the sort of unprompted lurch that makes a robot
untrustworthy on a bench.

## Before you trust the numbers

With no `models/stereo/calibration.npz` the depth path falls back to published
optics and every distance is approximate — obstacles rank correctly but the
metres carry real error, and --stop-m is only as good as they are. The script
says so loudly at startup. Run `tools/calibrate_stereo.py` to fix it.

Stereo needs BOTH sensors, so nothing else may hold a camera: stop the vision
tool and the main app (`ORIO_STEREO=0`, `ORIO_VISION_DEBUG=0`) first.

Controls:
  W / S      forward / reverse            [ / ]    duty -/+ 5%
  A / D      turn left / right in place   space    stop
  G          toggle the guard off/on      Q / Esc  quit (stops first)

POSIX only: it wants a cbreak terminal, and the CSI cameras mean it only runs
on the Jetson anyway.
"""

from __future__ import annotations

import argparse
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

DEFAULT_PORT = "/dev/ttyACM0"
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


@dataclass
class Reading:
    """The latest stereo reading, or the error that prevented one."""

    clearance_m: float | None
    describe: str
    timestamp: float
    error: str | None = None


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
                    self._reading = Reading(
                        clearance_m=None,
                        describe="stereo read failed",
                        timestamp=time.monotonic(),
                        error=f"{type(exc).__name__}: {exc}",
                    )
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
    """What the guard did to a command, and why."""

    left: int
    right: int
    state: str  # "clear" | "slow" | "blocked" | "off" | "not-forward"
    reason: str
    blocked: bool


class Guard:
    def __init__(self, stop_m: float, slow_m: float, min_scale: float, stale_s: float) -> None:
        self.stop_m = stop_m
        self.slow_m = slow_m
        self.min_scale = min_scale
        self.stale_s = stale_s
        self.enabled = True

    def apply(self, left: int, right: int, reading: Reading | None) -> Decision:
        if not self.enabled:
            return Decision(left, right, "off", "guard disabled", False)

        # Only forward motion is gated. A pure spin sums to zero; reverse is
        # negative. Both stay available so the operator can always escape.
        if left + right <= 0:
            return Decision(left, right, "not-forward", "not driving forward", False)

        if reading is None:
            return Decision(0, 0, "blocked", "no stereo reading yet", True)

        age = time.monotonic() - reading.timestamp
        if age > self.stale_s:
            return Decision(0, 0, "blocked", f"stereo reading stale ({age:.1f}s)", True)
        if reading.error is not None:
            return Decision(0, 0, "blocked", reading.error, True)

        clearance = reading.clearance_m
        if clearance is None:
            return Decision(0, 0, "blocked", "clearance UNKNOWN ahead", True)
        if clearance < self.stop_m:
            return Decision(0, 0, "blocked", f"obstacle at {clearance:.2f} m", True)
        if clearance < self.slow_m:
            span = max(self.slow_m - self.stop_m, 1e-6)
            scale = self.min_scale + (1.0 - self.min_scale) * (clearance - self.stop_m) / span
            return Decision(
                round(left * scale),
                round(right * scale),
                "slow",
                f"{clearance:.2f} m ahead, duty x{scale:.2f}",
                False,
            )
        return Decision(left, right, "clear", f"{clearance:.2f} m ahead", False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="WASD teleop with a stereo depth guard that stops before obstacles.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="STM32 drivetrain serial port")
    parser.add_argument("--duty", type=float, default=DEFAULT_DUTY_PERCENT, help="starting duty %%")
    parser.add_argument("--stop-m", type=float, default=0.50, help="refuse forward inside this")
    parser.add_argument("--slow-m", type=float, default=1.20, help="start slowing inside this")
    parser.add_argument("--min-scale", type=float, default=0.35, help="duty scale at --stop-m")
    parser.add_argument("--stale-s", type=float, default=0.50, help="reading older than this blocks")
    parser.add_argument(
        "--guard-sectors",
        type=int,
        default=3,
        help="how many centre sectors count as 'ahead' (of %d)" % config.STEREO_SECTORS,
    )
    parser.add_argument("--no-guard", action="store_true", help="start with the guard off")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.duty <= 100.0:
        print("--duty must be between 0 and 100")
        return 1
    if args.stop_m >= args.slow_m:
        print("--stop-m must be less than --slow-m")
        return 1
    if args.stop_m < config.STEREO_MIN_RANGE_M:
        print(
            f"warning: --stop-m {args.stop_m:.2f} is inside the cameras' minimum range "
            f"({config.STEREO_MIN_RANGE_M:.2f} m). Closer than that reads UNKNOWN, which "
            "blocks anyway — so the effective stop distance is the minimum range."
        )

    guard = Guard(args.stop_m, args.slow_m, args.min_scale, args.stale_s)
    guard.enabled = not args.no_guard

    print(__doc__)
    print("--- opening stereo (both sensors, ~2 s) ---")
    sensor = Sensor(args.guard_sectors)
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
    last_hud = 0.0

    def desired() -> tuple[int, int]:
        if latch is None:
            return (0, 0)
        permille = round(duty_percent * 10)
        return (latch[0] * permille, latch[1] * permille)

    try:
        try:
            link = Drivetrain(args.port).connect()
        except Exception as exc:
            print(
                f"\ncould not open the drivetrain on {args.port}: {exc}\n"
                "  Is the STM32 plugged in? Check `ls /dev/ttyACM* /dev/ttyTHS*` and "
                "pass the right one with --port."
            )
            return 1

        with link as dt, KeyReader() as keys:
            print(f"duty={duty_percent:g}%  guard={'ON' if guard.enabled else 'OFF'} "
                  f"(stop<{args.stop_m:g}m, slow<{args.slow_m:g}m)\n")
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
                        print("\nstop")
                    elif key == b"g":
                        guard.enabled = not guard.enabled
                        if not guard.enabled:
                            latch = None  # never hand back an already-moving robot
                        print(f"\nguard {'ON' if guard.enabled else 'OFF — nothing will stop you'}")
                    elif key in DIRECTION_KEYS:
                        latch = DIRECTION_KEYS[key]
                        print(f"\n{DIRECTION_NAMES.get(latch, latch)}")

                reading = sensor.reading
                decision = guard.apply(*desired(), reading)
                if decision.blocked and latch is not None:
                    print(f"\nGUARD: forward refused — {decision.reason}")
                    latch = None  # explicit re-press required; no silent resume

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
                    clear = "----" if reading is None or reading.clearance_m is None else (
                        f"{reading.clearance_m:.2f}"
                    )
                    print(
                        f"\r ahead {clear:>5} m │ {decision.state:<11} │ "
                        f"L{command[0]:+5d} R{command[1]:+5d} │ duty {duty_percent:3.0f}% │ "
                        f"guard {'ON ' if guard.enabled else 'OFF'} ",
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
