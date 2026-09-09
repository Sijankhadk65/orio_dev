#!/usr/bin/env python3
"""WASD teleop that steers around obstacles instead of stopping at them.

    uv run python tools/teleop_guarded.py                      # /dev/orio_drive
    uv run python tools/teleop_guarded.py --port /dev/ttyACM1
    uv run python tools/teleop_guarded.py --duty 40 --clear-m 1.5
    uv run python tools/teleop_guarded.py --neck-tilt 40        # look further out
    uv run python tools/teleop_guarded.py --no-neck             # leave the head alone

Tap W and the robot cruises forward on its own, steering around what it sees
and carrying on. It is the operator who says "go" and "stop"; between those,
the heading is chosen by the policy, from `orio/stereo.py`'s sector map. A/S/D
remain manual overrides for the times you want to place the robot by hand.

The policy itself now lives in `orio/avoid.py` — it moved there when the main
app started driving through it too, so that both are steering with the same
code. This tool keeps the CLI flags for sweeping the tunables; the app takes
its values from `config.AVOID_*`. Everything documented below still describes
what `Avoider` does, because it is the same object.

## The head is aimed first, and held

Before the cameras open, the neck is driven to pan 175 deg / tilt 25 deg and
*kept* there. Both are needed. Aiming matters because every threshold below is
a distance measured through this head: --stop-m and --clear-m describe the
ground the robot is about to cross, and a head pointing somewhere else measures
somewhere else while reporting the same numbers. Holding matters because of how
the motion board's e-stop works — it cuts the servo PWM rather than freezing it,
so a released neck has no holding torque and sags. The board e-stops itself
500 ms after the last heartbeat, which means the head only stays put for as long
as the link stays open. That link therefore lives for the whole session and is
released, deliberately, on the way out.

Note the neck is the MOTION board's, not the drivetrain's: a second serial port
(--motion-port), a second identity handshake, and a set of angle limits the
firmware is still characterising. If the board refuses the pose the run stops
rather than driving blind-ish; --no-neck skips the head entirely.

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
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.avoid import DEFAULT_HALF_WIDTH_M, Avoider, Sensor
from orio.drivetrain import Drivetrain
from orio.motion import JOINT_NECK, Motion

# The drivetrain board, by its stable udev name — never a raw /dev/ttyACM*,
# whose number is enumeration order and can point at the motion board
# instead. See config.DRIVETRAIN_PORT for why and how to override.
DEFAULT_PORT = config.DRIVETRAIN_PORT

# The neck lives on the OTHER board — the servos are the motion board's, and it
# is a separate serial link with its own identity handshake. See
# config.MOTION_PORT, and orio/motion.py for why the link has to stay open.
DEFAULT_MOTION_PORT = config.MOTION_PORT

# Where the head is put before the run starts, on the vendor's scale (pan
# 0..270, tilt 0..180, each from that servo's own zero end). Tilt 25 points the
# cameras down at the floor immediately ahead, which is the ground the avoider
# is about to steer over; pan 175 is a few degrees off the neck's 180 home, so
# "straight ahead" for the stereo pair is straight ahead for the chassis.
NECK_PAN_DEG = 175.0
NECK_TILT_DEG = 30.0

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
        "--half-width", type=float, default=DEFAULT_HALF_WIDTH_M,
        help="half the robot's width plus clearance margin, in metres — sets how "
             "wide the forward corridor is that must stay clear. Widening it also "
             "moves the distance inside which turning cannot clear the corridor "
             "at all (half-width / sin 31.8 deg); see the back-off note in decide()",
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
    parser.add_argument(
        "--motion-port", default=DEFAULT_MOTION_PORT,
        help="serial port of the MOTION board, which owns the neck servos — a "
             "different board and a different port from --port",
    )
    parser.add_argument(
        "--neck-pan", type=float, default=NECK_PAN_DEG,
        help="neck pan angle held for the run, in degrees on the vendor scale (0..270)",
    )
    parser.add_argument(
        "--neck-tilt", type=float, default=NECK_TILT_DEG,
        help="neck tilt angle held for the run, in degrees on the vendor scale (0..180)",
    )
    parser.add_argument(
        "--no-neck", action="store_true",
        help="skip the neck entirely and do not open the motion board — the head "
             "stays wherever it already is (slack, if nothing else is holding it)",
    )
    parser.add_argument("--no-avoid", action="store_true", help="start with avoidance off")
    return parser.parse_args()


def aim_neck(args) -> Motion:
    """Put the head where this run wants it, and return the link holding it there.

    The neck is on the motion board, not the drivetrain — a separate port, a
    separate identity handshake, and servos rather than wheels. Two firmware
    facts shape this:

    The link cannot be closed afterwards. An e-stop on the motion board runs
    `Servo_Stop()` on every joint, which is `HAL_TIM_PWM_Stop` — the pulse train
    ceases and the joint is released, with no holding torque — and the watchdog
    e-stops the board 500 ms after the last heartbeat. Aim-and-disconnect would
    let the head sag before the first frame was ever captured, so the returned
    link is kept open, heartbeating, for the whole session.

    The angles are the firmware's to accept. `handle_move_joint_to` NACKs
    OUT_OF_RANGE against `kJointLimits[]` and moves nothing, so `move_to()`'s
    verdict is checked rather than assumed. That table is live: the neck's tilt
    window is currently opened to the servo's full travel for calibration and is
    meant to narrow again once the mechanical stops are characterised, at which
    point a tilt that works today starts being refused.
    """
    print(f"--- aiming the neck via the motion board on {args.motion_port} ---")
    try:
        link = Motion(args.motion_port).connect()
    except Exception as exc:
        raise RuntimeError(
            f"could not open the motion board on {args.motion_port}: {exc}\n"
            "  The neck is on the MOTION board, not the drivetrain — check\n"
            "  `ls -l /dev/orio_* /dev/ttyACM*`. If /dev/orio_motion is missing,\n"
            "  the udev rule is not installed on this machine (see\n"
            "  99-orio-stm32.rules); find the motion board in\n"
            "  `ls /dev/serial/by-id/` by its ST-LINK serial and pass it with\n"
            "  --motion-port. Do not guess between ttyACM0 and ttyACM1: the wrong\n"
            "  one is the drivetrain, and it takes a joint angle as throttle.\n"
            "  To drive without touching the head at all, pass --no-neck."
        ) from exc

    print(f"    identity confirmed on the wire: {link.identity}")
    try:
        rejection = link.move_to(JOINT_NECK, args.neck_pan, args.neck_tilt)
        if rejection is not None:
            raise RuntimeError(
                f"the motion board refused the neck pose "
                f"(pan {args.neck_pan:g}°, tilt {args.neck_tilt:g}°): {rejection.reason}\n"
                "  OUT_OF_RANGE means the angle is outside kJointLimits[] in\n"
                "  orio-stm-motion/Core/Src/servo_joint.c and NOTHING moved. That\n"
                "  window is still being characterised — the neck's tilt is opened\n"
                "  to full travel temporarily and narrows again once the stops are\n"
                "  known — so check the firmware's limits rather than this script's\n"
                "  defaults, and override with --neck-pan / --neck-tilt."
            )

        # Confirm the pose off the board rather than trusting the ACK: the ACK
        # says the angles were accepted and applied to the PWM, and reading them
        # back is the only thing that also proves the position latched.
        status = link.read_status()
        if status is None:
            print("    (no STATUS came back — pose commanded but unconfirmed)")
        else:
            angles = status.joints.get("neck")
            print(f"    neck holding at {angles}" if angles else "    neck pose unreported")
    except Exception:
        link.close()
        raise
    return link


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

    # The neck comes first, before the cameras it aims and before the wheels:
    # the avoider's thresholds are all distances measured through this head, so
    # a run that started with the head somewhere else would be steering on a
    # view of somewhere else. Failing to place it is therefore fatal rather than
    # a warning — use --no-neck to drive without it deliberately.
    neck = None
    if not args.no_neck:
        try:
            neck = aim_neck(args)
        except Exception as exc:
            print(f"\n{exc}")
            return 1

    try:
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

            # connect() has already refused the link if this is the wrong board;
            # printing what answered makes a swapped board visible in the log of a
            # session that DID start, not just in the failure path above.
            print(f"    identity confirmed on the wire: {link.identity}")

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
    finally:
        if neck is not None:
            # Releases the servos: the firmware has no way to hold a pose with
            # the link down, so the head goes slack here (and would anyway,
            # 500 ms after the last heartbeat).
            print("--- releasing the neck ---")
            neck.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
