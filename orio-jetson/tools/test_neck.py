#!/usr/bin/env python3
"""Exercise the neck joint through a fixed sequence of moves and check each one.

    uv run python tools/test_neck.py                    # /dev/orio_motion
    uv run python tools/test_neck.py --dry-run          # print the plan, move nothing
    uv run python tools/test_neck.py --port /dev/ttyACM0
    uv run python tools/test_neck.py --pan-span 30 --tilt-span 8
    uv run python tools/test_neck.py --hold 5           # sit on each pose to look at it

Run it after flashing `orio-stm-motion` to confirm the neck still moves the way
the Jetson expects: both axes, both directions, and both together. It talks
through `orio/motion.py` — the same link the app uses — so it also exercises the
udev name, the CMD_WHOAMI identity handshake and the heartbeat hold, not just
the servos.

## What this verifies, and what it cannot

Each step commands a pose, waits out `motion.travel_time_s()` plus a settle, and
reads `CMD_GET_STATUS` back. A step passes when the board reports the angles that
were asked for.

That is a check of the FIRMWARE'S OWN IDEA of where the joint is —
`current_pan_cdeg` / `current_tilt_cdeg`, the value it last converted to a pulse
width. These servos give the STM32 no position feedback, so nothing on the wire
can tell you the horn actually turned. A servo that is unpowered, stalled against
a stop, or has a stripped horn reports exactly the same angles as one doing its
job.

**So the physical half of this test is you watching the head.** The status check
catches a link that dropped, a NACK, and a pose that never got applied; it cannot
catch a joint that did not move. Use --hold to give yourself time to look.

## Why the safe band is enforced here rather than by the firmware

Normally range is the firmware's business: `handle_move_joint_to` validates
against `kJointLimits[]` and answers NACK_OUT_OF_RANGE without moving anything,
which is why `motion.py` deliberately keeps no copy of those limits.

The neck is the exception, right now. Its row in `kJointLimits[]` is opened to
the servos' full travel (pan 0..270, tilt 0..180) and marked TEMPORARY, for
calibration — so the firmware currently accepts *every* neck angle and protects
nothing. Meanwhile `config.py` records what was measured on the robot on
2026-09-09: from tilt 50 onward the joint is against a mechanical stop, and
commanding further just stalls the servo, which then heats and draws
locked-rotor current until something gives.

A test that walked the neck to the edge of the window the firmware advertises
would therefore drive it into that stop, and get an ACK for doing it. Hence the
guard below, and hence --allow-unsafe to defeat it deliberately rather than by
accident. Once the stops are characterised and written back into
`kJointLimits[]`, the firmware becomes the authority again and this guard should
shrink to nothing — delete it then rather than letting two tables disagree.

**This script never sends CMD_RESET_JOINTS.** Home is the midpoint of the
commandable window, so while that window is the full travel the neck's home is
tilt 90 — a good 40 deg past the measured stop. Homing the neck is not safe
until the real limits are in the firmware.

## The head goes slack at the end

The motion board's e-stop cuts the servo PWM rather than freezing it, and the
board e-stops itself 500 ms after the last heartbeat. Closing the link therefore
always releases the joints: the head will sag under its own weight when this
exits, however it exits. The last step returns to the nominal pose so it starts
that fall from somewhere sensible — support the head if the drop matters.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.motion import JOINT_NECK, IdentityError, Motion, travel_time_s

# The motion board by its stable udev name — never a raw /dev/ttyACM*, whose
# number is enumeration order and can point at the drivetrain instead. The
# identity handshake in Motion.connect() is what actually makes that safe; see
# config.MOTION_PORT and orio/motion.py.
DEFAULT_PORT = config.MOTION_PORT

# The pose the robot actually runs at, and the centre this test works around.
# Pan 175 is a few degrees off the neck's 180 home so the cameras point where
# the chassis does; tilt 40 is the middle of the usable window measured on
# 2026-09-09 (see the long note in config.py).
NOMINAL_PAN_DEG = config.NECK_PAN_DEG
NOMINAL_TILT_DEG = config.NECK_TILT_DEG

# Guard band — the angles this script will command without --allow-unsafe.
#
# TILT_GUARD_MAX is the load-bearing one and it is a measurement, not a margin:
# at tilt 50 the joint is already against its mechanical stop (config.py,
# 2026-09-09 — 50 through 80 all read the same distance because the head stopped
# moving at 50). Staying below it is the difference between a test and a stall.
#
# The other three are conservative rather than measured. Nothing has swept the
# neck's pan stops or its downward tilt stop yet, so they bound the test to
# angles the robot is known to have held: pan 175 and 180 in normal operation,
# tilt 30 as the previous default. Widen them from a sweep_axis.py measurement,
# not from a guess.
PAN_GUARD_MIN, PAN_GUARD_MAX = 150.0, 210.0
TILT_GUARD_MIN, TILT_GUARD_MAX = 28.0, 48.0

# How far each axis moves either side of nominal by default. Kept well inside
# the guard so the default run has room, and so widening a span is a decision
# rather than something that silently clips.
DEFAULT_PAN_SPAN_DEG = 20.0
# 5 deg either side of nominal keeps the default run inside 35..45 — exactly the
# usable window measured on 2026-09-09, and a comfortable 5 deg clear of the
# stop at 50 rather than creeping up on it.
DEFAULT_TILT_SPAN_DEG = 5.0

# Added to the predicted travel before reading the pose back. travel_time_s()
# runs up to ~33 ms short of the firmware's real travel (it integrates the ramp
# in integer hundredths of a degree and loses the remainder), and at the 120
# deg/s cruise that is a few degrees still to go. This covers it with room over.
DEFAULT_SETTLE_S = 0.35

# How far the reported pose may sit from the commanded one and still pass. The
# firmware arrives exactly on target once the profile finishes, so this is
# slack for a status frame caught mid-step, not for a sloppy servo.
ANGLE_TOLERANCE_DEG = 1.0


@dataclass(frozen=True)
class Step:
    """One commanded pose and what it is meant to demonstrate."""

    label: str
    pan_deg: float
    tilt_deg: float


def build_plan(pan: float, tilt: float, pan_span: float, tilt_span: float) -> list[Step]:
    """The move sequence: each axis alone in both directions, then both at once.

    Every step returns to the nominal pose before the next axis is tried, so a
    failure names one axis travelling in one direction rather than some
    accumulated pose nobody commanded. The diagonal at the end is the one step
    that moves both together — the firmware runs each axis along its own profile
    and they finish independently, so a joint that is fine one axis at a time can
    still be wrong here.
    """
    centre = Step("centre", pan, tilt)
    return [
        centre,
        Step("pan left", pan - pan_span, tilt),
        centre,
        Step("pan right", pan + pan_span, tilt),
        centre,
        # Lower tilt looks UP: tilt 30 frames the ceiling, tilt 45 the floor
        # ahead. Worth naming, because the number moving down while the head
        # moves up reads backwards on the console otherwise.
        Step("tilt up", pan, tilt - tilt_span),
        centre,
        Step("tilt down", pan, tilt + tilt_span),
        centre,
        Step("diagonal", pan - pan_span, tilt + tilt_span),
        Step("return to nominal", pan, tilt),
    ]


def guard_violations(plan: list[Step]) -> list[str]:
    """Which steps leave the guard band, described well enough to act on."""
    problems = []
    for step in plan:
        if not PAN_GUARD_MIN <= step.pan_deg <= PAN_GUARD_MAX:
            problems.append(
                f"{step.label}: pan {step.pan_deg:g}° is outside the "
                f"{PAN_GUARD_MIN:g}..{PAN_GUARD_MAX:g}° guard"
            )
        if not TILT_GUARD_MIN <= step.tilt_deg <= TILT_GUARD_MAX:
            problems.append(
                f"{step.label}: tilt {step.tilt_deg:g}° is outside the "
                f"{TILT_GUARD_MIN:g}..{TILT_GUARD_MAX:g}° guard"
                + (
                    " — the joint is against its mechanical stop from 50° and stalls there"
                    if step.tilt_deg >= 50.0
                    else ""
                )
            )
    return problems


def run_step(link: Motion, step: Step, previous: Step, settle_s: float, hold_s: float) -> bool:
    """Command one pose, wait it out, and check what the board reports back."""
    pan_delta = abs(step.pan_deg - previous.pan_deg)
    tilt_delta = abs(step.tilt_deg - previous.tilt_deg)
    # Both axes travel at once, so the move costs the longer of the two.
    expected_s = max(travel_time_s(pan_delta), travel_time_s(tilt_delta))

    print(
        f"  {step.label:<18} → pan {step.pan_deg:7.2f}°  tilt {step.tilt_deg:6.2f}°"
        f"   (~{expected_s:.2f}s)",
        flush=True,
    )

    rejection = link.move_to(JOINT_NECK, step.pan_deg, step.tilt_deg)
    if rejection is not None:
        print(f"      FAIL  board refused the move: {rejection.reason}")
        return False

    time.sleep(expected_s + settle_s)

    status = link.read_status()
    if status is None:
        print("      FAIL  no status frame came back — the link may have dropped")
        return False
    if status.estopped:
        print("      FAIL  the board is e-stopped; the servos are released")
        return False

    angles = status.joints.get("neck")
    if angles is None:
        print("      FAIL  the status frame carried no neck angles")
        return False

    pan_error = abs(angles.pan_deg - step.pan_deg)
    tilt_error = abs(angles.tilt_deg - step.tilt_deg)
    if pan_error > ANGLE_TOLERANCE_DEG or tilt_error > ANGLE_TOLERANCE_DEG:
        print(
            f"      FAIL  board reports {angles} — off by "
            f"pan {pan_error:.2f}°, tilt {tilt_error:.2f}°"
        )
        return False

    print(f"      ok    board reports {angles}")
    if hold_s > 0:
        time.sleep(hold_s)
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="motion board serial port")
    parser.add_argument("--pan", type=float, default=NOMINAL_PAN_DEG, help="nominal pan angle")
    parser.add_argument("--tilt", type=float, default=NOMINAL_TILT_DEG, help="nominal tilt angle")
    parser.add_argument(
        "--pan-span", type=float, default=DEFAULT_PAN_SPAN_DEG, help="pan travel either side"
    )
    parser.add_argument(
        "--tilt-span", type=float, default=DEFAULT_TILT_SPAN_DEG, help="tilt travel either side"
    )
    parser.add_argument(
        "--settle", type=float, default=DEFAULT_SETTLE_S, help="extra wait before reading the pose"
    )
    parser.add_argument(
        "--hold", type=float, default=0.0, help="seconds to sit on each pose so you can watch it"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print the plan and guard check, move nothing"
    )
    parser.add_argument(
        "--allow-unsafe",
        action="store_true",
        help="command angles outside the guard band (see the module docstring — "
        "tilt 50+ stalls the servo against its stop)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    plan = build_plan(args.pan, args.tilt, args.pan_span, args.tilt_span)

    print(f"neck test plan — {len(plan)} steps, nominal pan {args.pan:g}°, tilt {args.tilt:g}°")

    problems = guard_violations(plan)
    if problems:
        for problem in problems:
            print(f"  ! {problem}")
        if not args.allow_unsafe:
            print(
                "\nRefusing to run. The firmware's neck window is temporarily opened to\n"
                "full travel, so it would ACK these angles and drive the joint into its\n"
                "stop. Narrow the spans, or pass --allow-unsafe if you know the head is\n"
                "unloaded and you are watching it."
            )
            return 2
        print("\n--allow-unsafe given: running anyway. Keep a hand on the power.")

    if args.dry_run:
        print()
        previous = Step("start", args.pan, args.tilt)
        total_s = 0.0
        for step in plan:
            expected_s = max(
                travel_time_s(abs(step.pan_deg - previous.pan_deg)),
                travel_time_s(abs(step.tilt_deg - previous.tilt_deg)),
            )
            total_s += expected_s + args.settle + args.hold
            print(
                f"  {step.label:<18} → pan {step.pan_deg:7.2f}°  tilt {step.tilt_deg:6.2f}°"
                f"   (~{expected_s:.2f}s)"
            )
            previous = step
        print(f"\ndry run — nothing was commanded. A real run takes about {total_s:.0f}s.")
        return 0

    print(f"\nopening {args.port}")
    try:
        link = Motion(args.port).connect()
    except IdentityError as exc:
        # Fatal by design: every frame this sends is also a well-formed frame on
        # the drivetrain board, where it would decode as wheel throttle.
        print(f"FAIL  {exc}")
        return 1
    except Exception as exc:
        print(f"FAIL  could not open {args.port}: {exc}")
        return 1

    print(f"      {link.identity}")
    print("\nwatch the head — the status check cannot see whether it actually moved\n")

    passed = 0
    previous = Step("start", args.pan, args.tilt)
    try:
        with link:
            for step in plan:
                if run_step(link, step, previous, args.settle, args.hold):
                    passed += 1
                previous = step
    except KeyboardInterrupt:
        print("\ninterrupted — the link is closing and the head will go slack")
        return 130
    finally:
        print("\nlink closed: the servos are released and the head will sag.")

    print(f"\n{passed}/{len(plan)} steps passed")
    return 0 if passed == len(plan) else 1


if __name__ == "__main__":
    sys.exit(main())
