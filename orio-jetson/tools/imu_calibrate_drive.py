#!/usr/bin/env python3
"""Calibrate the IMU: the robot drives the parts it can, you tilt the rest.

Four short manoeuvres under its own power settle the forward axis and the yaw
sign. Then it asks for three shim poses, because a robot on a flat floor cannot
tilt itself far enough to measure body tilt at all.

    uv run python tools/imu_calibrate_drive.py
    uv run python tools/imu_calibrate_drive.py --duty 5 --csv /tmp/imudrive.csv

Have a shim ready before starting (a book or a plank, 20-40 mm) and something
to chock the wheels with. `tools/imu_calibrate.py` is the same calibration with
no driving at all, for when there is no space to move.

## Read this before running it: the robot drives BLIND

This is a bench tool. It talks to the drivetrain directly, so **the avoidance
policy is not in the loop** — no cameras, no stereo, no `Avoider`, nothing
watching where it is going. That is the opposite of `tools/teleop_guarded.py`
and of every path through `body.py`, and it is only acceptable because the
manoeuvres are ~1 s each at 5% duty (~0.3 m/s, so ~0.3 m of travel) and a human
is standing over it.

Clear **2 m ahead, 1 m behind and 1 m either side**. Hand near the power
switch. Ctrl-C stops the wheels and e-stops the board on the way out, and so
does any exception — that is `Drivetrain.__exit__`'s guarantee.

## Why the split

  * **Forward axis and its sign — driven.** Accelerating forward, then in
    reverse, moves one accelerometer channel one way and then the other.
    Differencing the two cancels gravity and any fixed bias.
  * **Yaw sign — driven.** Pivot left, pivot right, compare. This is the one
    the policy needs most: measured turns and heading hold both rest on it.
  * **Nose up/down, and side-lean — by hand, on a shim.** Driving on a flat
    floor produces a few tenths of a degree of body rock, tangled up with the
    castor re-pointing. A 30 mm shim produces several degrees of unambiguous
    tilt. The weaker measurement is not worth having when the stronger one
    costs thirty seconds.

    The lean takes BOTH wheels, one after the other. On a three-point chassis
    a shim under one wheel tilts the body diagonally — measured here as +4.1
    deg of nose against +1.7 deg of lean — so left and right are subtracted to
    cancel the nose component.

The rest-pose offsets come from standing still, before anything moves.
"""

from __future__ import annotations

import argparse
import csv
import statistics as st
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import imu_cal  # noqa: E402  (tools/ is on the path: these run as scripts)
from imu_cal import (  # noqa: E402
    Collector, Window, hand_step, measure, print_settings, roll_result, tilt_result,
    yaw_result,
)
from orio import config  # noqa: E402
from orio.drivetrain import DRIVE_REFRESH_S, Drivetrain  # noqa: E402
from orio.imu import RvcReader  # noqa: E402

BURST_S = 1.0
PAUSE_S = 1.5
MIN_ACCEL_MG = 8.0  # below this an accel difference is noise


def preflight(link: Drivetrain) -> bool:
    """Refuse to start if the ESCs are not powered.

    A drive command into unpowered ESCs moves nothing and reports nothing, and
    the run comes back full of UNKNOWNs. The telemetry says so up front: both
    wheels invalid with 0 V in is the drive battery being off, which is exactly
    how this robot sits on the bench.
    """
    deadline = time.monotonic() + 2.0
    status = None
    while time.monotonic() < deadline:
        link.request_status()
        time.sleep(0.2)
        status = link.last_status
        if status is not None and any(w.valid for w in status.wheels.values()):
            break
    if status is None:
        print("\nNo telemetry from the drivetrain board — not driving.")
        return False
    valid = [side for side, w in status.wheels.items() if w.valid]
    if not valid:
        print("\nBoth wheels report INVALID telemetry (v_in 0 V): the drive battery")
        print("looks switched off. Turn the motor power on and run this again —")
        print("driving now would move nothing and measure nothing.")
        return False
    if len(valid) < len(status.wheels):
        missing = [s for s in status.wheels if s not in valid]
        print(f"\nOnly the {'/'.join(valid)} wheel is reporting; {'/'.join(missing)} is not.")
        print("Fix that first — a pivot needs both wheels to mean anything.")
        return False
    print(f"both wheels reporting, {st.mean(w.v_in for w in status.wheels.values()):.1f} V in")
    return True


def manoeuvre(link: Drivetrain, collector: Collector, label: str,
              left: int, right: int, seconds: float) -> Window:
    """Hold one drive command for `seconds`, recording the IMU throughout.

    The wheels are zeroed in a `finally`, so an exception or a Ctrl-C here
    still leaves the robot stopped rather than driving on.
    """
    print(f"  {label}: left {left:+d}, right {right:+d} per-mille for {seconds:g}s ...")
    collector.start()
    started = time.monotonic()
    sent_at = 0.0
    try:
        while time.monotonic() - started < seconds:
            now = time.monotonic()
            # Re-send: the firmware drops a command it never got, and an
            # un-refreshed setpoint is a stopped wheel. Drive first, always.
            if now - sent_at >= DRIVE_REFRESH_S:
                link.set_drive(left, right)
                sent_at = now
            time.sleep(0.02)
    finally:
        link.set_drive(0, 0)
    window = collector.stop()
    print(f"    turned {window.yaw_span:+.1f} deg;  ax {window.mean('ax_mg'):+.0f}  "
          f"ay {window.mean('ay_mg'):+.0f} mg   ({len(window.samples)} readings)")
    time.sleep(PAUSE_S)
    return window


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=config.DRIVETRAIN_PORT,
                    help="drivetrain board (the stable name, not ttyACM*)")
    ap.add_argument("--imu-port", default="/dev/ttyTHS1")
    ap.add_argument("--duty", type=float, default=config.DRIVE_SPEED_MAX_PERCENT,
                    help="straight-line duty PERCENT, capped at the robot's ceiling")
    ap.add_argument("--pivot-duty", type=float, default=config.AVOID_ESCAPE_DUTY / 10.0,
                    help="duty PERCENT for the pivots (the escape duty, which is what "
                         "actually turns this chassis)")
    ap.add_argument("--seconds", type=float, default=BURST_S, help="length of each manoeuvre")
    ap.add_argument("--shim-mm", type=float, default=None, help="shim thickness, for the log only")
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--yes", action="store_true", help="skip the are-you-clear prompt")
    args = ap.parse_args()

    cap = max(config.DRIVE_SPEED_MAX_PERCENT, config.AVOID_ESCAPE_DUTY / 10.0)
    duty = round(max(-cap, min(cap, args.duty)) * 10)
    pivot = round(max(-cap, min(cap, args.pivot_duty)) * 10)
    seconds = min(args.seconds, 2.0)

    print("=" * 78)
    print("IMU CALIBRATION")
    print("=" * 78)
    print(f"""
Part 1 — the robot drives itself: forward, back, pivot left, pivot right. Each
lasts {seconds:g}s at {duty / 10:g}% duty (pivots {pivot / 10:g}%), with a stop between.
Total travel is well under a metre.

  IT DRIVES BLIND. The cameras and the avoidance policy are NOT in this loop.
  Clear 2 m ahead, 1 m behind, 1 m either side. Stand over it, hand near the
  power switch. Ctrl-C stops the wheels and e-stops the board.

Part 2 — you tilt it three times on a shim, because a robot on a flat floor
cannot tilt itself: castor up, then left wheel up, then right wheel up. Both
wheels are needed because this chassis stands on three points, so one wheel on
a shim tilts it diagonally; the two sides subtract to leave pure lean.
Have ready: a shim 20-40 mm thick (a book), and wheel chocks.""")
    print(imu_cal.DIAGRAM)

    if not args.yes:
        try:
            if input("Space clear, shim to hand? Type yes to drive: ").strip().lower() not in ("y", "yes"):
                print("stopped — nothing was driven")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\nstopped — nothing was driven")
            return 1

    collector = Collector()
    with RvcReader(port=args.imu_port, on_reading=collector) as imu:
        time.sleep(0.5)
        if imu.fresh(max_age_s=1.0) is None:
            print(f"No data from the IMU on {args.imu_port} — nothing will be driven.")
            print("Check P0 to 3Vo, P1 to GND, BT to 3Vo, SDA to Jetson pin 10, then power-cycle.")
            return 1

        try:
            with Drivetrain(args.port).connect() as link:
                print(f"\ndrivetrain on {args.port}; IMU on {args.imu_port}")
                if not preflight(link):
                    return 1

                print("\n" + "-" * 78)
                print("PART 1 of 2: the robot drives (4 moves, hands off)")
                print("-" * 78)
                print("  rest (the zero): stand clear, nothing touching it")
                rest = measure(collector, imu_cal.REST_S)
                fwd = manoeuvre(link, collector, "forward", duty, duty, seconds)
                rev = manoeuvre(link, collector, "reverse", -duty, -duty, seconds)
                piv_l = manoeuvre(link, collector, "pivot left", -pivot, pivot, seconds)
                piv_r = manoeuvre(link, collector, "pivot right", pivot, -pivot, seconds)
        except KeyboardInterrupt:
            print("\nstopped — wheels zeroed and board e-stopped")
            return 1
        except OSError as exc:
            print(f"\ncould not open the drivetrain on {args.port}: {exc}")
            print("The two STM32 boards have stable names (/dev/orio_drive, /dev/orio_motion);")
            print("pass --port explicitly rather than guessing between ttyACM0 and ttyACM1.")
            return 1

        print("\n" + "=" * 78)
        print("PART 2 of 2: three tilts, by hand — the wheels are stopped now")
        print("=" * 78)
        try:
            nose_down = hand_step(imu, collector, seconds=imu_cal.SAMPLE_S, **imu_cal.NOSE_DOWN_STEP)
            left_up = hand_step(imu, collector, seconds=imu_cal.SAMPLE_S, **imu_cal.LEFT_UP_STEP)
            right_up = hand_step(imu, collector, seconds=imu_cal.SAMPLE_S, **imu_cal.RIGHT_UP_STEP)
            rest2 = hand_step(
                imu, collector, seconds=imu_cal.SAMPLE_S,
                title="Last one: flat on the floor again",
                instructions=["Take the shim away and put the robot flat.",
                              "Nothing under any wheel, nothing touching it."],
                finds="whether the resting position is repeatable",
                expect="should read close to the resting numbers from part 1",
            )
        except (EOFError, KeyboardInterrupt):
            print("\nstopped — the driven part is done, but the tilts are not")
            return 1

    return report(args, rest, fwd, rev, piv_l, piv_r, nose_down, left_up, right_up,
                  rest2, collector)


def report(args, rest, fwd, rev, piv_l, piv_r, nose_down, left_up, right_up,
           rest2, collector) -> int:
    print("\n" + "=" * 78)
    print("RESULTS")
    print("=" * 78)

    pitch0, roll0 = rest.mean("pitch_deg"), rest.mean("roll_deg")
    print("\n1. HOW THE ROBOT SITS AT REST")
    print(f"   Front-back: {pitch0:+.2f} deg ({'nose down' if pitch0 < 0 else 'nose up'})")
    print(f"   Side-lean:  {roll0:+.2f} deg")
    print("   This is what the code will treat as 'level', because every distance")
    print("   the robot drives by was measured with it sitting like this.")

    # Forward vs reverse: the difference cancels gravity and any fixed bias.
    d_ax = (fwd.mean("ax_mg") - rev.mean("ax_mg")) / 2
    d_ay = (fwd.mean("ay_mg") - rev.mean("ay_mg")) / 2
    fwd_axis, fwd_delta = ("ax", d_ax) if abs(d_ax) >= abs(d_ay) else ("ay", d_ay)
    print("\n2. WHICH WAY IS FORWARD (driven)")
    print(f"   Forward vs reverse: ax {d_ax:+.0f} mg, ay {d_ay:+.0f} mg")
    if abs(fwd_delta) >= MIN_ACCEL_MG:
        print(f"   -> forward shows on {fwd_axis.upper()}, "
              f"{'positive' if fwd_delta > 0 else 'negative'} when accelerating forward")
        print("   " + ("(+Y forward, as the silkscreen says)" if fwd_axis == "ay"
                       else "(NOT the Y axis — worth checking how the board is mounted)"))
    else:
        print("   COULD NOT TELL: too little acceleration to separate. Try --seconds 1.5,")
        print("                   or a surface with more grip.")

    yaw_l, yaw_r = piv_l.yaw_span, piv_r.yaw_span
    yaw_sign, yaw_ok = yaw_result(yaw_l, yaw_r)
    print("\n3. WHICH WAY IS A LEFT TURN (driven)")
    print(f"   Pivot left turned {yaw_l:+.1f} deg;  pivot right turned {yaw_r:+.1f} deg")
    if yaw_ok:
        print(f"   -> the sensor calls a left turn {'positive' if yaw_sign > 0 else 'negative'}; "
              "the setting below makes it positive")
    else:
        print("   COULD NOT TELL: the pivots did not turn far enough, or both went the")
        print("                   same way. Raise --pivot-duty or --seconds and re-run.")

    print("\n4. WHICH NUMBER MEANS WHAT (tilted by hand)")
    pitch_axis, pitch_sign, pitch_ok = tilt_result(rest, nose_down, "Front tilted down:", nose=True)
    roll_axis, roll_sign, roll_ok = roll_result(left_up, right_up)
    if pitch_ok and roll_ok and pitch_axis == roll_axis:
        pitch_ok = roll_ok = False
        print("\n   PROBLEM: both tilts moved the same number, so the two shim positions")
        print("            cannot be told apart. Re-run, shimming the CASTOR for the")
        print("            first tilt and the WHEELS for the other two.")

    dp = rest2.mean("pitch_deg") - pitch0
    dr = rest2.mean("roll_deg") - roll0
    print("\n5. DID IT COME BACK TO THE SAME PLACE?")
    print(f"   Since the start: front-back moved {dp:+.2f} deg, side-lean moved {dr:+.2f} deg")
    if max(abs(dp), abs(dr)) > 0.8:
        print("   That is more than the castor alone explains — check the IMU bracket")
        print("   is tight, and that nothing is left under a wheel.")
    else:
        print("   Fine — that is the castor re-pointing, which is real body tilt.")

    print_settings(pitch0, roll0, pitch_sign, pitch_ok, roll_sign, roll_ok, yaw_sign, yaw_ok)
    if args.shim_mm:
        print(f"(shim used: {args.shim_mm:g} mm)")

    if args.csv:
        with args.csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "index", "heading", "pitch", "roll", "ax_mg", "ay_mg", "az_mg"])
            t0 = collector.all[0].timestamp if collector.all else 0.0
            for r in collector.all:
                w.writerow([f"{r.timestamp - t0:.3f}", r.index, f"{r.heading_deg:.2f}",
                            f"{r.pitch_deg:.2f}", f"{r.roll_deg:.2f}",
                            f"{r.ax_mg:.0f}", f"{r.ay_mg:.0f}", f"{r.az_mg:.0f}"])
        print(f"\nevery reading -> {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
