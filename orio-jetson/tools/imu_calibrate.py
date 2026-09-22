#!/usr/bin/env python3
"""Calibrate the IMU without driving: every step done by hand.

The IMU reports angles in its own frame. What the code needs is the BODY's
attitude, and between the two sit a mount that is never perfectly square, a
chassis with its own lean, and axis names that do not mean what the datasheet
calls them once the part is mounted with +Y forward. This measures all three
and prints the settings to save.

    uv run python tools/imu_calibrate.py
    uv run python tools/imu_calibrate.py --shim-mm 30 --csv /tmp/imucal.csv

**Nothing here ever commands a motor.** Use `tools/imu_calibrate_drive.py`
instead when there is floor space: it drives the forward and turn steps itself
and only asks for the two tilts, which is less to get wrong by hand. This tool
is the fallback for a bench, a tight room, or a robot with its drive battery
off.

## The robot is heavy — do not lift it

Every tilt is made by rolling the robot onto a shim (a book, a plank: 20-40
mm), never by picking an end up, and the castor end goes first because it is
the light one. Chock the wheels before shimming: a robot this heavy that starts
rolling does not get caught by hand.

## The five poses

  1. **Rest, castor trailing** — the calibration zero. Every threshold in
     `avoid.py` was measured with the body in this pose, so "level" for this
     robot means "as it sits here", not "as a spirit level sees it". Roll it
     forward half a metre first so the castor trails: where it points changes
     body pitch by ~0.6 deg, which is real tilt and not noise.
  2. **Shim under the castor** — nose down. Which angle is body pitch, and
     which way is negative.
  3. **Shim under the left drive wheel** — left side up. Which angle is body
     roll, and its sign.
  4. **A quarter turn to the left, pushed by hand** — the sign of yaw, so that
     "+30 deg" means a left turn to the policy.
  5. **Rest again** — repeatability. A disagreement with step 1 of more than a
     few tenths of a degree means something moved: the bracket, the castor, or
     a shim still under a wheel.

Steps 2-4 only need the SIGN, so 20-40 mm of shim is plenty. The magnitudes are
a cross-check, not a target.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import imu_cal  # noqa: E402  (tools/ is on the path: these run as scripts)
from imu_cal import (  # noqa: E402
    Collector, hand_step, print_settings, tilt_result, yaw_result,
)
from orio.imu import RvcReader  # noqa: E402

REST_STEP = dict(
    title="Robot flat on the floor, straight",
    instructions=[
        "Put the robot on flat, level floor.",
        "Push it FORWARD about half a metre and let it stop on its own.",
        "  (that leaves the castor trailing behind, the way it sits after driving)",
        "Take your hands off it completely. Do not lean on it.",
    ],
    finds="the resting position — everything else is measured against this",
    expect="nothing should move; the numbers should sit still",
)

TURN_STEP = dict(
    title="Turn the whole robot a quarter turn to the LEFT",
    instructions=[
        "Robot flat on the floor, nothing under any wheel.",
        "Push the body round on the spot, to the LEFT, about a quarter turn (90 deg).",
        "  Left = anticlockwise seen from above. If it faced the window, it should",
        "  now face whatever was to the left of the window.",
        "It does not have to be exact — anything over 30 deg works.",
        "Hands off.",
    ],
    finds="which direction of turn the sensor calls positive",
    expect="the heading number should change by roughly the angle you turn it",
)

REST2_STEP = dict(
    title="Leave it alone, flat on the floor",
    instructions=[
        "Do not move the robot at all for this step.",
        "Just make sure nothing is under any wheel and nothing is touching it.",
    ],
    finds="whether the resting position is repeatable",
    expect="should read close to step 1 (within a few tenths of a degree)",
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--seconds", type=float, default=imu_cal.SAMPLE_S,
                    help="measuring window per step")
    ap.add_argument("--shim-mm", type=float, default=None, help="shim thickness, for the log only")
    ap.add_argument("--csv", type=Path, default=None, help="write every reading here")
    args = ap.parse_args()

    print("=" * 78)
    print("IMU MOUNT CALIBRATION (no driving)")
    print("=" * 78)
    print("""
This works out how the IMU board sits inside the robot, so the code can tell
the BODY's tilt from the sensor's own tilt.

There are 5 steps. Each one: put the robot in a position, press Enter, keep
your hands off while it measures. Then it prints the settings to save.

THE ROBOT IS NEVER LIFTED AND NEVER DRIVEN. Tilts are made by rolling it onto
a shim -- a book or a plank, 20-40 mm thick. Chock the wheels first.

You need: a shim (20-40 mm), wheel chocks, flat floor. Ctrl-C to stop.""")
    print(imu_cal.DIAGRAM)

    collector = Collector()
    with RvcReader(port=args.port, on_reading=collector) as reader:
        time.sleep(0.5)
        if reader.fresh(max_age_s=1.0) is None:
            print(f"No data from the IMU on {args.port}.")
            print("Check: P0 to 3Vo, P1 to GND, BT to 3Vo, SDA to Jetson pin 10,")
            print("then power-cycle the board (it only reads those pins at power-up).")
            return 1

        steps = [
            ("rest", REST_STEP),
            ("nose_down", imu_cal.NOSE_DOWN_STEP),
            ("left_up", imu_cal.LEFT_UP_STEP),
            ("turn_left", TURN_STEP),
            ("rest2", REST2_STEP),
        ]
        windows = {}
        try:
            for n, (key, step) in enumerate(steps, 1):
                titled = dict(step, title=f"STEP {n} of {len(steps)}: {step['title']}")
                windows[key] = hand_step(reader, collector, seconds=args.seconds, **titled)
        except (EOFError, KeyboardInterrupt):
            print("\nstopped")
            return 1

        if any(not w.samples for w in windows.values()):
            print("\na step captured nothing — nothing to report")
            return 1

        rest = windows["rest"]
        pitch0, roll0 = rest.mean("pitch_deg"), rest.mean("roll_deg")

        print("\n" + "=" * 78)
        print("RESULTS")
        print("=" * 78)
        print("\n1. HOW THE ROBOT SITS AT REST")
        print(f"   Front-back: {pitch0:+.2f} deg ({'nose down' if pitch0 < 0 else 'nose up'})")
        print(f"   Side-lean:  {roll0:+.2f} deg")
        print("   This is what the code will treat as 'level', because every distance")
        print("   the robot drives by was measured with it sitting like this.")

        print("\n2. WHICH NUMBER MEANS WHAT")
        pitch_axis, pitch_sign, pitch_ok = tilt_result(
            rest, windows["nose_down"], "Front tilted down:", nose=True)
        roll_axis, roll_sign, roll_ok = tilt_result(
            rest, windows["left_up"], "Left side tilted up:", nose=False)
        if pitch_ok and roll_ok and pitch_axis == roll_axis:
            pitch_ok = roll_ok = False
            print("\n   PROBLEM: both tilts moved the same number, so the two shim positions")
            print("            cannot be told apart. Re-run, shimming the CASTOR in step 2")
            print("            and the LEFT WHEEL in step 3.")

        turned = windows["turn_left"].yaw_span
        yaw_sign, yaw_ok = yaw_result(turned)
        print("\n3. WHICH WAY IS A LEFT TURN")
        print(f"   The quarter turn moved the heading {turned:+.1f} deg")
        if yaw_ok:
            print(f"   -> the sensor calls a left turn {'positive' if yaw_sign > 0 else 'negative'}; "
                  "the setting below makes it positive")
        else:
            print("   COULD NOT TELL: less than 30 deg of turn was measured. Push the robot")
            print("                   further round and run this step again.")

        dp = windows["rest2"].mean("pitch_deg") - pitch0
        dr = windows["rest2"].mean("roll_deg") - roll0
        print("\n4. DOES IT COME BACK TO THE SAME PLACE?")
        print(f"   Between the first and last steps: front-back moved {dp:+.2f} deg, "
              f"side-lean moved {dr:+.2f} deg")
        if max(abs(dp), abs(dr)) > 0.5:
            print("   PROBLEM: more than expected. Either the castor is pointing a different")
            print("            way (normal, up to ~0.6 deg), or the IMU bracket is loose.")
        else:
            print("   Good — that is within the expected range.")

        print_settings(pitch0, roll0, pitch_sign, pitch_ok, roll_sign, roll_ok, yaw_sign, yaw_ok)
        if args.shim_mm:
            print(f"(shim used: {args.shim_mm:g} mm)")

        if args.csv:
            with args.csv.open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["step", "index", "heading", "pitch", "roll", "ax_mg", "ay_mg", "az_mg"])
                for key, window in windows.items():
                    for r in window.samples:
                        w.writerow([key, r.index, f"{r.heading_deg:.2f}",
                                    f"{r.pitch_deg:.2f}", f"{r.roll_deg:.2f}",
                                    f"{r.ax_mg:.0f}", f"{r.ay_mg:.0f}", f"{r.az_mg:.0f}"])
            print(f"\nevery reading -> {args.csv}")

        print(f"\nlink: {reader.packets} packets, {reader.bad_checksums} bad checksums, "
              f"{reader.dropped} dropped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
