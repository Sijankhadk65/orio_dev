#!/usr/bin/env python3
"""Work out how the BNO085 is mounted, by tilting the robot on purpose.

The IMU reports angles in its own frame. What the code needs is the BODY's
attitude, and between the two sit a mount that is never perfectly square, a
chassis that has its own lean, and axis names that do not mean what the
datasheet calls them once the part is mounted with +Y forward. This tool
measures all three, one step at a time, and prints the `config.py` lines.

It is a measuring tool: **nothing here ever commands a motor.** The robot is
moved by hand, and every step is a pose you put it in and then leave alone.

    uv run python tools/imu_calibrate.py
    uv run python tools/imu_calibrate.py --shim-mm 30 --csv /tmp/imucal.csv

## The robot is heavy — do not lift it

Every tilt here is made by rolling the robot onto a shim (a book, a plank, a
wedge of ply: 20-40 mm), never by picking an end up. The steps are ordered so
the castor end moves first, because that is the light end and the one that
rolls.

Chock the wheels before shimming. A robot this heavy that starts rolling does
not get caught by hand.

## What each step measures

  1. **Rest, castor trailing** — the calibration zero. Every threshold in
     `avoid.py` was measured with the body in this pose, so "level" for this
     robot means "as it sits here", not "as a spirit level sees it". Roll it
     forward half a metre first so the castor trails instead of sitting
     crossways: the castor pitches the body by ~0.6 deg depending on where it
     points, which is real body pitch and not noise.
  2. **Shim under the castor** — the rear rises, the nose goes down. Fixes
     which reported angle is body pitch, and which way is negative.
  3. **Shim under one drive wheel** — one side rises. Fixes which reported
     angle is body roll, and its sign.
  4. **Rotate a quarter turn to the left** — push the body round on the spot.
     Fixes the sign of yaw, so "+30 deg" means a left turn to the policy.
  5. **Rest again** — repeatability. A second rest that disagrees with the
     first by more than a few tenths of a degree means something moved: the
     bracket, the castor, or the shim was still under a wheel.

Steps 2-4 only need the SIGN, so a small tilt is enough; 20-40 mm of shim is
plenty. The magnitudes are printed as a cross-check, not as a target.
"""

from __future__ import annotations

import argparse
import csv
import statistics as st
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio.imu import RATE_HZ, RvcReader, _wrap180  # noqa: E402

SETTLE_S = 1.0  # let a hand-placed robot stop swaying before measuring
SAMPLE_S = 3.0
STILL_SIGMA_DEG = 0.05  # a hand resting on the body shows up well above this
MIN_TILT_DEG = 0.8  # below this a shim step proves nothing


class Step:
    """One pose: how to put the robot in it, and what came back."""

    def __init__(self, key: str, title: str, instructions: list[str], measures: str,
                 expect: str = "") -> None:
        self.key = key
        self.title = title
        self.instructions = instructions
        self.measures = measures
        self.expect = expect  # what should change on screen, in plain words
        self.samples: list = []

    @property
    def mean(self) -> dict:
        return {
            name: st.mean(getattr(r, name) for r in self.samples)
            for name in ("pitch_deg", "roll_deg", "ax_mg", "ay_mg", "az_mg")
        }

    @property
    def yaw_mean(self) -> float:
        return st.mean(r.heading_deg for r in self.samples)

    @property
    def sigma(self) -> float:
        return max(
            st.pstdev([r.pitch_deg for r in self.samples]),
            st.pstdev([r.roll_deg for r in self.samples]),
        )


DIAGRAM = """
    Which end is which, and which side is LEFT:

            FRONT  =  the two drive wheels (hub motors), the way the robot drives
             ___________
            |  o     o  |   <- drive wheels, left one on YOUR left when you
     LEFT   |           |      stand BEHIND the robot looking the way it faces
      side  |    []     |   <- the IMU, on the plank
            |     o     |   <- castor (the small swivel wheel)
             -----------
            BACK  =  the castor end. This is the light end. Shim here first.

    "left" always means the ROBOT's left, i.e. your left when you stand
    behind it and look forward with it.
"""

STEPS = [
    Step(
        "rest",
        "Robot flat on the floor, straight",
        [
            "Put the robot on flat, level floor.",
            "Push it FORWARD about half a metre and let it stop on its own.",
            "  (that makes the castor line up behind, the way it sits after driving)",
            "Take your hands off it completely. Do not lean on it.",
        ],
        "the resting position — everything else is measured against this",
        expect="nothing should move; the numbers should sit still",
    ),
    Step(
        "nose_down",
        "Tilt the FRONT down, by raising the BACK",
        [
            "Chock both drive wheels (front) so the robot cannot roll.",
            "Put the shim (a book, 20-40 mm) on the floor just behind the castor.",
            "Ease the CASTOR up onto the shim, so the BACK of the robot is higher",
            "  and the FRONT dips down. Do NOT lift the robot — roll/slide it on.",
            "Hands off once it is sitting on the shim.",
        ],
        "which number is the front-to-back tilt, and which way is down",
        expect="one number should move by a few degrees, the other should barely move",
    ),
    Step(
        "left_up",
        "Tilt the LEFT side up",
        [
            "Take the shim out from under the castor; put the robot flat again.",
            "Now put the shim under the LEFT drive wheel only.",
            "  (left = your left when standing BEHIND the robot, looking forward)",
            "Roll or ease that wheel onto it so the LEFT side sits higher than the right.",
            "Hands off.",
        ],
        "which number is the side-to-side lean, and which way is which",
        expect="the OTHER number should now move by a few degrees",
    ),
    Step(
        "turn_left",
        "Turn the whole robot a quarter turn to the LEFT",
        [
            "Take the shim away. Robot flat on the floor again.",
            "Push the body around on the spot, to the LEFT, about a quarter turn (90 deg).",
            "  Left = anticlockwise seen from above. If it faced the window, it should",
            "  now face whatever was to the left of the window.",
            "It does not have to be exactly 90 deg — anything over 30 deg works.",
            "Hands off.",
        ],
        "which direction of turn counts as positive",
        expect="the heading number should change by roughly the angle you turned",
    ),
    Step(
        "rest2",
        "Leave it alone, flat on the floor",
        [
            "Do not move the robot at all for this step.",
            "Just make sure nothing is under any wheel and nothing is touching it.",
        ],
        "whether the resting position is repeatable",
        expect="should read close to step 1 (within a few tenths of a degree)",
    ),
]


def live_preview(reader: RvcReader, note: str) -> None:
    """Show the numbers moving while the robot is being put in place.

    Without this the tool is a black box with a blinking cursor: you cannot
    tell a wrongly-placed shim from a dead link until the summary at the end.
    """
    print(f"    live: {note}")
    print("    (press Enter when the robot is in place and your hands are off)")
    stop = threading.Event()

    def show() -> None:
        while not stop.is_set():
            r = reader.fresh(max_age_s=0.5)
            if r is not None:
                sys.stdout.write(
                    f"\r      front-back {r.pitch_deg:+7.2f} deg   "
                    f"side-lean {r.roll_deg:+7.2f} deg   heading {r.heading_deg:+7.1f} deg  "
                )
            else:
                sys.stdout.write("\r      no data from the IMU ...                      ")
            sys.stdout.flush()
            time.sleep(0.1)

    thread = threading.Thread(target=show, daemon=True)
    thread.start()
    try:
        input()
    finally:
        stop.set()
        thread.join(timeout=0.5)
        sys.stdout.write("\r" + " " * 78 + "\r")


def capture(collector: "Collector", step: Step, sample_s: float) -> None:
    """Fill `step.samples` with a still window, warning if it was not still."""
    print(f"    holding still for {SETTLE_S:g}s, then measuring for {sample_s:g}s ...")
    time.sleep(SETTLE_S)
    collector.start()
    time.sleep(sample_s)
    step.samples = collector.stop()

    if not step.samples:
        print("    NOTHING MEASURED — the IMU stopped sending. Check the wire.")
        return

    m = step.mean
    print(f"    measured: front-back {m['pitch_deg']:+.2f} deg, "
          f"side-lean {m['roll_deg']:+.2f} deg  ({len(step.samples)} readings)")

    expected = sample_s * RATE_HZ
    if len(step.samples) < expected * 0.8:
        print(f"    PROBLEM: only {len(step.samples)} readings, expected about {expected:.0f}.")
        print("             The link is dropping data — check the TX wire and the 3.3 V.")
    if step.sigma > STILL_SIGMA_DEG:
        print(f"    PROBLEM: the robot was still moving (wobble {step.sigma:.2f} deg).")
        print("             Take your hands off, wait for it to settle, and re-run this step.")


class Collector:
    """Keeps every packet during a measuring window, none outside one.

    The reader hands packets over in bursts, so a window has to be recorded as
    they arrive rather than sampled — see `RvcReader(on_reading=...)`.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: list | None = None

    def __call__(self, reading) -> None:
        with self._lock:
            if self._samples is not None:
                self._samples.append(reading)

    def start(self) -> None:
        with self._lock:
            self._samples = []

    def stop(self) -> list:
        with self._lock:
            samples, self._samples = self._samples or [], None
        return samples


def axis_report(rest: Step, tilted: Step, label: str) -> tuple[str, int, float]:
    """Which angle moved, by how much, and which way. Returns (angle, sign, deg).

    `ok` is False when the step did not prove anything — too little tilt, or a
    shim placed so far off centre that both angles moved. A sign guessed from
    0.03 deg of noise is worse than no sign at all, so the caller prints
    UNKNOWN rather than a number in that case.

    `sign` is what `config.py` needs so that the reported angle is positive in
    the direction named in `orio/imu.py`: pitch + is nose UP, roll + is leaning
    RIGHT. The tilt steps put the robot nose DOWN and left side UP (= leaning
    right), so the expected reported signs are negative and positive.
    """
    a, b = rest.mean, tilted.mean
    d_pitch = b["pitch_deg"] - a["pitch_deg"]
    d_roll = b["roll_deg"] - a["roll_deg"]
    d_ax = b["ax_mg"] - a["ax_mg"]
    d_ay = b["ay_mg"] - a["ay_mg"]

    angle, delta = ("pitch", d_pitch) if abs(d_pitch) >= abs(d_roll) else ("roll", d_roll)
    other = d_roll if angle == "pitch" else d_pitch
    moved = "front-back" if angle == "pitch" else "side-lean"

    print(f"  {label}")
    print(f"    front-back changed {d_pitch:+7.2f} deg      side-lean changed {d_roll:+7.2f} deg")

    ok = True
    if abs(delta) < MIN_TILT_DEG:
        ok = False
        print(f"    COULD NOT TELL: the robot barely tilted ({abs(delta):.2f} deg).")
        print("                    Use a thicker shim (30-40 mm), make sure the wheel or")
        print("                    castor is properly up on it, and run the tool again.")
    elif abs(other) > abs(delta) * 0.5:
        ok = False
        print(f"    COULD NOT TELL: the other number moved {other:+.2f} deg as well, which")
        print("                    means the shim was off to one side. Centre it under the")
        print("                    wheel (or the castor) and run the tool again.")
    else:
        print(f"    -> this tilt is the {moved.upper()} number ({abs(delta):.1f} deg tilt)")

    # Nose down wants a negative report; left-side-up (leaning right) wants positive.
    want_negative = label.lower().startswith("front")
    sign = -1 if (delta > 0) == want_negative else 1
    return angle, sign, delta, ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--seconds", type=float, default=SAMPLE_S, help="measuring window per step")
    ap.add_argument("--shim-mm", type=float, default=None, help="shim thickness, for the log only")
    ap.add_argument("--csv", type=Path, default=None, help="write every sample here")
    args = ap.parse_args()

    print("=" * 78)
    print("IMU MOUNT CALIBRATION")
    print("=" * 78)
    print("""
This works out how the IMU board sits inside the robot, so the code can tell
the BODY's tilt from the sensor's own tilt.

There are 5 steps. Each one: put the robot in a position, press Enter, keep
your hands off for about 4 seconds while it measures. Then it prints the
settings to save.

THE ROBOT IS NEVER LIFTED AND NEVER DRIVEN. Tilts are made by rolling it onto
a shim -- a book or a plank, 20-40 mm thick. Chock the wheels first.

You need: a shim (20-40 mm), something to chock the wheels, flat floor.
Ctrl-C at any time to stop.""")
    print(DIAGRAM)

    collector = Collector()
    with RvcReader(port=args.port, on_reading=collector) as reader:
        if reader.fresh(max_age_s=1.0) is None:
            time.sleep(0.5)
        if reader.fresh(max_age_s=1.0) is None:
            print(f"No data from the IMU on {args.port}.")
            print("Check: P0 to 3Vo, P1 to GND, BT to 3Vo, SDA to Jetson pin 10,")
            print("then power-cycle the board (it only reads those pins at power-up).")
            return 1

        try:
            input("Ready? Press Enter to start. ")
        except (EOFError, KeyboardInterrupt):
            print("\nstopped")
            return 1

        for n, step in enumerate(STEPS, 1):
            print("\n" + "-" * 78)
            print(f"STEP {n} of {len(STEPS)}: {step.title}")
            print("-" * 78)
            for line in step.instructions:
                print(f"    {line}" if line.startswith(" ") else f"  - {line}")
            print(f"\n    This step finds: {step.measures}")
            try:
                live_preview(reader, step.expect)
            except (EOFError, KeyboardInterrupt):
                print("\nstopped")
                return 1
            capture(collector, step, args.seconds)

        by_key = {s.key: s for s in STEPS}
        if any(not s.samples for s in STEPS):
            print("a step captured nothing — nothing to report")
            return 1

        rest = by_key["rest"]
        print("\n" + "=" * 78)
        print("RESULTS")
        print("=" * 78)
        m = rest.mean
        print("\n1. HOW THE ROBOT SITS AT REST")
        print(f"   Front-back: {m['pitch_deg']:+.2f} deg "
              f"({'nose down' if m['pitch_deg'] < 0 else 'nose up'})")
        print(f"   Side-lean:  {m['roll_deg']:+.2f} deg")
        print("   This is what the code will treat as 'level', because every distance")
        print("   the robot drives by was measured with it sitting like this.")

        print("\n2. WHICH NUMBER MEANS WHAT")
        pitch_axis, pitch_sign, _, pitch_ok = axis_report(rest, by_key["nose_down"], "Front tilted down:")
        roll_axis, roll_sign, _, roll_ok = axis_report(rest, by_key["left_up"], "Left side tilted up:")
        if pitch_axis == roll_axis:
            print("\n   PROBLEM: both tilts moved the same number, so the two shim")
            print("            positions cannot be told apart. Re-run, and make sure")
            print("            step 2 shims the CASTOR and step 3 the LEFT WHEEL.")

        d_yaw = _wrap180(by_key["turn_left"].yaw_mean - rest.yaw_mean)
        yaw_sign = 1 if d_yaw > 0 else -1
        yaw_ok = abs(d_yaw) >= 30
        print(f"\n   Quarter turn left: heading changed {d_yaw:+.1f} deg")
        if yaw_ok:
            print(f"   -> the sensor calls a left turn {'positive' if d_yaw > 0 else 'negative'}, "
                  "and the setting below makes it positive")
        else:
            print("   COULD NOT TELL: less than 30 deg of turn was measured. Push the robot")
            print("                   further round (a quarter turn) and run the tool again.")

        rest2 = by_key["rest2"]
        dp = rest2.mean["pitch_deg"] - m["pitch_deg"]
        dr = rest2.mean["roll_deg"] - m["roll_deg"]
        print("\n3. DOES IT COME BACK TO THE SAME PLACE?")
        print(f"   Between the first and last steps: front-back moved {dp:+.2f} deg, "
              f"side-lean moved {dr:+.2f} deg")
        if max(abs(dp), abs(dr)) > 0.5:
            print("   PROBLEM: that is more than expected. Either the castor is pointing")
            print("            a different way (normal, up to ~0.6 deg), or the IMU bracket")
            print("            is loose. Check the bracket, then run the tool again.")
        else:
            print("   Good — that is within the expected range.")

        print("\n" + "=" * 78)
        print("4. SAVE THESE SETTINGS")
        print("=" * 78)
        print("Add these to settings.json (or tell Claude to put them in config.py):\n")
        unknown = "UNKNOWN — re-run the step above"
        print(f'  ORIO_IMU_PITCH_OFFSET_DEG = {m["pitch_deg"]:.2f}')
        print(f'  ORIO_IMU_ROLL_OFFSET_DEG  = {m["roll_deg"]:.2f}')
        print(f"  ORIO_IMU_PITCH_SIGN       = {f'{pitch_sign:+d}' if pitch_ok else unknown}")
        print(f"  ORIO_IMU_ROLL_SIGN        = {f'{roll_sign:+d}' if roll_ok else unknown}")
        print(f"  ORIO_IMU_YAW_SIGN         = {f'{yaw_sign:+d}' if yaw_ok else unknown}")
        if not (pitch_ok and roll_ok and yaw_ok):
            print("\nThe two offsets above are good and can be saved now. The UNKNOWN")
            print("lines need their step doing again — the tool will not guess them.")
        print("\nCheck it worked: save them, run this tool again, and at step 1 both")
        print("numbers should read close to 0.00.")
        if args.shim_mm:
            print(f"(shim used: {args.shim_mm:g} mm)")

        if args.csv:
            with args.csv.open("w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["step", "index", "yaw", "pitch", "roll", "ax_mg", "ay_mg", "az_mg"])
                for step in STEPS:
                    for r in step.samples:
                        w.writerow([step.key, r.index, f"{r.heading_deg:.2f}",
                                    f"{r.pitch_deg:.2f}", f"{r.roll_deg:.2f}",
                                    f"{r.ax_mg:.0f}", f"{r.ay_mg:.0f}", f"{r.az_mg:.0f}"])
            print(f"\nsamples -> {args.csv}")

        print(f"\nlink: {reader.packets} packets, {reader.bad_checksums} bad checksums, "
              f"{reader.dropped} dropped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
