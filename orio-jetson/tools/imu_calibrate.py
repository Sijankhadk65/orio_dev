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
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio.imu import RATE_HZ, RvcReader, _wrap180  # noqa: E402

SETTLE_S = 1.0  # let a hand-placed robot stop swaying before measuring
SAMPLE_S = 3.0
STILL_SIGMA_DEG = 0.05  # a hand resting on the body shows up well above this
MIN_TILT_DEG = 0.8  # below this a shim step proves nothing


class Step:
    """One pose: what to ask for, and what came back."""

    def __init__(self, key: str, prompt: str, detail: str) -> None:
        self.key = key
        self.prompt = prompt
        self.detail = detail
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


STEPS = [
    Step(
        "rest",
        "Robot on the floor, castor TRAILING (roll it forward ~0.5 m first). Nothing touching it.",
        "the calibration zero",
    ),
    Step(
        "nose_down",
        "Chock the wheels. Roll/ease the CASTOR up onto the shim so the NOSE goes DOWN.",
        "which angle is pitch, and its sign",
    ),
    Step(
        "left_up",
        "Shim out from under the castor. Now get the LEFT drive wheel up onto it (left side UP).",
        "which angle is roll, and its sign",
    ),
    Step(
        "turn_left",
        "Shim away, robot flat. Push the body a QUARTER TURN TO THE LEFT on the spot (~90 deg CCW seen from above).",
        "the sign of yaw",
    ),
    Step(
        "rest2",
        "Leave it where it is, flat on the floor, hands off.",
        "repeatability against step 1",
    ),
]


def capture(reader: RvcReader, step: Step, sample_s: float) -> None:
    """Fill `step.samples` with a still window, warning if it was not still."""
    print(f"    settling {SETTLE_S:g}s ... ", end="", flush=True)
    time.sleep(SETTLE_S)
    print(f"measuring {sample_s:g}s ... ", end="", flush=True)

    deadline = time.monotonic() + sample_s
    samples = []
    last = None
    while time.monotonic() < deadline:
        reading = reader.fresh(max_age_s=0.2)
        # We poll faster than 100 Hz, so the same packet comes back repeatedly:
        # keep each one once. By timestamp, not by index — the sensor's index
        # wraps every 2.56 s, which is shorter than a measuring window.
        if reading is not None and reading.timestamp != last:
            samples.append(reading)
            last = reading.timestamp
        time.sleep(1.0 / (RATE_HZ * 3))
    step.samples = samples

    expected = sample_s * RATE_HZ
    print(f"{len(step.samples)} packets")
    if len(step.samples) < expected * 0.8:
        print(f"    ! only {len(step.samples)} of ~{expected:.0f} packets — check the link")
    if step.samples and step.sigma > STILL_SIGMA_DEG:
        print(f"    ! not still (sigma {step.sigma:.3f} deg) — hands off, and re-run this step")


def axis_report(rest: Step, tilted: Step, label: str) -> tuple[str, int, float]:
    """Which angle moved, by how much, and which way. Returns (angle, sign, deg).

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
    accel = "ay" if abs(d_ay) >= abs(d_ax) else "ax"

    print(f"  {label}:")
    print(f"    d pitch {d_pitch:+7.2f} deg   d roll {d_roll:+7.2f} deg")
    print(f"    d ax    {d_ax:+7.0f} mg    d ay   {d_ay:+7.0f} mg")
    print(f"    -> moved {angle} by {delta:+.2f} deg; accel followed on {accel}")

    if abs(delta) < MIN_TILT_DEG:
        print(f"    ! only {abs(delta):.2f} deg of tilt — use a thicker shim and re-run")
    if abs(other) > abs(delta) * 0.5:
        print(f"    ! the other angle moved {other:+.2f} deg too — shim placed off to one side?")

    # Nose down wants a negative report; left-side-up (leaning right) wants positive.
    want_negative = label.startswith("nose")
    sign = -1 if (delta > 0) == want_negative else 1
    return angle, sign, delta


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--seconds", type=float, default=SAMPLE_S, help="measuring window per step")
    ap.add_argument("--shim-mm", type=float, default=None, help="shim thickness, for the log only")
    ap.add_argument("--csv", type=Path, default=None, help="write every sample here")
    args = ap.parse_args()

    print(__doc__.split("##")[0].strip())
    print("\nThe robot is NEVER lifted and NEVER driven. Ctrl-C to abort.\n")

    with RvcReader(port=args.port) as reader:
        if reader.fresh(max_age_s=1.0) is None:
            time.sleep(0.5)
        if reader.fresh(max_age_s=1.0) is None:
            print(f"No packets on {args.port}. Check P0/P1/BT strapping and the TX wire.")
            return 1

        for n, step in enumerate(STEPS, 1):
            print(f"[{n}/{len(STEPS)}] {step.prompt}\n    ({step.detail})")
            try:
                input("    press Enter when it is in place and your hands are off ... ")
            except (EOFError, KeyboardInterrupt):
                print("\naborted")
                return 1
            capture(reader, step, args.seconds)
            print()

        by_key = {s.key: s for s in STEPS}
        if any(not s.samples for s in STEPS):
            print("a step captured nothing — nothing to report")
            return 1

        rest = by_key["rest"]
        print("=" * 72)
        print("REST POSE (the calibration zero)")
        m = rest.mean
        print(f"  pitch {m['pitch_deg']:+.2f} deg   roll {m['roll_deg']:+.2f} deg   (sigma {rest.sigma:.3f})")
        print(f"  accel ax {m['ax_mg']:+.0f}  ay {m['ay_mg']:+.0f}  az {m['az_mg']:+.0f} mg")

        print("\nAXES")
        pitch_axis, pitch_sign, _ = axis_report(rest, by_key["nose_down"], "nose down (castor shimmed)")
        roll_axis, roll_sign, _ = axis_report(rest, by_key["left_up"], "left side up (left wheel shimmed)")
        if pitch_axis == roll_axis:
            print(f"\n  ! both steps moved '{pitch_axis}' — one of the two shims was misplaced")

        d_yaw = _wrap180(by_key["turn_left"].yaw_mean - rest.yaw_mean)
        yaw_sign = 1 if d_yaw > 0 else -1
        print(f"\n  quarter turn left: d yaw {d_yaw:+.1f} deg -> yaw sign {yaw_sign:+d}")
        if abs(d_yaw) < 30:
            print("    ! less than 30 deg of turn measured — turn further and re-run")

        rest2 = by_key["rest2"]
        dp = rest2.mean["pitch_deg"] - m["pitch_deg"]
        dr = rest2.mean["roll_deg"] - m["roll_deg"]
        print(f"\nREPEATABILITY  d pitch {dp:+.2f} deg   d roll {dr:+.2f} deg")
        if max(abs(dp), abs(dr)) > 0.5:
            print("  ! the rest pose moved — castor re-pointed, or the bracket is loose")

        print("\n" + "=" * 72)
        print("Put these in config.py (or settings.json / the environment):\n")
        print(f'  ORIO_IMU_PITCH_OFFSET_DEG = {m["pitch_deg"]:.2f}')
        print(f'  ORIO_IMU_ROLL_OFFSET_DEG  = {m["roll_deg"]:.2f}')
        print(f"  ORIO_IMU_PITCH_SIGN       = {pitch_sign:+d}")
        print(f"  ORIO_IMU_ROLL_SIGN        = {roll_sign:+d}")
        print(f"  ORIO_IMU_YAW_SIGN         = {yaw_sign:+d}")
        print("\nAfter setting them, re-run this tool: step 1 should read ~0.00 / ~0.00.")
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
