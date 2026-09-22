#!/usr/bin/env python3
"""Calibrate the IMU by driving the robot, instead of tilting it by hand.

Six short manoeuvres at the robot's own duty ceiling, a stop between each, and
the IMU watched throughout. It works out which reported number is which body
axis and which way round each one counts, from motion the robot makes itself.

    uv run python tools/imu_calibrate_drive.py
    uv run python tools/imu_calibrate_drive.py --duty 5 --csv /tmp/imudrive.csv

## Read this before running it: the robot drives BLIND

This is a bench tool. It talks to the drivetrain directly, so **the avoidance
policy is not in the loop** — no cameras, no stereo, no `Avoider`, nothing
watching where it is going. That is the opposite of `tools/teleop_guarded.py`
and of every path through `body.py`, and it is only acceptable because the
manoeuvres are ~1 s each at 5% duty (~0.3 m/s, so ~0.3 m of travel) and a human
is standing over it.

Clear **2 m ahead, 1 m behind and 1 m either side** before starting. Hand stays
near the power switch. Ctrl-C stops the wheels and e-stops the board on the way
out, and so does any exception — that is `Drivetrain.__exit__`'s guarantee.

## What driving can and cannot measure

Honest about the limits, because a confident wrong sign here ends up in the
tip guard:

  * **Forward axis and its sign — solid.** Accelerating forward, then in
    reverse, moves one accelerometer channel one way and then the other.
    Differencing the two cancels gravity and any fixed bias.
  * **Yaw sign — solid.** Pivot left, pivot right, compare. This is the one
    the policy needs most (measured turns, heading hold).
  * **Sideways axis and its sign — good.** An arc has the robot turning about
    a centre off to one side, so there is real centripetal acceleration, and
    the body leans outward. Arc left vs arc right cancels the bias.
  * **Nose up/down (pitch) sign — WEAK, and may come back UNKNOWN.** On flat
    floor the only pitch is the body rocking as it starts and stops, which is
    a few tenths of a degree and is confounded by the castor re-pointing. If
    this one comes back UNKNOWN, do that single step by hand with
    `tools/imu_calibrate.py` (shim under the castor) — that is what it is for.

The rest-pose offsets come from standing still, exactly as in the hand tool.
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

from orio import config  # noqa: E402
from orio.drivetrain import DRIVE_REFRESH_S, Drivetrain  # noqa: E402
from orio.imu import RvcReader, _wrap180  # noqa: E402

SETTLE_S = 1.2  # the body rocks after a stop; let it die down before measuring
REST_S = 2.0
BURST_S = 1.0
PAUSE_S = 1.5

MIN_ACCEL_MG = 8.0  # below this an accel difference is noise
MIN_YAW_DEG = 10.0  # below this a pivot proved nothing
MIN_LEAN_DEG = 0.15  # body lean in an arc is small; this is the floor for it
MIN_PITCH_DEG = 0.25  # start/stop rock, weaker still


class Window:
    """Samples gathered over one span of time, with the means callers want."""

    def __init__(self, samples: list) -> None:
        self.samples = samples

    def mean(self, name: str) -> float:
        return st.mean(getattr(r, name) for r in self.samples) if self.samples else 0.0

    @property
    def yaw_span(self) -> float:
        """How far yaw moved across the window, wrap-safe."""
        if len(self.samples) < 2:
            return 0.0
        return _wrap180(self.samples[-1].heading_deg - self.samples[0].heading_deg)


class Collector:
    """Records every packet inside a window, none outside one."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: list | None = None
        self.all: list = []

    def __call__(self, reading) -> None:
        with self._lock:
            self.all.append(reading)
            if self._samples is not None:
                self._samples.append(reading)

    def start(self) -> None:
        with self._lock:
            self._samples = []

    def stop(self) -> Window:
        with self._lock:
            samples, self._samples = self._samples or [], None
        return Window(samples)


def rest_window(collector: Collector, seconds: float, label: str) -> Window:
    print(f"  {label}: standing still for {seconds:g}s ...")
    time.sleep(SETTLE_S)
    collector.start()
    time.sleep(seconds)
    w = collector.stop()
    print(f"    front-back {w.mean('pitch_deg'):+.2f} deg   side-lean {w.mean('roll_deg'):+.2f} deg"
          f"   ({len(w.samples)} readings)")
    return w


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
    w = collector.stop()
    print(f"    yaw moved {w.yaw_span:+.1f} deg;  ax {w.mean('ax_mg'):+.0f}  ay {w.mean('ay_mg'):+.0f} mg"
          f"   ({len(w.samples)} readings)")
    time.sleep(PAUSE_S)
    return w


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
    v_in = st.mean(w.v_in for w in status.wheels.values())
    print(f"both wheels reporting, {v_in:.1f} V in\n")
    return True


def decide_axis(d_a: float, d_b: float, names: tuple[str, str], floor: float):
    """Which of two channels moved more, its sign, and whether it is believable."""
    name, delta, other = (names[0], d_a, d_b) if abs(d_a) >= abs(d_b) else (names[1], d_b, d_a)
    ok = abs(delta) >= floor and abs(other) <= abs(delta) * 0.5
    return name, delta, ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=config.DRIVETRAIN_PORT,
                    help="drivetrain board (the stable name, not ttyACM*)")
    ap.add_argument("--imu-port", default="/dev/ttyTHS1")
    ap.add_argument("--duty", type=float, default=config.DRIVE_SPEED_MAX_PERCENT,
                    help="straight-line duty PERCENT, capped at the robot's ceiling")
    ap.add_argument("--pivot-duty", type=float, default=config.AVOID_ESCAPE_DUTY / 10.0,
                    help="duty PERCENT for pivots and arcs (the escape duty, which is "
                         "what actually turns this chassis)")
    ap.add_argument("--seconds", type=float, default=BURST_S, help="length of each manoeuvre")
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--yes", action="store_true", help="skip the are-you-clear prompt")
    args = ap.parse_args()

    cap = max(config.DRIVE_SPEED_MAX_PERCENT, config.AVOID_ESCAPE_DUTY / 10.0)
    duty = round(max(-cap, min(cap, args.duty)) * 10)
    pivot = round(max(-cap, min(cap, args.pivot_duty)) * 10)
    seconds = min(args.seconds, 2.0)

    print("=" * 78)
    print("IMU CALIBRATION BY DRIVING")
    print("=" * 78)
    print(f"""
The robot will make 6 short moves on its own: forward, back, pivot left, pivot
right, arc left, arc right. Each lasts {seconds:g}s at {duty / 10:g}% duty
(pivots {pivot / 10:g}%), with a stop in between. Total travel is well under a metre.

IT DRIVES BLIND. The cameras and the avoidance policy are NOT in this loop.

Before you say yes:
  - 2 m clear ahead, 1 m behind, 1 m either side
  - floor is flat and level, nothing to run into or fall off
  - you are standing over it, hand near the power switch
  - Ctrl-C stops the wheels and e-stops the board""")

    if not args.yes:
        try:
            if input("\nIs the space clear? Type yes to drive: ").strip().lower() not in ("y", "yes"):
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
                rest = rest_window(collector, REST_S, "rest (the zero)")
                fwd = manoeuvre(link, collector, "forward", duty, duty, seconds)
                rev = manoeuvre(link, collector, "reverse", -duty, -duty, seconds)
                piv_l = manoeuvre(link, collector, "pivot left", -pivot, pivot, seconds)
                piv_r = manoeuvre(link, collector, "pivot right", pivot, -pivot, seconds)
                arc_l = manoeuvre(link, collector, "arc left", 0, pivot, seconds)
                arc_r = manoeuvre(link, collector, "arc right", pivot, 0, seconds)
                rest2 = rest_window(collector, REST_S, "rest again")
        except KeyboardInterrupt:
            print("\nstopped — wheels zeroed and board e-stopped")
            return 1
        except OSError as exc:
            print(f"\ncould not open the drivetrain on {args.port}: {exc}")
            print("The two STM32 boards have stable names (/dev/orio_drive, /dev/orio_motion);")
            print("pass --port explicitly rather than guessing between ttyACM0 and ttyACM1.")
            return 1

    return report(args, rest, fwd, rev, piv_l, piv_r, arc_l, arc_r, rest2, collector, duty, pivot)


def report(args, rest, fwd, rev, piv_l, piv_r, arc_l, arc_r, rest2, collector, duty, pivot) -> int:
    print("\n" + "=" * 78)
    print("RESULTS")
    print("=" * 78)

    print("\n1. HOW THE ROBOT SITS AT REST")
    pitch0, roll0 = rest.mean("pitch_deg"), rest.mean("roll_deg")
    print(f"   Front-back: {pitch0:+.2f} deg ({'nose down' if pitch0 < 0 else 'nose up'})")
    print(f"   Side-lean:  {roll0:+.2f} deg")

    # Forward vs reverse: the difference cancels gravity and any fixed bias.
    d_ax = (fwd.mean("ax_mg") - rev.mean("ax_mg")) / 2
    d_ay = (fwd.mean("ay_mg") - rev.mean("ay_mg")) / 2
    fwd_axis, fwd_delta, fwd_ok = decide_axis(d_ax, d_ay, ("ax", "ay"), MIN_ACCEL_MG)
    print("\n2. WHICH WAY IS FORWARD")
    print(f"   Driving forward vs reverse: ax {d_ax:+.0f} mg, ay {d_ay:+.0f} mg")
    if fwd_ok:
        print(f"   -> forward shows on {fwd_axis.upper()}, "
              f"{'positive' if fwd_delta > 0 else 'negative'} when accelerating forward")
        print(f"   ({'+Y forward, as the silkscreen says' if fwd_axis == 'ay' else 'NOT the Y axis — check the mounting'})")
    else:
        print("   COULD NOT TELL: too little acceleration to separate. Try a longer")
        print("                   burst (--seconds 1.5) on a surface with more grip.")

    # Yaw: pivot left vs right.
    yaw_l, yaw_r = piv_l.yaw_span, piv_r.yaw_span
    yaw_ok = abs(yaw_l) >= MIN_YAW_DEG and abs(yaw_r) >= MIN_YAW_DEG and (yaw_l > 0) != (yaw_r > 0)
    yaw_sign = 1 if yaw_l > 0 else -1
    print("\n3. WHICH WAY IS A LEFT TURN")
    print(f"   Pivot left: yaw {yaw_l:+.1f} deg      pivot right: yaw {yaw_r:+.1f} deg")
    if yaw_ok:
        print(f"   -> the sensor calls a left turn {'positive' if yaw_sign > 0 else 'negative'}; "
              "the setting below makes it positive")
    else:
        print("   COULD NOT TELL: the pivots did not turn far enough, or both went the")
        print("                   same way. Raise --pivot-duty or --seconds and re-run.")

    # Arc left vs arc right: centripetal acceleration sideways, and the body leans out.
    d_lat_ax = (arc_l.mean("ax_mg") - arc_r.mean("ax_mg")) / 2
    d_lat_ay = (arc_l.mean("ay_mg") - arc_r.mean("ay_mg")) / 2
    lat_axis, lat_delta, lat_ok = decide_axis(d_lat_ax, d_lat_ay, ("ax", "ay"), MIN_ACCEL_MG)
    d_lean = (arc_l.mean("roll_deg") - arc_r.mean("roll_deg")) / 2
    d_lean_pitch = (arc_l.mean("pitch_deg") - arc_r.mean("pitch_deg")) / 2
    lean_axis, lean_delta, lean_ok = decide_axis(
        d_lean_pitch, d_lean, ("pitch", "roll"), MIN_LEAN_DEG)
    # Turning left, the body leans RIGHT (outward), which is +roll by convention.
    roll_sign = 1 if lean_delta > 0 else -1
    print("\n4. WHICH NUMBER IS THE SIDE-LEAN")
    print(f"   Arc left vs arc right: ax {d_lat_ax:+.0f} mg, ay {d_lat_ay:+.0f} mg;"
          f" front-back {d_lean_pitch:+.2f} deg, side-lean {d_lean:+.2f} deg")
    if lat_ok:
        print(f"   -> sideways shows on {lat_axis.upper()}")
    if lean_ok and lean_axis == "roll":
        print(f"   -> leaning out of a left turn moves the SIDE-LEAN number "
              f"{lean_delta:+.2f} deg, so leaning right is "
              f"{'positive' if roll_sign > 0 else 'negative'}")
    else:
        lean_ok = False
        print("   COULD NOT TELL: the lean in an arc was too small to read here.")
        print("                   Use tools/imu_calibrate.py step 3 (shim under the")
        print("                   left wheel) for this one — it tilts far further.")

    # Pitch: the body rocks as it starts and stops. Weak by construction.
    d_pitch = (fwd.mean("pitch_deg") - rev.mean("pitch_deg")) / 2
    pitch_ok = abs(d_pitch) >= MIN_PITCH_DEG
    pitch_sign = -1 if d_pitch > 0 else 1
    print("\n5. WHICH WAY IS NOSE-DOWN (the weak one)")
    print(f"   Accelerating forward vs reverse moved front-back {d_pitch:+.2f} deg")
    if pitch_ok:
        print("   -> accelerating forward pitches the nose UP, so nose-down is "
              f"{'negative' if pitch_sign > 0 else 'positive'}")
    else:
        print("   COULD NOT TELL: only body rock is available on a flat floor, and it")
        print("                   was below the noise. Do this one by hand:")
        print("                   tools/imu_calibrate.py, step 2 (shim under the castor).")

    dp = rest2.mean("pitch_deg") - pitch0
    dr = rest2.mean("roll_deg") - roll0
    print("\n6. DID IT COME BACK TO THE SAME PLACE?")
    print(f"   front-back moved {dp:+.2f} deg, side-lean moved {dr:+.2f} deg after driving")
    if max(abs(dp), abs(dr)) > 0.8:
        print("   That is more than the castor alone explains — check the IMU bracket.")
    else:
        print("   Fine — that is the castor re-pointing, which is real body tilt.")

    unknown = "UNKNOWN — see above"
    print("\n" + "=" * 78)
    print("7. SAVE THESE SETTINGS")
    print("=" * 78)
    print(f"  ORIO_IMU_PITCH_OFFSET_DEG = {pitch0:.2f}")
    print(f"  ORIO_IMU_ROLL_OFFSET_DEG  = {roll0:.2f}")
    print(f"  ORIO_IMU_PITCH_SIGN       = {f'{pitch_sign:+d}' if pitch_ok else unknown}")
    print(f"  ORIO_IMU_ROLL_SIGN        = {f'{roll_sign:+d}' if lean_ok else unknown}")
    print(f"  ORIO_IMU_YAW_SIGN         = {f'{yaw_sign:+d}' if yaw_ok else unknown}")

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
