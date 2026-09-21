#!/usr/bin/env python3
"""Step the neck through a range of tilts and print the sector map at each one.

    uv run python tools/tilt_sweep.py                       # tilt 20..40 in 5s
    uv run python tools/tilt_sweep.py --tilts 30,35,40
    uv run python tools/tilt_sweep.py --samples 15 --settle 1.0

This is the measurement `config.NECK_TILT_DEG` is chosen from, and the one its
comment says to redo after ANY change to the head geometry or the camera mount.
It was last taken by hand on 2026-09-09, before the body was inclined; the body
is level again, so the aim needs choosing afresh rather than carried over.

Stand the robot on open floor facing the longest clear run the room has, with
nothing in the first couple of metres. At each tilt the tool reads the stereo
map `--samples` times and prints, per sector, the MEDIAN distance and how often
that sector was known at all. What you are looking for is the tilt at which:

  * the centre sector is known nearly every time (the floor ahead gives it
    texture) — that is what lets the avoider cruise instead of pivoting blind,
  * and its distance is comfortably ABOVE `AVOID_CLEAR_M`, or the floor itself
    reads as an obstacle and the robot only ever steers (tilt 45 did this on
    2026-09-09: 1.06 m straight ahead against a 1.20 m clear threshold).

Then put a person or a box about 1 m ahead and run it again: the aim must also
see something standing on the floor at that range, not only the floor.

Moves the head only; the wheels are never touched. The firmware refuses any
tilt outside `kJointLimits[]` and nothing moves — the tool reports it and
carries on with the next tilt.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.motion import JOINT_NECK, Motion, travel_time_s
from orio.stereo import ObstacleDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--motion-port", default=config.MOTION_PORT, help="motion board serial port")
    parser.add_argument("--pan", type=float, default=config.NECK_PAN_DEG, help="pan held for the sweep")
    parser.add_argument("--tilts", default="20,25,30,35,40", help="comma-separated tilts to visit")
    parser.add_argument("--samples", type=int, default=10, help="stereo readings per tilt")
    parser.add_argument("--settle", type=float, default=0.8, help="extra wait after each move")
    return parser.parse_args()


def fmt(distance: float | None) -> str:
    return "  ----" if distance is None else f"{distance:6.2f}"


def measure(detector: ObstacleDetector, samples: int):
    """Per sector: (angle, median known distance or None, fraction known)."""
    readings = [detector.sense() for _ in range(samples)]
    out = []
    for i, sector in enumerate(readings[0].sectors):
        known = [r.sectors[i].distance_m for r in readings if r.sectors[i].known]
        median = statistics.median(known) if known else None
        out.append((sector.angle_deg, median, len(known) / samples))
    return out


def main() -> int:
    args = parse_args()
    tilts = [float(t) for t in args.tilts.split(",") if t.strip()]

    print(f"--- motion board on {args.motion_port} ---")
    # Kept open for the whole run: the board releases the servos 500 ms after
    # the last heartbeat, and a slack neck sags between readings.
    link = Motion(args.motion_port).connect()
    detector = ObstacleDetector()
    try:
        print("--- opening stereo (both sensors, ~2 s) ---")
        detector.sense()

        rows = []
        previous = None
        for tilt in tilts:
            rejection = link.move_to(JOINT_NECK, args.pan, tilt)
            if rejection is not None:
                print(f"tilt {tilt:g}: refused by the board ({rejection.reason}), skipped")
                continue
            delta = abs(tilt - previous) if previous is not None else 20.0
            time.sleep(travel_time_s(delta) + args.settle)
            previous = tilt
            rows.append((tilt, measure(detector, args.samples)))
            print(f"tilt {tilt:g}: measured")

        if not rows:
            print("no tilt was accepted; check kJointLimits[] in servo_joint.c")
            return 1

        angles = [a for a, _, _ in rows[0][1]]
        centre = len(angles) // 2
        print()
        print(f"median distance per sector, m (---- = never known); pan {args.pan:g}")
        print("tilt  " + " ".join(f"{a:+6.1f}" for a in angles) + "   centre known")
        for tilt, sectors in rows:
            line = " ".join(fmt(d) for _, d, _ in sectors)
            print(f"{tilt:4g}  {line}   {sectors[centre][2]:5.0%}")
        print()
        print(
            f"AVOID_CLEAR_M = {config.AVOID_CLEAR_M:.2f} m, AVOID_STOP_M = {config.AVOID_STOP_M:.2f} m, "
            f"range gate {config.STEREO_MIN_RANGE_M:.2f}-{config.STEREO_MAX_RANGE_M:.1f} m"
        )
        return 0
    finally:
        detector.close()
        link.close()


if __name__ == "__main__":
    sys.exit(main())
