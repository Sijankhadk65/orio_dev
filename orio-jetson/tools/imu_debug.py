#!/usr/bin/env python3
"""What the IMU is reporting right now, through the config's calibration.

The counterpart to `tools/stereo_debug.py`: one screen showing what the rest of
the code sees. Unlike the calibration tools, this applies
`config.IMU_*` — so it is the tool that says whether a calibration took.

    uv run python tools/imu_debug.py
    uv run python tools/imu_debug.py --seconds 30

Sitting still on the floor, front-back and side-lean should both read close to
0.00 after a good calibration; they are the rest pose with the measured offsets
taken out. Heading is zeroed at startup (`--no-zero` to keep the sensor's own),
because yaw only means anything against a recent reference.

Reads only. Nothing here commands a motor.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config  # noqa: E402
from orio.imu import reader_from_config  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = forever)")
    ap.add_argument("--no-zero", action="store_true", help="do not zero the heading at startup")
    args = ap.parse_args()

    print(f"IMU on {config.IMU_PORT}  (enabled={config.IMU_ENABLED}, stale>{config.IMU_STALE_S:g}s)")
    print(f"calibration: pitch offset {config.IMU_PITCH_OFFSET_DEG:+.2f} deg "
          f"x{config.IMU_PITCH_SIGN:+d}, roll offset {config.IMU_ROLL_OFFSET_DEG:+.2f} deg "
          f"x{config.IMU_ROLL_SIGN:+d}, yaw x{config.IMU_YAW_SIGN:+d}")
    print("sitting still, front-back and side-lean should both read about 0.00\n")

    with reader_from_config() as imu:
        time.sleep(0.5)
        if imu.fresh(max_age_s=1.0) is None:
            print(f"No data on {config.IMU_PORT}. Check the strapping (P0 high, P1 low, BT high),")
            print("the TX wire to pin 10, then power-cycle the sensor.")
            return 1
        if not args.no_zero:
            imu.zero_heading()

        deadline = time.monotonic() + args.seconds if args.seconds else None
        try:
            while deadline is None or time.monotonic() < deadline:
                r = imu.fresh(max_age_s=config.IMU_STALE_S)
                if r is None:
                    sys.stdout.write("\rSTALE — no fresh reading" + " " * 40)
                else:
                    sys.stdout.write(
                        f"\rfront-back {r.pitch_deg:+7.2f}  side-lean {r.roll_deg:+7.2f}  "
                        f"heading {r.heading_deg:+7.1f}  |a| {r.accel_mg:4.0f} mg  "
                        f"[{imu.packets} pkt, {imu.bad_checksums} bad, {imu.dropped} dropped]"
                    )
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
