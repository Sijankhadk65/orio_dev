#!/usr/bin/env python3
"""What the FSESCs are actually reporting, per wheel, live.

The bench tool for setting `ORIO_BUMP_STALL_CURRENT_A` and
`ORIO_BUMP_STALL_ERPM`. Both are estimates in `config.py` and both need two
populations measured off this robot before `ORIO_BUMP=1` means anything: what a
free-running wheel draws, and what a jammed one draws. Put the threshold
between them.

It runs `orio.bump.StallDetector` alongside the raw numbers with the config
currently in force, so the same screen shows the evidence and the verdict. A
column of `BUMP` while the wheels spin free means the current threshold is too
low; no `BUMP` on a genuine jam means it is too high, or `--stall-erpm` is
below what a stationary wheel really reports.

    # watch, without moving anything
    uv run python tools/wheel_telemetry.py

    # hold a duty while watching — the calibration run
    uv run python tools/wheel_telemetry.py --duty 50

    # log it for later
    uv run python tools/wheel_telemetry.py --duty 50 --csv /tmp/stall.csv

## Safety

Stalling a hub motor at duty is a thermal event on the ESC: sub-ohm windings,
no back-EMF, current pinned at whatever the FSESC limit is. Keep a jam to a
couple of seconds, give the controllers airflow, and touch-check between runs.
`--seconds` defaults to 20 for that reason and `--duty` is capped at 300
per-mille, which is far more than the 50 this robot cruises at.

The wheels are zeroed and the board e-stopped on every exit path, including an
exception and a Ctrl-C — that is `Drivetrain.__exit__`'s guarantee and the only
reason this tool is safe to run with the wheels on the ground.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from orio import config
from orio.bump import StallDetector
from orio.drivetrain import Drivetrain

DUTY_CAP = 300


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Live wheel telemetry, for calibrating the bump thresholds.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default=config.DRIVETRAIN_PORT,
                   help="drivetrain board — a udev symlink, never a raw ttyACM*")
    p.add_argument("--duty", type=int, default=0,
                   help=f"per-mille duty to hold on BOTH wheels while watching, "
                        f"capped at +/-{DUTY_CAP}. 0 just watches")
    p.add_argument("--seconds", type=float, default=20.0,
                   help="stop after this long. Short by default because a stall is "
                        "a thermal event on the ESC")
    p.add_argument("--hz", type=float, default=10.0, help="sample rate")
    p.add_argument("--csv", default=None, help="also append samples to this file")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    duty = max(-DUTY_CAP, min(DUTY_CAP, args.duty))
    if duty != args.duty:
        print(f"duty clamped to {duty} (cap is +/-{DUTY_CAP})")

    detector = StallDetector()
    period = 1.0 / max(args.hz, 0.1)
    peaks = {"left": 0.0, "right": 0.0}
    spinning = {"left": 0.0, "right": 0.0}  # peak current while actually turning
    samples = 0
    invalid = 0
    csv = open(args.csv, "a") if args.csv else None
    if csv:
        csv.write("t,side,erpm,current_a,v_in,fault,valid,duty\n")

    print(f"thresholds in force: current >= {config.BUMP_STALL_CURRENT_A} A, "
          f"|erpm| <= {config.BUMP_STALL_ERPM}, duty >= {config.BUMP_MIN_DUTY}, "
          f"held {config.BUMP_CONFIRM_S}s")
    if duty:
        print(f"HOLDING {duty} per-mille on both wheels for {args.seconds:g}s — "
              f"Ctrl-C stops everything")
    print()

    # Opening is its own try, narrowly. Wrapping the whole run in one handler
    # made a bad call site inside the loop print the cable advice below and
    # blame the port, which cost a real debugging session.
    try:
        link = Drivetrain(args.port).connect()
    except Exception as exc:
        print(f"\ncould not open the drivetrain on {args.port}: {exc}\n"
              "  Check `ls -l /dev/orio_* /dev/ttyACM*`. If /dev/orio_drive is\n"
              "  missing the udev rule is not installed here — find the drivetrain\n"
              "  board in `ls /dev/serial/by-id/` by its ST-LINK serial and pass\n"
              "  --port. Do not guess between ttyACM0 and ttyACM1.")
        if csv:
            csv.close()
        return 1

    try:
        with link:
            print(f"board: {link.identity}\n")
            started = time.monotonic()
            while (time.monotonic() - started) < args.seconds:
                now = time.monotonic()
                if duty:
                    link.set_drive(duty, duty)
                link.request_status()
                time.sleep(period)

                status = link.last_status
                samples += 1
                bump = detector.update(time.monotonic(), (duty, duty), status)
                if status is None:
                    invalid += 1
                    print("\rno status from the board — is it answering?          ",
                          end="", flush=True)
                    continue

                cells = []
                for side in ("left", "right"):
                    tel = status.wheels.get(side)
                    if tel is None or not tel.valid:
                        invalid += 1
                        cells.append(f"{side[0].upper()} ---- invalid ----")
                        continue
                    peaks[side] = max(peaks[side], tel.current_a)
                    if abs(tel.erpm) > config.BUMP_STALL_ERPM:
                        spinning[side] = max(spinning[side], tel.current_a)
                    cells.append(
                        f"{side[0].upper()} {tel.erpm:+6d} erpm {tel.current_a:6.2f} A "
                        f"{tel.v_in:5.1f} V f{tel.fault_code}"
                    )
                    if csv:
                        csv.write(f"{now - started:.3f},{side},{tel.erpm},"
                                  f"{tel.current_a},{tel.v_in},{tel.fault_code},"
                                  f"{int(tel.valid)},{duty}\n")

                verdict = f" ** {bump.side.upper()} BUMP **" if bump else ""
                print(f"\r{'  │  '.join(cells)}{verdict}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()

    print("\n")
    if samples and invalid == samples * 2:
        print("EVERY sample came back invalid. `valid` is 0 on any poll that did not\n"
              "get a CRC-valid reply from the ESC (vesc.h:45), so the usual cause is\n"
              "motor power off — the STM32 is fine and the FSESCs are not answering.")
        return 1

    print(f"{samples} samples, {invalid} invalid wheel readings")
    for side in ("left", "right"):
        print(f"  {side:5s} peak {peaks[side]:6.2f} A   "
              f"peak while turning {spinning[side]:6.2f} A")
    turning = max(spinning.values())
    stalled = max(peaks.values())
    if stalled > turning > 0:
        print(f"\nfree-running peak {turning:.2f} A, overall peak {stalled:.2f} A.\n"
              f"If the overall peak was a genuine jam, ORIO_BUMP_STALL_CURRENT_A\n"
              f"belongs between them — around {(turning + stalled) / 2:.1f}.")
    else:
        print("\nNo separation to report: run this once free-running and once against\n"
              "something immovable, and compare the two peaks.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
