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

    # hold a duty while watching — the calibration run. PERCENT, as everywhere
    # else a human types a duty, so this is the speed the robot actually drives
    uv run python tools/wheel_telemetry.py --duty 5

    # log it for later
    uv run python tools/wheel_telemetry.py --duty 5 --csv /tmp/stall.csv

## Safety

Stalling a hub motor at duty is a thermal event on the ESC: sub-ohm windings,
no back-EMF, current pinned at whatever the FSESC limit is. Keep a jam to a
couple of seconds, give the controllers airflow, and touch-check between runs.
`--seconds` defaults to 20 for that reason, and `--duty` is capped at the
robot's own ceiling (`ORIO_DRIVE_SPEED_MAX_PERCENT`) rather than at some limit
this tool invented for itself.

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

# PERCENT, like `tools/teleop_guarded.py --duty` and like `set_speed`. This
# flag was per-mille at first, so `--duty 50` here and `--duty 5` there meant
# the same thing and the difference was invisible at the prompt. The board
# takes per-mille and the conversion belongs in one place — just above
# `set_drive`, not in the operator's head.
#
# The cap is the robot's ceiling, not this tool's own idea of safe.
DUTY_CAP_PERCENT = config.DRIVE_SPEED_MAX_PERCENT


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Live wheel telemetry, for calibrating the bump thresholds.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default=config.DRIVETRAIN_PORT,
                   help="drivetrain board — a udev symlink, never a raw ttyACM*")
    p.add_argument("--duty", type=float, default=0.0,
                   help=f"duty PERCENT to hold on BOTH wheels while watching, the "
                        f"same units as teleop_guarded. Capped at "
                        f"+/-{DUTY_CAP_PERCENT:g}%% — the robot's ceiling "
                        f"(ORIO_DRIVE_SPEED_MAX_PERCENT), not this tool's. "
                        f"0 just watches")
    p.add_argument("--seconds", type=float, default=20.0,
                   help="stop after this long. Short by default because a stall is "
                        "a thermal event on the ESC")
    p.add_argument("--hz", type=float, default=10.0, help="sample rate")
    p.add_argument("--csv", default=None, help="also append samples to this file")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    duty_percent = max(-DUTY_CAP_PERCENT, min(DUTY_CAP_PERCENT, args.duty))
    if duty_percent != args.duty:
        print(f"duty clamped to {duty_percent:g}% (the robot's ceiling is "
              f"+/-{DUTY_CAP_PERCENT:g}%)")
    # The one conversion, here and nowhere else: the board speaks per-mille.
    duty = round(duty_percent * 10)

    detector = StallDetector()
    period = 1.0 / max(args.hz, 0.1)
    peaks = {"left": 0.0, "right": 0.0}
    spinning = {"left": 0.0, "right": 0.0}  # peak current while actually turning
    samples = 0
    no_status = 0      # the board sent no STATUS frame at all
    invalid_wheels = 0  # a STATUS frame arrived, but a wheel in it was not valid
    csv = open(args.csv, "a") if args.csv else None
    if csv:
        csv.write("t,side,erpm,current_a,v_in,fault,valid,duty\n")

    print(f"thresholds in force: current >= {config.BUMP_STALL_CURRENT_A} A, "
          f"|erpm| <= {config.BUMP_STALL_ERPM}, duty >= {config.BUMP_MIN_DUTY}, "
          f"held {config.BUMP_CONFIRM_S}s")
    if duty:
        print(f"HOLDING {duty_percent:g}% ({duty} per-mille) on both wheels for "
              f"{args.seconds:g}s — Ctrl-C stops everything")
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
            last_sent = None
            while (time.monotonic() - started) < args.seconds:
                now = time.monotonic()
                # ONCE, not every tick. `handle_set_drive` calls `Vesc_SetDuty`
                # on both ESCs synchronously, each a blocking HAL_UART_Transmit
                # with a 10 ms timeout, so one SET_DRIVE frame occupies the
                # board's frame loop for ~20 ms. Re-sending it at 10 Hz and
                # asking for STATUS in the same breath means the STATUS request
                # lands mid-transaction and is dropped — the run comes back
                # "no status" from end to end while the wheels turn happily.
                #
                # There is no need to repeat it either: the firmware holds the
                # commanded speed and keeps re-sending it to the ESCs itself.
                # Every other caller in the tree sends on change only; this one
                # did not, and it is the only one that ever saw the failure.
                if duty and last_sent != (duty, duty):
                    link.set_drive(duty, duty)
                    last_sent = (duty, duty)
                    time.sleep(0.05)  # let both ESC transactions finish
                link.request_status()
                time.sleep(period)

                # Rejections are silent unless read (see drivetrain.py's module
                # docstring): a SET_DRIVE sent while e-stopped is NACKed and
                # otherwise ignored, so the wheels just do not move.
                rejection = link.take_rejection()
                if rejection is not None:
                    print(f"\nBOARD REJECTED: {rejection}")
                    last_sent = None  # say it again rather than dedupe it away

                status = link.last_status
                samples += 1
                bump = detector.update(time.monotonic(), (duty, duty), status)
                if status is None:
                    no_status += 1
                    print("\rno STATUS frame from the board                       ",
                          end="", flush=True)
                    continue

                cells = []
                for side in ("left", "right"):
                    tel = status.wheels.get(side)
                    if tel is None or not tel.valid:
                        invalid_wheels += 1
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
    if samples and no_status == samples:
        print("The board sent no STATUS frame at all, for the whole run.\n"
              "  That is the LINK, not the motors: with --duty 0 the same board\n"
              "  answers every poll. Suspect something occupying the firmware's\n"
              "  frame loop — SET_DRIVE blocks it for ~20 ms talking to both ESCs —\n"
              "  or a second process holding the port.")
        return 1
    if samples and invalid_wheels == samples * 2:
        print("Every wheel reading came back invalid, though the board was answering.\n"
              "  `valid` is 0 on any poll that did not get a CRC-valid reply from the\n"
              "  ESC (vesc.h:45), so the usual cause is motor power off — the STM32 is\n"
              "  fine and the FSESCs are not.")
        return 1

    print(f"{samples} samples, {no_status} with no STATUS frame, "
          f"{invalid_wheels} invalid wheel readings")
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
