#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial"]
# ///
"""
Spins both hub motors together in one CMD_SET_DRIVE frame -- for testing
differential-drive behavior (straight line, in-place turn) once each wheel
has already been confirmed working individually with move_wheel.py.

Safety notes: same as move_wheel.py -- auto-stops after --duration seconds,
on Ctrl+C, or on crash; the firmware's own heartbeat watchdog independently
stops both wheels within 500ms if this script's process is killed outright.
Make sure both wheels are off the ground / free to spin first.

Usage:
    pip install pyserial
    python tools/move_both.py <port> <left: forward|reverse|stop> <right: forward|reverse|stop> [--duty PERCENT] [--duration SECONDS]

Examples:
    python tools/move_both.py COM5 forward forward            # straight line
    python tools/move_both.py COM5 forward reverse             # turn in place
    python tools/move_both.py COM5 forward stop --duty 40
"""
import argparse
import sys

from proto_client import (
    CMD_ACK,
    CMD_GET_STATUS,
    CMD_SET_DRIVE,
    CMD_STOP,
    arm,
    open_serial,
    send_and_show,
    set_drive_payload,
)

DEFAULT_DUTY_PERCENT = 30.0
DEFAULT_DURATION_S = 1.5

DIRECTION_CHOICES = ("forward", "reverse", "stop")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Spin both hub motors together in one frame -- for differential-drive tests.",
    )
    parser.add_argument("port", help="serial port, e.g. COM5")
    parser.add_argument("left", choices=DIRECTION_CHOICES, help="left wheel direction")
    parser.add_argument("right", choices=DIRECTION_CHOICES, help="right wheel direction")
    parser.add_argument(
        "--duty",
        type=float,
        default=DEFAULT_DUTY_PERCENT,
        help=f"duty cycle percent, 0-100, applied to whichever side isn't 'stop' (default {DEFAULT_DUTY_PERCENT:g})",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"seconds to spin before auto-stopping (default {DEFAULT_DURATION_S:g})",
    )
    return parser.parse_args()


def direction_to_permille(direction: str, duty_percent: float) -> int:
    if direction == "stop":
        return 0
    permille = round(duty_percent * 10)
    return -permille if direction == "reverse" else permille


def main():
    args = parse_args()
    if not (0.0 <= args.duty <= 100.0):
        print("--duty must be between 0 and 100")
        sys.exit(1)

    left_permille = direction_to_permille(args.left, args.duty)
    right_permille = direction_to_permille(args.right, args.duty)

    ser = open_serial(args.port)
    try:
        arm(ser)

        print(f"\n--- left={args.left} right={args.right} at {args.duty:g}% for {args.duration:g}s ---")
        resp = send_and_show(
            ser,
            f"SET_DRIVE left={left_permille} right={right_permille}",
            CMD_SET_DRIVE,
            set_drive_payload(left_permille, right_permille),
            delay_s=args.duration,
        )
        if resp is not None and resp[0] != CMD_ACK:
            print("command was rejected -- see reason above (likely out of range or e-stopped)")
            sys.exit(1)

        print("\n--- status while spinning ---")
        send_and_show(ser, "GET_STATUS", CMD_GET_STATUS, delay_s=0.0)
    finally:
        print("\n--- stopping ---")
        send_and_show(ser, "STOP", CMD_STOP, keep_alive=False, delay_s=0.2)
        ser.close()


if __name__ == "__main__":
    main()
