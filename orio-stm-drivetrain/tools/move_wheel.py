#!/usr/bin/env python3
"""
Spins one hub motor via its FSESC, by side and direction, for a bounded
duration -- a bench test for confirming wiring and rotation direction
before trusting the drivetrain to a real drive command from the Jetson.

CMD_SET_DRIVE always commands both wheels in one frame (see
Core/Inc/protocol.h), so this holds the other side at 0 while the
requested side spins.

Safety notes:
  - The wheel is automatically stopped (CMD_STOP) after --duration seconds,
    or immediately if the script is interrupted (Ctrl+C) or crashes.
  - Independently of this script, the firmware's own heartbeat watchdog
    (PROTO_HEARTBEAT_TIMEOUT_MS, see protocol.h) stops both wheels within
    500ms of any heartbeat stopping -- e.g. if this script's process is
    killed outright rather than interrupted cleanly.
  - Make sure the wheel is off the ground / free to spin before running this.

Usage:
    pip install pyserial
    python tools/move_wheel.py <port> <left|right> <forward|reverse> [--duty PERCENT] [--duration SECONDS]

Examples:
    python tools/move_wheel.py COM5 left forward
    python tools/move_wheel.py COM5 right reverse --duty 50 --duration 3
"""
import argparse
import sys

from proto_client import (
    CMD_ACK,
    CMD_GET_STATUS,
    CMD_SET_DRIVE,
    CMD_STOP,
    WHEEL_LEFT,
    WHEEL_RIGHT,
    arm,
    open_serial,
    send_and_show,
    set_drive_payload,
)

WHEEL_NAME_TO_SIDE = {"left": WHEEL_LEFT, "right": WHEEL_RIGHT}

DEFAULT_DUTY_PERCENT = 30.0
DEFAULT_DURATION_S = 1.5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Spin one hub motor by side and direction, holding the other side stopped.",
    )
    parser.add_argument("port", help="serial port, e.g. COM5")
    parser.add_argument("side", choices=sorted(WHEEL_NAME_TO_SIDE), help="which wheel to spin")
    parser.add_argument("direction", choices=("forward", "reverse"), help="which way to spin it")
    parser.add_argument(
        "--duty",
        type=float,
        default=DEFAULT_DUTY_PERCENT,
        help=f"duty cycle percent, 0-100 (default {DEFAULT_DUTY_PERCENT:g})",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"seconds to spin before auto-stopping (default {DEFAULT_DURATION_S:g})",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not (0.0 <= args.duty <= 100.0):
        print("--duty must be between 0 and 100")
        sys.exit(1)

    side = WHEEL_NAME_TO_SIDE[args.side]
    permille = round(args.duty * 10)
    if args.direction == "reverse":
        permille = -permille

    left_permille = permille if side == WHEEL_LEFT else 0
    right_permille = permille if side == WHEEL_RIGHT else 0

    ser = open_serial(args.port)
    try:
        arm(ser)

        print(f"\n--- spinning {args.side} {args.direction} at {args.duty:g}% for {args.duration:g}s ---")
        resp = send_and_show(
            ser,
            f"SET_DRIVE {args.side}={permille}",
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
