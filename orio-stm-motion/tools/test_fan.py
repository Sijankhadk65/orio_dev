#!/usr/bin/env python3
"""
Fan-only test for the orio-stm-motion UART protocol -- speed ramp, range
validation, and e-stop interaction. Leaves the fan at 0% when done.

Usage:
    pip install pyserial
    python tools/test_fan.py COM5
"""

from proto_client import (
    CMD_SET_FAN_SPEED,
    CMD_STOP,
    arm,
    open_port,
    send_and_show,
)


def run(ser):
    arm(ser)

    print("\n--- fan: ramp speed 0 -> 50 -> 100% ---")
    for pct in (0, 50, 100):
        send_and_show(ser, f"SET_FAN_SPEED {pct}%", CMD_SET_FAN_SPEED, bytes([pct]))

    print("\n--- fan: speed out of range (101%), expect NACK OUT_OF_RANGE ---")
    send_and_show(ser, "SET_FAN_SPEED 101%", CMD_SET_FAN_SPEED, bytes([101]))

    print("\n--- stop, then fan speed should be rejected as ESTOPPED ---")
    print(
        "    (no keep-alive heartbeats in this block -- one would clear the e-stop we're testing for)"
    )
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False)
    send_and_show(
        ser, "SET_FAN_SPEED 50%", CMD_SET_FAN_SPEED, bytes([50]), keep_alive=False
    )

    print(
        "\n--- heartbeat clears e-stop, fan should resume at its last *accepted* speed (100% -- the 50% above was rejected) ---"
    )
    arm(ser)

    print("\n--- fan: back to 0% (cleanup) ---")
    send_and_show(ser, "SET_FAN_SPEED 0%", CMD_SET_FAN_SPEED, bytes([0]))


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
