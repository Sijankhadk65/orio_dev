#!/usr/bin/env python3
"""
ARGB-only test for the orio-stm-motion UART protocol -- cycles the fan's LED
ring through a few colors. Not e-stop gated (lighting isn't a motion-safety
concern), so this doesn't exercise CMD_STOP.

Usage:
    pip install pyserial
    python tools/test_rgb.py COM5
"""
import sys

from proto_client import (
    CMD_SET_FAN_RGB,
    arm,
    expect_ack,
    open_port,
    reset_results,
    results_exit_code,
    send_and_show,
)


def run(ser):
    arm(ser)  # not required for RGB, but keeps the board's watchdog happy if this runs long

    print("\n--- fan RGB: red, green, blue, white, then off ---")
    colors = (
        ("red", (255, 0, 0)),
        ("green", (0, 255, 0)),
        ("blue", (0, 0, 255)),
        ("white", (255, 255, 255)),
        ("off", (0, 0, 0)),
    )
    for name, rgb in colors:
        send_and_show(
            ser,
            f"SET_FAN_RGB {name}",
            CMD_SET_FAN_RGB,
            bytes(rgb),
            expect=expect_ack(CMD_SET_FAN_RGB),
        )


def main():
    reset_results()
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()
    sys.exit(results_exit_code("RGB"))


if __name__ == "__main__":
    main()
