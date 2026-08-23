#!/usr/bin/env python3
"""
ARGB-only test for the orio-stm-motion UART protocol -- cycles the fan's LED
ring through a few colors. Not e-stop gated (lighting isn't a motion-safety
concern), so this doesn't exercise CMD_STOP.

Usage:
    pip install pyserial
    python tools/test_rgb.py COM5
"""
from proto_client import CMD_SET_FAN_RGB, arm, open_port, send_and_show


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
        send_and_show(ser, f"SET_FAN_RGB {name}", CMD_SET_FAN_RGB, bytes(rgb))


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
