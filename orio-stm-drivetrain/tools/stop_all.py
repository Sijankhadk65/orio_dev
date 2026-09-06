#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial"]
# ///
"""
Immediately e-stops both wheels -- no arming needed, CMD_STOP works
regardless of the link's current e-stop state. Useful as a manual kill
switch while bench-testing, or to confirm the link is alive at all.

Usage:
    pip install pyserial
    python tools/stop_all.py COM5
"""
from proto_client import CMD_GET_STATUS, CMD_STOP, open_port, send_and_show


def run(ser):
    print("--- stopping both wheels ---")
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False, delay_s=0.2)

    print("\n--- status after stop ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS, keep_alive=False, delay_s=0.0)


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
