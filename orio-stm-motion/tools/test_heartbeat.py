#!/usr/bin/env python3
"""
Link/e-stop-only test for the orio-stm-motion UART protocol -- heartbeat
handshake, malformed-frame handling, and the heartbeat watchdog timeout.
Doesn't touch servos, fan, or RGB.

Usage:
    pip install pyserial
    python tools/test_heartbeat.py COM5
"""
import time

from proto_client import (
    CMD_GET_STATUS,
    CMD_HEARTBEAT,
    arm,
    open_port,
    send_and_show,
)


def run(ser):
    arm(ser)

    print("\n--- status right after arming ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- corrupted CRC, expect silent drop (no response), board stays alive ---")
    send_and_show(ser, "HEARTBEAT (bad CRC)", CMD_HEARTBEAT, corrupt_crc=True)
    send_and_show(ser, "HEARTBEAT (good, confirms still alive)", CMD_HEARTBEAT)

    print("\n--- heartbeat watchdog: clear e-stop, then let it lapse (>500ms) ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT)
    time.sleep(0.6)
    send_and_show(ser, "GET_STATUS (expect estopped=1)", CMD_GET_STATUS)

    print("\n--- heartbeat re-arms the link ---")
    arm(ser)


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
