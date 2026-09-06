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

import sys

from proto_client import (
    CMD_GET_STATUS,
    CMD_HEARTBEAT,
    arm,
    expect_ack,
    expect_no_response,
    expect_status,
    open_port,
    reset_results,
    results_exit_code,
    send_and_show,
)


def run(ser):
    arm(ser)

    print("\n--- status right after arming ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS, expect=expect_status(estopped=0))

    print("\n--- corrupted CRC, expect silent drop (no response), board stays alive ---")
    send_and_show(
        ser,
        "HEARTBEAT (bad CRC)",
        CMD_HEARTBEAT,
        corrupt_crc=True,
        expect=expect_no_response(),
    )
    send_and_show(
        ser,
        "HEARTBEAT (good, confirms still alive)",
        CMD_HEARTBEAT,
        expect=expect_ack(CMD_HEARTBEAT),
    )

    print("\n--- heartbeat watchdog: clear e-stop, then let it lapse (>500ms) ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT, expect=expect_ack(CMD_HEARTBEAT))
    time.sleep(0.6)
    # Plain sleep, not hold_alive() -- the lapse is the thing under test.
    send_and_show(
        ser,
        "GET_STATUS (watchdog should have re-tripped the e-stop)",
        CMD_GET_STATUS,
        expect=expect_status(estopped=1),
    )

    print("\n--- heartbeat re-arms the link ---")
    arm(ser)


def main():
    reset_results()
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()
    sys.exit(results_exit_code("heartbeat / e-stop"))


if __name__ == "__main__":
    main()
