#!/usr/bin/env python3
"""
Identity-handshake test for the orio-stm-motion UART protocol: proves the board
on a port says what it is, says it while e-stopped, and that saying it does not
keep the board alive. Doesn't touch servos, fan, or RGB.

Why this exists: both STM32 Nucleos present their ST-LINK VCP as USB
0483:374b, so both land on /dev/ttyACM* in enumeration order and the udev
symlinks are the only thing naming them apart. A symlink cannot catch a board
swapped for one with a different ST-LINK serial, nor motion firmware flashed
onto the drivetrain board. Both boards share this framing and CRC, so a
CMD_SET_DRIVE meant for the wheels that reaches this board passes CRC and
decodes as a valid frame of the wrong kind -- the servos slew to whatever a
wheel duty pair means, with nothing on the console. Identity has to come off
the wire.

Worth running against the drivetrain board on purpose: it must FAIL with
role=drivetrain rather than pass quietly. That is the failure this handshake
exists to make loud.

Usage:
    pip install pyserial
    python tools/test_whoami.py COM5
"""
import sys
import time

from proto_client import (
    CMD_GET_STATUS,
    CMD_STOP,
    CMD_WHOAMI,
    EXPECTED_PROTO_VERSION,
    EXPECTED_ROLE,
    arm,
    build_frame,
    expect_identity,
    expect_status,
    open_port,
    read_frame,
    reset_results,
    results_exit_code,
    send_and_show,
)

# Longer than the firmware's PROTO_HEARTBEAT_TIMEOUT_MS (500ms), so the
# watchdog has had a clear chance to trip by the end of the WHOAMI flood.
WHOAMI_FLOOD_S = 0.8
WHOAMI_FLOOD_INTERVAL_S = 0.1


def run(ser):
    # Force the e-stop on rather than assuming it: this module also runs inside
    # test_protocol.py, after subsystems that leave the board armed.
    print("--- e-stop the board, so the next reply has to come from an e-stopped board ---")
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False, delay_s=0.2)
    send_and_show(
        ser,
        "GET_STATUS (precondition: e-stopped)",
        CMD_GET_STATUS,
        keep_alive=False,
        delay_s=0.0,
        expect=expect_status(estopped=1),
    )

    print("\n--- WHOAMI answers while e-stopped, and names THIS board ---")
    # Not gated on the e-stop, deliberately: the Jetson asks identity before its
    # first heartbeat, so a handler that required an armed link would deadlock
    # startup. It must also reply with CMD_IDENTITY, not an ACK -- an ACK proves
    # the link works while saying nothing about which board is on it.
    send_and_show(
        ser,
        "WHOAMI (e-stopped)",
        CMD_WHOAMI,
        keep_alive=False,
        delay_s=0.0,
        expect=expect_identity(role=EXPECTED_ROLE, proto_version=EXPECTED_PROTO_VERSION),
    )

    print("\n--- WHOAMI must not feed the heartbeat watchdog ---")
    arm(ser)
    send_and_show(
        ser,
        "GET_STATUS (armed)",
        CMD_GET_STATUS,
        keep_alive=False,
        delay_s=0.0,
        expect=expect_status(estopped=0),
    )
    # Nothing but WHOAMI from here, for longer than the watchdog window. If the
    # handler touched s_last_heartbeat_tick the board would still be armed at
    # the end of it, and a controller polling identity could hold the servos
    # live without ever proving the link healthy. Sent raw rather than through
    # send_and_show(expect=...) so the flood contributes one check, not a dozen
    # identical ones.
    end = time.monotonic() + WHOAMI_FLOOD_S
    floods = 0
    while time.monotonic() < end:
        ser.write(build_frame(CMD_WHOAMI))
        read_frame(ser, timeout_s=0.2)
        floods += 1
        time.sleep(WHOAMI_FLOOD_INTERVAL_S)
    print(f"    sent {floods} WHOAMI frames over {WHOAMI_FLOOD_S:g}s, no heartbeats")

    send_and_show(
        ser,
        "GET_STATUS (watchdog should have re-tripped despite the WHOAMI traffic)",
        CMD_GET_STATUS,
        keep_alive=False,
        delay_s=0.0,
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
    sys.exit(results_exit_code("identity handshake"))


if __name__ == "__main__":
    main()
