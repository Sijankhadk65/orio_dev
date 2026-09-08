#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial"]
# ///
"""
Identity-handshake test for the orio-stm-drivetrain UART protocol: proves the
board on a port says what it is, says it while e-stopped, and that saying it
does not keep the board alive.

Why this exists: both STM32 Nucleos present their ST-LINK VCP as USB
0483:374b, so both land on /dev/ttyACM* in enumeration order and the udev
symlinks are the only thing naming them apart. A symlink cannot catch a board
swapped for one with a different ST-LINK serial, nor drivetrain firmware
flashed onto the motion board. Both boards share this framing and CRC, so a
CMD_SET_DRIVE that reaches the motion board passes CRC and decodes as a valid
frame of the wrong kind -- servos slew to whatever a wheel duty pair means,
with nothing on the console. Identity has to come off the wire.

Worth running against the motion board on purpose: it must FAIL with
role=motion rather than pass quietly. That is the failure this handshake
exists to make loud.

Usage:
    pip install pyserial
    python tools/test_whoami.py COM5
"""
import sys
import time

from proto_client import (
    CMD_GET_STATUS,
    CMD_STATUS,
    CMD_WHOAMI,
    EXPECTED_PROTO_VERSION,
    EXPECTED_ROLE,
    ROLE_NAMES,
    arm,
    build_frame,
    open_port,
    parse_status_payload,
    read_frame,
    send_and_show,
    whoami,
)

# Longer than the firmware's PROTO_HEARTBEAT_TIMEOUT_MS (500ms), so the
# watchdog has had a clear chance to trip by the end of the WHOAMI flood.
WHOAMI_FLOOD_S = 0.8
WHOAMI_FLOOD_INTERVAL_S = 0.1

_FAILURES = []


def check(ok: bool, label: str, detail: str = ""):
    """Records a check instead of raising, so one run reports every failure and
    still runs the cleanup at the tail of the script."""
    print(f"   {'PASS' if ok else 'FAIL'}  {label}" + ("" if ok or not detail else f": {detail}"))
    if not ok:
        _FAILURES.append((label, detail))


def read_estopped(ser, label: str):
    """Returns the board's e-stop flag from a fresh CMD_STATUS, or None."""
    resp = send_and_show(ser, label, CMD_GET_STATUS, keep_alive=False, delay_s=0.0)
    if resp is None or resp[0] != CMD_STATUS:
        return None
    estopped, _ = parse_status_payload(resp[1])
    return bool(estopped)


def run(ser):
    print("--- WHOAMI answers while e-stopped, before anything is armed ---")
    # Deliberately no arm() first. The board boots e-stopped, the Jetson asks
    # identity before its first heartbeat, and a handler that gated the reply on
    # the e-stop would deadlock that startup order -- so an unarmed board that
    # stays silent here is a real bug, not a test artefact.
    ident = whoami(ser)
    if ident is None:
        check(False, "WHOAMI answers while e-stopped", "no CMD_IDENTITY reply")
    else:
        check(True, "WHOAMI answers while e-stopped")
        check(
            ident["role"] == EXPECTED_ROLE,
            f"role is {ROLE_NAMES[EXPECTED_ROLE]}",
            f"got {ident['role_name']} (0x{ident['role']:02x}), firmware "
            f"{ident['firmware']} -- this port is the WRONG BOARD",
        )
        check(
            ident["proto_version"] == EXPECTED_PROTO_VERSION,
            f"protocol version is v{EXPECTED_PROTO_VERSION}",
            f"got v{ident['proto_version']}",
        )

    print("\n--- confirm the board really was e-stopped while it answered ---")
    estopped = read_estopped(ser, "GET_STATUS")
    check(
        estopped is True,
        "board was e-stopped during the WHOAMI",
        "no STATUS reply" if estopped is None else f"estopped={int(estopped)}",
    )

    print("\n--- WHOAMI must not feed the heartbeat watchdog ---")
    arm(ser)
    # Nothing but WHOAMI from here, for longer than the watchdog window. If the
    # handler touched s_last_heartbeat_tick the board would still be armed at
    # the end of it, and a controller polling identity could hold the wheels
    # live without ever proving the link healthy.
    end = time.monotonic() + WHOAMI_FLOOD_S
    floods = 0
    while time.monotonic() < end:
        ser.write(build_frame(CMD_WHOAMI))
        read_frame(ser, timeout_s=0.2)
        floods += 1
        time.sleep(WHOAMI_FLOOD_INTERVAL_S)
    print(f"    sent {floods} WHOAMI frames over {WHOAMI_FLOOD_S:g}s, no heartbeats")

    estopped = read_estopped(ser, "GET_STATUS")
    check(
        estopped is True,
        "watchdog still trips despite WHOAMI traffic",
        "no STATUS reply"
        if estopped is None
        else f"estopped={int(estopped)} -- WHOAMI is feeding the watchdog",
    )


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()

    print()
    print("=" * 68)
    if _FAILURES:
        print(f"identity handshake: {len(_FAILURES)} check(s) FAILED")
        for label, detail in _FAILURES:
            print(f"  FAIL  {label}: {detail}")
    else:
        print("identity handshake: all checks passed")
    print("=" * 68)
    sys.exit(1 if _FAILURES else 0)


if __name__ == "__main__":
    main()
