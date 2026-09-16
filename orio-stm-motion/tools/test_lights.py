#!/usr/bin/env python3
"""
Lights-only test for the orio-stm-motion UART protocol -- walks each of the
four chassis lights on its own, then a few combinations, then checks the two
ways a bad CMD_SET_LIGHTS frame must be rejected.

Nothing reads the lights back: CMD_GET_STATUS carries the e-stop flag and the
joint angles, not the light mask. So the ACKs prove the firmware accepted each
frame, and YOUR EYES prove the right lamp lit. Watch the hardware while this
runs; a clean pass with a mis-wired transistor looks identical from here.

Deliberately exercises CMD_STOP: unlike the fan, the lights are NOT e-stop
gated (see CMD_SET_LIGHTS in Core/Inc/protocol.h -- cutting the lights the
instant the link drops would blind the cameras). That is a design decision
worth a regression check, so this asserts the lights still answer while
e-stopped, then re-arms.

Doubles as the pin-finder. The per-light walk pauses SETTLE_S between lamps,
which is long enough to sweep a Morpho header with a multimeter in DC volts
and see which pad went to 3.3 V -- that is how the PB4..PB7 assignments in
Core/Src/lights.c get confirmed against the physical board. Raise SETTLE_S if
you want longer to probe.

Usage:
    pip install pyserial
    python tools/test_lights.py COM5
"""
import sys

from proto_client import (
    CMD_HEARTBEAT,
    CMD_SET_LIGHTS,
    CMD_STOP,
    LIGHT_CHEST,
    LIGHT_DRL_LEFT,
    LIGHT_DRL_RIGHT,
    LIGHT_NAMES,
    LIGHT_TRAILER,
    LIGHTS_ALL_MASK,
    NACK_BAD_LENGTH,
    NACK_OUT_OF_RANGE,
    arm,
    describe_mask,
    expect_ack,
    expect_nack,
    lights_mask,
    open_port,
    reset_results,
    results_exit_code,
    send_and_show,
)

# Long enough to find the lit lamp -- or to find the live pad with a meter,
# which is the other job this walk does.
SETTLE_S = 2.0


def set_lights(ser, mask, expect=None, delay_s=None, keep_alive=True):
    """Sends one CMD_SET_LIGHTS frame, labelled by the lamps it should light."""
    return send_and_show(
        ser,
        f"SET_LIGHTS {mask:#04x} ({describe_mask(mask)})",
        CMD_SET_LIGHTS,
        bytes([mask]),
        expect=expect,
        delay_s=SETTLE_S if delay_s is None else delay_s,
        keep_alive=keep_alive,
    )


def run(ser):
    arm(ser)  # not required -- lights aren't gated -- but keeps the watchdog quiet

    print("\n--- one lamp at a time (watch the hardware, or probe the header) ---")
    for light in (LIGHT_TRAILER, LIGHT_DRL_LEFT, LIGHT_DRL_RIGHT, LIGHT_CHEST):
        set_lights(ser, lights_mask(light), expect=expect_ack(CMD_SET_LIGHTS))
        print(f"     ^ only {LIGHT_NAMES[light]} should be lit")

    print("\n--- combinations ---")
    for ids in (
        (LIGHT_DRL_LEFT, LIGHT_DRL_RIGHT),
        (LIGHT_TRAILER, LIGHT_DRL_LEFT, LIGHT_DRL_RIGHT),
        (LIGHT_TRAILER, LIGHT_DRL_LEFT, LIGHT_DRL_RIGHT, LIGHT_CHEST),
    ):
        set_lights(ser, lights_mask(*ids), expect=expect_ack(CMD_SET_LIGHTS))

    set_lights(ser, lights_mask(), expect=expect_ack(CMD_SET_LIGHTS))

    print("\n--- rejects a mask with bits above LIGHTS_ALL_MASK ---")
    # Rejected outright rather than masked down to 0x0F, so a client built
    # against a board that grew a fifth light finds out instead of silently
    # lighting four.
    send_and_show(
        ser,
        f"SET_LIGHTS {LIGHTS_ALL_MASK + 1:#04x} (undefined light)",
        CMD_SET_LIGHTS,
        bytes([LIGHTS_ALL_MASK + 1]),
        expect=expect_nack(NACK_OUT_OF_RANGE, CMD_SET_LIGHTS),
    )

    print("\n--- rejects a wrong-length payload ---")
    send_and_show(
        ser,
        "SET_LIGHTS with 2-byte payload",
        CMD_SET_LIGHTS,
        bytes([0x01, 0x00]),
        expect=expect_nack(NACK_BAD_LENGTH, CMD_SET_LIGHTS),
    )

    print("\n--- still answers while e-stopped (lighting is not motion safety) ---")
    # keep_alive=False throughout: a heartbeat would clear the e-stop and the
    # check would pass for the wrong reason.
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False, expect=expect_ack(CMD_STOP))
    set_lights(
        ser,
        lights_mask(LIGHT_DRL_LEFT, LIGHT_DRL_RIGHT),
        expect=expect_ack(CMD_SET_LIGHTS),
        keep_alive=False,
    )

    print("\n--- cleanup: re-arm, lights off ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT, expect=expect_ack(CMD_HEARTBEAT))
    set_lights(ser, lights_mask(), expect=expect_ack(CMD_SET_LIGHTS), delay_s=0.2)


def main():
    reset_results()
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()
    sys.exit(results_exit_code("LIGHTS"))


if __name__ == "__main__":
    main()
