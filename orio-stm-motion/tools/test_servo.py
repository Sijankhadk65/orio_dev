#!/usr/bin/env python3
"""
Servo/joint-only test for the orio-stm-motion UART protocol -- movement,
range validation, reset pose, and e-stop interaction. Only the neck has
hardware wired up today; left-arm/right-arm exercise the latch-only path.

Every angle below is derived from JOINT_LIMITS_DEG / JOINT_HOME_DEG in
proto_client.py rather than written as a literal, so a bracket's window
moving re-aims this script instead of leaving it asserting against a
window the firmware no longer has. Update those tables, not this file.

Usage:
    pip install pyserial
    python tools/test_servo.py COM5
"""
import sys

from proto_client import (
    CMD_GET_STATUS,
    CMD_MOVE_JOINT_TO,
    CMD_RESET_JOINTS,
    CMD_STOP,
    JOINT_HOME_DEG,
    JOINT_POS_LEFT_ARM,
    JOINT_POS_NAMES,
    JOINT_POS_NECK,
    JOINT_POS_RIGHT_ARM,
    MOVE_DELAY_S,
    NACK_ESTOPPED,
    NACK_OUT_OF_RANGE,
    arm,
    axis_just_above,
    axis_just_below,
    axis_limit,
    axis_mid,
    expect_ack,
    expect_nack,
    expect_status,
    move_joint_payload,
    open_port,
    reset_results,
    results_exit_code,
    send_and_show,
)


def move(ser, position, description, pan_deg, tilt_deg, expect, delay_s=None):
    """Sends one CMD_MOVE_JOINT_TO and checks the firmware's answer against
    expect (an expect_*() object from proto_client)."""
    joint = JOINT_POS_NAMES[position]
    print(f"\n--- {description} ---")
    send_and_show(
        ser,
        f"MOVE_JOINT_TO {joint} pan={pan_deg:g} tilt={tilt_deg:g}",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(position, pan_deg, tilt_deg),
        expect=expect,
        **({} if delay_s is None else {"delay_s": delay_s}),
    )


def run(ser):
    arm(ser)

    neck_pan_lo, neck_pan_hi = axis_limit(JOINT_POS_NECK, "pan")
    neck_tilt_lo, neck_tilt_hi = axis_limit(JOINT_POS_NECK, "tilt")
    neck_pan_mid = axis_mid(JOINT_POS_NECK, "pan")
    neck_tilt_mid = axis_mid(JOINT_POS_NECK, "tilt")

    print("\n--- status before any move ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS, expect=expect_status(estopped=0))

    print(
        f"\n(neck windows: pan {neck_pan_lo:g}..{neck_pan_hi:g}, "
        f"tilt {neck_tilt_lo:g}..{neck_tilt_hi:g})"
    )

    # Both axes inside their window, both away from the midpoint the joint
    # is already parked at, so the servos visibly travel.
    move(
        ser,
        JOINT_POS_NECK,
        "move both neck axes in one frame",
        pan_deg=neck_pan_mid - 30.0,
        tilt_deg=neck_tilt_lo,
        expect=expect_ack(CMD_MOVE_JOINT_TO),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- status after the move ---")
    send_and_show(
        ser,
        "GET_STATUS",
        CMD_GET_STATUS,
        # CMD_GET_STATUS reports the angles the firmware ACCEPTED, which are
        # latched immediately -- not where the servos have physically
        # travelled to, since the joint ramps there over MOVE_DELAY_S.
        expect=expect_status(
            estopped=0,
            joints={JOINT_POS_NECK: (neck_pan_mid - 30.0, neck_tilt_lo)},
        ),
    )

    # Pan hard against each end of its window; tilt held at its midpoint so
    # only pan is under test.
    move(
        ser,
        JOINT_POS_NECK,
        "neck pan at its ceiling, tilt held mid-window",
        pan_deg=neck_pan_hi,
        tilt_deg=neck_tilt_mid,
        expect=expect_ack(CMD_MOVE_JOINT_TO),
        delay_s=MOVE_DELAY_S,
    )
    move(
        ser,
        JOINT_POS_NECK,
        "neck pan at its floor, tilt held mid-window",
        pan_deg=neck_pan_lo,
        tilt_deg=neck_tilt_mid,
        expect=expect_ack(CMD_MOVE_JOINT_TO),
        delay_s=MOVE_DELAY_S,
    )

    # Tilt hard against each end of its window. The floor is already covered
    # by the first move above; without this ceiling case a window that
    # silently NARROWS still passes every test, because the out-of-range
    # cases below stay out of range and keep being rejected.
    move(
        ser,
        JOINT_POS_NECK,
        "neck tilt at its ceiling, pan held mid-window",
        pan_deg=neck_pan_mid,
        tilt_deg=neck_tilt_hi,
        expect=expect_ack(CMD_MOVE_JOINT_TO),
        delay_s=MOVE_DELAY_S,
    )

    # One axis out of range at a time, the other held legal, so the NACK can
    # only be attributed to the axis under test.
    move(
        ser,
        JOINT_POS_NECK,
        "neck pan above its ceiling",
        pan_deg=axis_just_above(JOINT_POS_NECK, "pan"),
        tilt_deg=neck_tilt_mid,
        expect=expect_nack(NACK_OUT_OF_RANGE, CMD_MOVE_JOINT_TO),
    )
    move(
        ser,
        JOINT_POS_NECK,
        "neck pan below its floor",
        pan_deg=axis_just_below(JOINT_POS_NECK, "pan"),
        tilt_deg=neck_tilt_mid,
        expect=expect_nack(NACK_OUT_OF_RANGE, CMD_MOVE_JOINT_TO),
    )
    move(
        ser,
        JOINT_POS_NECK,
        "neck tilt above its ceiling",
        pan_deg=neck_pan_mid,
        tilt_deg=axis_just_above(JOINT_POS_NECK, "tilt"),
        expect=expect_nack(NACK_OUT_OF_RANGE, CMD_MOVE_JOINT_TO),
    )
    move(
        ser,
        JOINT_POS_NECK,
        "neck tilt below its floor",
        pan_deg=neck_pan_mid,
        tilt_deg=axis_just_below(JOINT_POS_NECK, "tilt"),
        expect=expect_nack(NACK_OUT_OF_RANGE, CMD_MOVE_JOINT_TO),
    )

    # The arms carry a wider tilt window than the neck, so an angle legal
    # here would be rejected on the neck -- that difference is the point of
    # per-joint limits and is worth exercising rather than assuming.
    arm_tilt_lo, arm_tilt_hi = axis_limit(JOINT_POS_LEFT_ARM, "tilt")
    print(
        f"\n(left-arm tilt window is {arm_tilt_lo:g}..{arm_tilt_hi:g}, "
        f"wider than the neck's {neck_tilt_lo:g}..{neck_tilt_hi:g})"
    )
    move(
        ser,
        JOINT_POS_LEFT_ARM,
        "left-arm tilt at its floor (below the neck's floor, legal here)",
        pan_deg=axis_mid(JOINT_POS_LEFT_ARM, "pan"),
        tilt_deg=arm_tilt_lo,
        expect=expect_ack(CMD_MOVE_JOINT_TO),  # latched only -- no hardware bound
        delay_s=MOVE_DELAY_S,
    )

    neck_home = JOINT_HOME_DEG[JOINT_POS_NECK]
    print("\n--- reset every joint to its home pose in one command ---")
    print(
        f"    (neck home = pan {neck_home['pan']:+.2f}, tilt {neck_home['tilt']:+.2f} "
        f"on the command scale)"
    )
    send_and_show(
        ser,
        "RESET_JOINTS",
        CMD_RESET_JOINTS,
        delay_s=MOVE_DELAY_S,
        expect=expect_ack(CMD_RESET_JOINTS),
    )

    print(
        f"\n--- status after reset: neck should read "
        f"pan={neck_home['pan']:g}, tilt={neck_home['tilt']:g} ---"
    )
    send_and_show(
        ser,
        "GET_STATUS",
        CMD_GET_STATUS,
        expect=expect_status(
            estopped=0,
            joints={
                pos: (home["pan"], home["tilt"])
                for pos, home in JOINT_HOME_DEG.items()
            },
        ),
    )

    print("\n--- stop, then move/reset should be rejected as ESTOPPED ---")
    print("    (no keep-alive heartbeats in this block -- one would clear the e-stop we're testing for)")
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False, expect=expect_ack(CMD_STOP))
    # handle_move_joint_to() checks range BEFORE the e-stop flag, so this
    # move must be in range on BOTH axes or it comes back OUT_OF_RANGE and
    # proves nothing about the e-stop. Using each window's midpoint keeps
    # that true no matter how the windows move.
    print("\n--- move while e-stopped (both axes mid-window): expect NACK ESTOPPED ---")
    send_and_show(
        ser,
        f"MOVE_JOINT_TO neck pan={neck_pan_mid:g} tilt={neck_tilt_mid:g}",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, neck_pan_mid, neck_tilt_mid),
        keep_alive=False,
        expect=expect_nack(NACK_ESTOPPED, CMD_MOVE_JOINT_TO),
    )
    print("\n--- reset while e-stopped: expect NACK ESTOPPED ---")
    send_and_show(
        ser,
        "RESET_JOINTS",
        CMD_RESET_JOINTS,
        keep_alive=False,
        expect=expect_nack(NACK_ESTOPPED, CMD_RESET_JOINTS),
    )

    print("\n--- heartbeat clears e-stop, reset once more to leave the neck home ---")
    arm(ser)
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, delay_s=MOVE_DELAY_S)


def main():
    reset_results()
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()
    sys.exit(results_exit_code("servo / joints"))


if __name__ == "__main__":
    main()
