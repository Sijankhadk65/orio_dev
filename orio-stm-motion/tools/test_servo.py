#!/usr/bin/env python3
"""
Servo/joint-only test for the orio-stm-motion UART protocol -- movement,
range validation, reset pose, and e-stop interaction. Only the neck has
hardware wired up today; left-arm/right-arm exercise the latch-only path.

Usage:
    pip install pyserial
    python tools/test_servo.py COM5
"""
from proto_client import (
    CMD_GET_STATUS,
    CMD_MOVE_JOINT_TO,
    CMD_RESET_JOINTS,
    CMD_STOP,
    JOINT_POS_LEFT_ARM,
    JOINT_POS_NECK,
    MOVE_DELAY_S,
    arm,
    move_joint_payload,
    open_port,
    send_and_show,
)


def run(ser):
    arm(ser)

    print("\n--- status before any move ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- move the neck to pan=30deg, tilt=50deg in one frame ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=30 tilt=50",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 30, 50),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- status after the move ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- neck pan to 130deg (within +-135), tilt held at 60: expect ACK ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=130 tilt=60",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 130, 60),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- neck pan out of range (140deg, +-135 limit), expect NACK OUT_OF_RANGE ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=140 tilt=60",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 140, 60),
    )

    print("\n--- neck tilt out of range (95deg, 30-90 limit), expect NACK OUT_OF_RANGE ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=0 tilt=95",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 0, 95),
    )

    print("\n--- neck tilt out of range (10deg, below the 30 deg floor), expect NACK OUT_OF_RANGE ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=0 tilt=10",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 0, 10),
    )

    print("\n--- left-arm joint (same pan/tilt limits as neck): expect ACK ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO left-arm pan=10 tilt=40",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_LEFT_ARM, 10, 40),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- reset every joint to its home pose in one command ---")
    print("    (neck home = pan +45.00, tilt +60.00 on the command scale -- tilt's window midpoint)")
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, delay_s=MOVE_DELAY_S)

    print("\n--- status after reset: neck should read pan=45.0, tilt=60.0 ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- stop, then move/reset should be rejected as ESTOPPED ---")
    print("    (no keep-alive heartbeats in this block -- one would clear the e-stop we're testing for)")
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False)
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=10 tilt=60",  # in-range angles, so ESTOPPED is the only reason this gets NACK'd
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 10, 60),
        keep_alive=False,
    )
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, keep_alive=False)

    print("\n--- heartbeat clears e-stop, reset once more to leave the neck home ---")
    arm(ser)
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, delay_s=MOVE_DELAY_S)


def main():
    ser = open_port()
    try:
        run(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
