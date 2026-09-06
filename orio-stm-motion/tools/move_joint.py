#!/usr/bin/env python3
"""
Moves a single axis (pan or tilt) of one joint to an absolute angle.

CMD_MOVE_JOINT_TO always commands both servos of a joint in one frame (see
Core/Inc/protocol.h), so this reads the joint's current angles from
CMD_GET_STATUS first and re-sends the other axis unchanged alongside the
one being moved.

Usage:
    pip install pyserial
    python tools/move_joint.py <port> <joint> <pan|tilt> <degrees>

Angles are on the vendor scale: pan 0..270, tilt 0..180, measured from each
servo's own zero end, the same numbers the vendor's datasheet and example
sketches use. Tilt 90 is mid-travel -- level for a square bracket.

The firmware rejects the whole frame with NACK OUT_OF_RANGE if either axis
falls outside that joint's commandable window, and the windows differ by
joint. See JOINT_LIMITS_DEG in proto_client.py for the current values.

Examples:
    python tools/move_joint.py COM5 neck pan 180
    python tools/move_joint.py COM5 neck tilt 90
    python tools/move_joint.py COM5 left-arm tilt 60
"""
import argparse
import sys

from proto_client import (
    CMD_ACK,
    CMD_GET_STATUS,
    CMD_MOVE_JOINT_TO,
    JOINT_POS_NAMES,
    MOVE_DELAY_S,
    arm,
    move_joint_payload,
    open_serial,
    send_and_show,
)

JOINT_NAME_TO_POS = {name: pos for pos, name in JOINT_POS_NAMES.items()}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move one axis of a joint to an absolute angle, holding the other axis steady.",
    )
    parser.add_argument("port", help="serial port, e.g. COM5")
    parser.add_argument("joint", choices=sorted(JOINT_NAME_TO_POS), help="which joint to move")
    parser.add_argument("axis", choices=("pan", "tilt"), help="which axis to move")
    parser.add_argument("degrees", type=float, help="target angle in degrees")
    return parser.parse_args()


def get_joint_angles(ser: "serial.Serial", position: int):
    """Reads CMD_GET_STATUS and returns (pan_deg, tilt_deg) for one joint position."""
    resp = send_and_show(ser, "GET_STATUS", CMD_GET_STATUS, delay_s=0.0)
    if resp is None:
        print("no status response -- can't read the joint's current angle")
        sys.exit(1)

    _, payload = resp
    body = payload[1:]  # skip the leading estopped byte
    offset = position * 4
    pan_cdeg = int.from_bytes(body[offset : offset + 2], "little", signed=True)
    tilt_cdeg = int.from_bytes(body[offset + 2 : offset + 4], "little", signed=True)
    return pan_cdeg / 100.0, tilt_cdeg / 100.0


def main():
    args = parse_args()
    position = JOINT_NAME_TO_POS[args.joint]

    ser = open_serial(args.port)
    try:
        arm(ser)

        print(f"\n--- reading current angles for {args.joint} ---")
        pan_deg, tilt_deg = get_joint_angles(ser, position)
        print(f"    current: pan={pan_deg:g} tilt={tilt_deg:g}")

        if args.axis == "pan":
            pan_deg = args.degrees
        else:
            tilt_deg = args.degrees

        print(f"\n--- moving {args.joint} {args.axis} to {args.degrees:g}deg (pan={pan_deg:g} tilt={tilt_deg:g}) ---")
        resp = send_and_show(
            ser,
            f"MOVE_JOINT_TO {args.joint} pan={pan_deg:g} tilt={tilt_deg:g}",
            CMD_MOVE_JOINT_TO,
            move_joint_payload(position, pan_deg, tilt_deg),
            delay_s=MOVE_DELAY_S,
        )
        if resp is not None and resp[0] != CMD_ACK:
            print("move was rejected -- see reason above (likely out of range or e-stopped)")
            sys.exit(1)

        print("\n--- status after the move ---")
        send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
