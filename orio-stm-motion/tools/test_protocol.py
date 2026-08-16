#!/usr/bin/env python3
"""
Manual test script for the orio-stm-motion UART protocol (see Core/Inc/protocol.h).

Usage:
    pip install pyserial
    python tools/test_protocol.py COM5

Walks through the command set against real hardware and prints what it sent
and what came back. Not an automated pass/fail harness -- read the output.
"""
import sys
import time

import serial

STX = 0xAA

# Pause after each command so the result (servo motion, fan ramp, RGB
# change) is visible before the next one fires. Movement commands get a
# longer pause since the servos physically take time to travel.
DEFAULT_DELAY_S = 0.6
MOVE_DELAY_S = 1.5

# Both delays above exceed the firmware's PROTO_HEARTBEAT_TIMEOUT_MS (500ms),
# so a plain sleep would let the watchdog re-trip the e-stop between every
# command. HEARTBEAT_INTERVAL_S keeps pinging comfortably under that limit
# during a pause; see hold_alive().
HEARTBEAT_INTERVAL_S = 0.3

CMD_HEARTBEAT = 0x01
CMD_MOVE_JOINT_TO = 0x02
CMD_STOP = 0x03
CMD_GET_STATUS = 0x04
CMD_SET_FAN_SPEED = 0x05
CMD_SET_FAN_RGB = 0x06
CMD_RESET_JOINTS = 0x07

CMD_ACK = 0x80
CMD_NACK = 0x81
CMD_STATUS = 0x82

# Must match ServoJointPosition_t in Core/Inc/servo_joint.h.
JOINT_POS_NECK = 0
JOINT_POS_LEFT_ARM = 1
JOINT_POS_RIGHT_ARM = 2

JOINT_POS_NAMES = {
    JOINT_POS_NECK: "neck",
    JOINT_POS_LEFT_ARM: "left-arm",
    JOINT_POS_RIGHT_ARM: "right-arm",
}

CMD_NAMES = {
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_MOVE_JOINT_TO: "MOVE_JOINT_TO",
    CMD_STOP: "STOP",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_SET_FAN_SPEED: "SET_FAN_SPEED",
    CMD_SET_FAN_RGB: "SET_FAN_RGB",
    CMD_RESET_JOINTS: "RESET_JOINTS",
    CMD_ACK: "ACK",
    CMD_NACK: "NACK",
    CMD_STATUS: "STATUS",
}

NACK_REASONS = {
    0x01: "BAD_CRC",
    0x02: "BAD_LENGTH",
    0x03: "UNKNOWN_CMD",
    0x04: "OUT_OF_RANGE",
    0x05: "ESTOPPED",
}


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, must match crc16_ccitt() in Core/Src/protocol.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, payload: bytes = b"", corrupt_crc: bool = False) -> bytes:
    length = 1 + len(payload)
    body = bytes([length, cmd]) + payload
    crc = crc16_ccitt(body)
    if corrupt_crc:
        crc ^= 0xFFFF
    return bytes([STX, length, cmd]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def move_joint_payload(position: int, pan_deg: float, tilt_deg: float) -> bytes:
    """Payload for CMD_MOVE_JOINT_TO: [position][pan_cdeg][tilt_cdeg], moving
    both servos of that joint pair in one frame."""
    pan_cdeg = round(pan_deg * 100)
    tilt_cdeg = round(tilt_deg * 100)
    return (
        bytes([position])
        + pan_cdeg.to_bytes(2, "little", signed=True)
        + tilt_cdeg.to_bytes(2, "little", signed=True)
    )


def read_frame(ser: serial.Serial, timeout_s: float = 1.0):
    """Reads one frame, returns (cmd, payload) or None on timeout/bad CRC."""
    ser.timeout = timeout_s
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == STX:
            break
    length_b = ser.read(1)
    if not length_b:
        return None
    length = length_b[0]
    body = ser.read(length)
    if len(body) != length:
        return None
    crc_bytes = ser.read(2)
    if len(crc_bytes) != 2:
        return None
    crc_rx = crc_bytes[0] | (crc_bytes[1] << 8)
    if crc16_ccitt(bytes([length]) + body) != crc_rx:
        print("  !! response CRC mismatch, ignoring")
        return None
    return body[0], body[1:]


def hold_alive(ser: serial.Serial, duration_s: float):
    """Sleeps for duration_s while sending periodic CMD_HEARTBEAT frames so
    the board's heartbeat watchdog doesn't re-trip the e-stop during a pause
    added purely so this script's output is easier to watch. Drains each
    heartbeat's ACK so it doesn't linger in the serial read buffer."""
    end = time.monotonic() + duration_s
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(HEARTBEAT_INTERVAL_S, remaining))
        if time.monotonic() >= end:
            return
        ser.write(build_frame(CMD_HEARTBEAT))
        read_frame(ser, timeout_s=0.2)


def send_and_show(
    ser: serial.Serial,
    label: str,
    cmd: int,
    payload: bytes = b"",
    corrupt_crc: bool = False,
    delay_s: float = DEFAULT_DELAY_S,
    keep_alive: bool = True,
):
    """keep_alive=False skips the post-command heartbeat pings during the
    delay -- use it when a test deliberately wants the e-stop to stay
    engaged (e.g. right after CMD_STOP), since a heartbeat would clear it."""
    frame = build_frame(cmd, payload, corrupt_crc=corrupt_crc)
    print(f"-> {label}: {frame.hex(' ')}")
    ser.write(frame)

    resp = read_frame(ser)
    if resp is None:
        print("<- (no response / timeout)")
        if keep_alive:
            hold_alive(ser, delay_s)
        else:
            time.sleep(delay_s)
        return None

    resp_cmd, resp_payload = resp
    name = CMD_NAMES.get(resp_cmd, hex(resp_cmd))
    if resp_cmd == CMD_NACK and len(resp_payload) >= 2:
        reason = NACK_REASONS.get(resp_payload[1], hex(resp_payload[1]))
        print(f"<- {name} orig_cmd={hex(resp_payload[0])} reason={reason}")
    elif resp_cmd == CMD_ACK and len(resp_payload) >= 1:
        print(f"<- {name} orig_cmd={hex(resp_payload[0])}")
    elif resp_cmd == CMD_STATUS:
        estopped = resp_payload[0]
        joints = {}
        body = resp_payload[1:]
        for i in range(0, len(body), 4):
            pos = i // 4
            pan_cdeg = int.from_bytes(body[i : i + 2], "little", signed=True)
            tilt_cdeg = int.from_bytes(body[i + 2 : i + 4], "little", signed=True)
            joints[JOINT_POS_NAMES.get(pos, pos)] = (pan_cdeg / 100.0, tilt_cdeg / 100.0)
        print(f"<- {name} estopped={estopped} joints_deg(pan,tilt)={joints}")
    else:
        print(f"<- {name} payload={resp_payload.hex(' ')}")

    if keep_alive:
        hold_alive(ser, delay_s)
    else:
        time.sleep(delay_s)
    return resp


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <serial-port, e.g. COM5>")
        sys.exit(1)

    port = sys.argv[1]
    ser = serial.Serial(port, 115200, timeout=1.0)
    time.sleep(0.2)  # let the port settle after opening

    print("--- heartbeat clears the boot-time e-stop ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT)

    print("\n--- status before any move ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- move the neck to pan=30deg, tilt=-20deg in one frame ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=30 tilt=-20",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 30, -20),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- status after the move ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- fan: ramp speed 0 -> 50 -> 100% ---")
    for pct in (0, 50, 100):
        send_and_show(ser, f"SET_FAN_SPEED {pct}%", CMD_SET_FAN_SPEED, bytes([pct]))

    print("\n--- fan: speed out of range (101%), expect NACK OUT_OF_RANGE ---")
    send_and_show(ser, "SET_FAN_SPEED 101%", CMD_SET_FAN_SPEED, bytes([101]))

    print("\n--- fan RGB: red, green, blue, then off ---")
    for name, rgb in (("red", (255, 0, 0)), ("green", (0, 255, 0)), ("blue", (0, 0, 255)), ("off", (0, 0, 0))):
        send_and_show(ser, f"SET_FAN_RGB {name}", CMD_SET_FAN_RGB, bytes(rgb))

    print("\n--- neck pan to 130deg (within +-135), tilt held at 0: expect ACK ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=130 tilt=0",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 130, 0),
        delay_s=MOVE_DELAY_S,
    )

    print("\n--- neck pan out of range (140deg, +-135 limit), expect NACK OUT_OF_RANGE ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=140 tilt=0",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 140, 0),
    )

    print("\n--- neck tilt out of range (95deg, +-90 limit), expect NACK OUT_OF_RANGE ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=0 tilt=95",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 0, 95),
    )

    print("\n--- left-arm joint (no hardware wired up yet): latches for status but no ACK error either ---")
    send_and_show(
        ser,
        "MOVE_JOINT_TO left-arm pan=10 tilt=10",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_LEFT_ARM, 10, 10),
    )

    print("\n--- reset every joint to its home pose in one command ---")
    print("    (neck home = pan 180deg / tilt 90deg raw servo angle = pan +45.00 / tilt 0.00 on the command scale)")
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, delay_s=MOVE_DELAY_S)

    print("\n--- status after reset: neck should read pan=45.0, tilt=0.0 ---")
    send_and_show(ser, "GET_STATUS", CMD_GET_STATUS)

    print("\n--- corrupted CRC, expect silent drop (no response), board stays alive ---")
    send_and_show(ser, "HEARTBEAT (bad CRC)", CMD_HEARTBEAT, corrupt_crc=True)
    send_and_show(ser, "HEARTBEAT (good, confirms still alive)", CMD_HEARTBEAT)

    print("\n--- stop (also cuts fan speed to 0%), then move/reset/fan-speed should be rejected as ESTOPPED ---")
    print("    (no keep-alive heartbeats in this block -- one would clear the e-stop we're testing for)")
    send_and_show(ser, "STOP", CMD_STOP, keep_alive=False)
    send_and_show(
        ser,
        "MOVE_JOINT_TO neck pan=10 tilt=0",
        CMD_MOVE_JOINT_TO,
        move_joint_payload(JOINT_POS_NECK, 10, 0),
        keep_alive=False,
    )
    send_and_show(ser, "RESET_JOINTS", CMD_RESET_JOINTS, keep_alive=False)
    send_and_show(ser, "SET_FAN_SPEED 50%", CMD_SET_FAN_SPEED, bytes([50]), keep_alive=False)

    print("\n--- heartbeat watchdog: clear e-stop, then let it lapse (>500ms) ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT)
    time.sleep(0.6)
    send_and_show(ser, "GET_STATUS (expect estopped=1)", CMD_GET_STATUS)

    ser.close()


if __name__ == "__main__":
    main()
