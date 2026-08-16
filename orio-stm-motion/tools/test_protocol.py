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

CMD_HEARTBEAT = 0x01
CMD_MOVE_ARM_TO = 0x02
CMD_STOP = 0x03
CMD_GET_STATUS = 0x04
CMD_SET_FAN_SPEED = 0x05
CMD_SET_FAN_RGB = 0x06

CMD_ACK = 0x80
CMD_NACK = 0x81
CMD_STATUS = 0x82

CMD_NAMES = {
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_MOVE_ARM_TO: "MOVE_ARM_TO",
    CMD_STOP: "STOP",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_SET_FAN_SPEED: "SET_FAN_SPEED",
    CMD_SET_FAN_RGB: "SET_FAN_RGB",
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


def send_and_show(ser: serial.Serial, label: str, cmd: int, payload: bytes = b"", corrupt_crc: bool = False):
    frame = build_frame(cmd, payload, corrupt_crc=corrupt_crc)
    print(f"-> {label}: {frame.hex(' ')}")
    ser.write(frame)

    resp = read_frame(ser)
    if resp is None:
        print("<- (no response / timeout)")
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
        joints = []
        for i in range(1, len(resp_payload), 2):
            angle_cdeg = int.from_bytes(resp_payload[i : i + 2], "little", signed=True)
            joints.append(angle_cdeg / 100.0)
        print(f"<- {name} estopped={estopped} joint_angles_deg={joints}")
    else:
        print(f"<- {name} payload={resp_payload.hex(' ')}")
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

    print("\n--- move joint 0 to 30.00 deg ---")
    send_and_show(ser, "MOVE_ARM_TO joint=0 30deg", CMD_MOVE_ARM_TO, bytes([0]) + (3000).to_bytes(2, "little", signed=True))

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

    print("\n--- move out of range (95 deg), expect NACK OUT_OF_RANGE ---")
    send_and_show(ser, "MOVE_ARM_TO joint=0 95deg", CMD_MOVE_ARM_TO, bytes([0]) + (9500).to_bytes(2, "little", signed=True))

    print("\n--- corrupted CRC, expect silent drop (no response), board stays alive ---")
    send_and_show(ser, "HEARTBEAT (bad CRC)", CMD_HEARTBEAT, corrupt_crc=True)
    send_and_show(ser, "HEARTBEAT (good, confirms still alive)", CMD_HEARTBEAT)

    print("\n--- stop (also cuts fan speed to 0%), then move/fan-speed should be rejected as ESTOPPED ---")
    send_and_show(ser, "STOP", CMD_STOP)
    send_and_show(ser, "MOVE_ARM_TO joint=0 10deg", CMD_MOVE_ARM_TO, bytes([0]) + (1000).to_bytes(2, "little", signed=True))
    send_and_show(ser, "SET_FAN_SPEED 50%", CMD_SET_FAN_SPEED, bytes([50]))

    print("\n--- heartbeat watchdog: clear e-stop, then let it lapse (>500ms) ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT)
    time.sleep(0.6)
    send_and_show(ser, "GET_STATUS (expect estopped=1)", CMD_GET_STATUS)

    ser.close()


if __name__ == "__main__":
    main()
