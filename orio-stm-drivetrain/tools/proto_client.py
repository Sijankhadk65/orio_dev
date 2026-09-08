"""
Shared UART protocol client for the orio-stm-drivetrain test scripts (see
Core/Inc/protocol.h for the authoritative frame/command definitions).

Not a script itself -- the test scripts in this directory import from here
so the framing/CRC/serial-handling logic lives in exactly one place.
"""
import sys
import time

import serial

STX = 0xAA

# Pause after each command so the result (wheel spin-up, telemetry change)
# is visible before the next one fires.
DEFAULT_DELAY_S = 0.6

# Both this and DEFAULT_DELAY_S exceed the firmware's
# PROTO_HEARTBEAT_TIMEOUT_MS (500ms), so a plain sleep would let the
# watchdog re-trip the e-stop between every command. HEARTBEAT_INTERVAL_S
# keeps pinging comfortably under that limit during a pause; see hold_alive().
HEARTBEAT_INTERVAL_S = 0.3

CMD_HEARTBEAT = 0x01
CMD_SET_DRIVE = 0x02
CMD_STOP = 0x03
CMD_GET_STATUS = 0x04
CMD_WHOAMI = 0x08

CMD_ACK = 0x80
CMD_NACK = 0x81
CMD_STATUS = 0x82
CMD_IDENTITY = 0x83

# Must match ProtoRole in Core/Inc/protocol.h -- the same numbering on both
# boards, so this client can name the board it did NOT expect to reach.
ROLE_DRIVETRAIN = 0x01
ROLE_MOTION = 0x02

ROLE_NAMES = {
    ROLE_DRIVETRAIN: "drivetrain",
    ROLE_MOTION: "motion",
}

# What this repo's firmware must answer CMD_WHOAMI with (PROTO_SELF_ROLE in
# Core/Inc/protocol.h), and the wire-protocol version it reports.
EXPECTED_ROLE = ROLE_DRIVETRAIN
EXPECTED_PROTO_VERSION = 1

# Must match WheelSide_t in Core/Inc/wheel.h.
WHEEL_LEFT = 0
WHEEL_RIGHT = 1

WHEEL_SIDE_NAMES = {
    WHEEL_LEFT: "left",
    WHEEL_RIGHT: "right",
}

CMD_NAMES = {
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_SET_DRIVE: "SET_DRIVE",
    CMD_STOP: "STOP",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_WHOAMI: "WHOAMI",
    CMD_ACK: "ACK",
    CMD_NACK: "NACK",
    CMD_STATUS: "STATUS",
    CMD_IDENTITY: "IDENTITY",
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


def set_drive_payload(left_permille: int, right_permille: int) -> bytes:
    """Payload for CMD_SET_DRIVE: [left_permille][right_permille], each a
    signed int16 in -1000..1000 for -100.0%..100.0% duty."""
    return left_permille.to_bytes(2, "little", signed=True) + right_permille.to_bytes(
        2, "little", signed=True
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
    added purely so a test script's output is easier to watch. Drains each
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


def parse_identity_payload(payload: bytes):
    """Decodes a CMD_IDENTITY payload into a dict.

    Layout is [role][fw_major][fw_minor][fw_patch][proto_version], identical on
    both boards -- deliberately, since a client has to be able to read the reply
    from the board it did not want before it can reject it."""
    role = payload[0]
    return {
        "role": role,
        "role_name": ROLE_NAMES.get(role, hex(role)),
        "firmware": f"{payload[1]}.{payload[2]}.{payload[3]}",
        "proto_version": payload[4],
    }


def parse_status_payload(payload: bytes):
    """Decodes a CMD_STATUS payload into (estopped, {side_name: telemetry_dict})."""
    estopped = payload[0]
    body = payload[1:]
    wheels = {}
    for i in range(0, len(body), 10):
        side = i // 10
        erpm = int.from_bytes(body[i : i + 4], "little", signed=True)
        current_ca = int.from_bytes(body[i + 4 : i + 6], "little", signed=True)
        v_in_dv = int.from_bytes(body[i + 6 : i + 8], "little", signed=True)
        fault_code = body[i + 8]
        valid = body[i + 9]
        wheels[WHEEL_SIDE_NAMES.get(side, side)] = {
            "erpm": erpm,
            "current_a": current_ca / 100.0,
            "v_in": v_in_dv / 10.0,
            "fault_code": fault_code,
            "valid": bool(valid),
        }
    return estopped, wheels


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
        estopped, wheels = parse_status_payload(resp_payload)
        print(f"<- {name} estopped={estopped} wheels={wheels}")
    elif resp_cmd == CMD_IDENTITY and len(resp_payload) >= 5:
        ident = parse_identity_payload(resp_payload)
        print(
            f"<- {name} role={ident['role_name']} (0x{ident['role']:02x}) "
            f"fw={ident['firmware']} proto=v{ident['proto_version']}"
        )
    else:
        print(f"<- {name} payload={resp_payload.hex(' ')}")

    if keep_alive:
        hold_alive(ser, delay_s)
    else:
        time.sleep(delay_s)
    return resp


def open_serial(port: str) -> serial.Serial:
    """Opens a serial port and lets it settle before the first write."""
    ser = serial.Serial(port, 115200, timeout=1.0)
    time.sleep(0.2)  # let the port settle after opening
    return ser


def open_port(argv=None) -> serial.Serial:
    """Parses <serial-port> from argv (defaults to sys.argv), opens it, and
    lets the port settle. Exits with a usage message if the arg is missing."""
    argv = sys.argv if argv is None else argv
    if len(argv) != 2:
        print(f"usage: {argv[0]} <serial-port, e.g. COM5>")
        sys.exit(1)

    return open_serial(argv[1])


def whoami(ser: serial.Serial, delay_s: float = 0.0):
    """Asks the board what it is and returns the decoded identity, or None.

    Sends no heartbeat, and keep_alive is off: identity has to be answerable
    while the board sits in its boot-time e-stop, since the Jetson checks it
    before arming anything (see CMD_WHOAMI in Core/Inc/protocol.h). Arming
    first would hide a handler that wrongly gated the reply on the e-stop."""
    resp = send_and_show(
        ser, "WHOAMI", CMD_WHOAMI, keep_alive=False, delay_s=delay_s
    )
    if resp is None:
        return None
    resp_cmd, resp_payload = resp
    if resp_cmd != CMD_IDENTITY or len(resp_payload) < 5:
        return None
    return parse_identity_payload(resp_payload)


def arm(ser: serial.Serial):
    """Sends the CMD_HEARTBEAT every standalone test needs first to clear
    the boot-time e-stop before it can command anything."""
    print("--- heartbeat clears the boot-time e-stop ---")
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT)
