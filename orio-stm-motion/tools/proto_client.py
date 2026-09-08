"""
Shared UART protocol client for the orio-stm-motion test scripts (see
Core/Inc/protocol.h for the authoritative frame/command definitions).

Not a script itself -- the test_*.py scripts in this directory import from
here so the framing/CRC/serial-handling logic lives in exactly one place.
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
EXPECTED_ROLE = ROLE_MOTION
EXPECTED_PROTO_VERSION = 1

# Must match ServoJointPosition_t in Core/Inc/servo_joint.h.
JOINT_POS_NECK = 0
JOINT_POS_LEFT_ARM = 1
JOINT_POS_RIGHT_ARM = 2

JOINT_POS_NAMES = {
    JOINT_POS_NECK: "neck",
    JOINT_POS_LEFT_ARM: "left-arm",
    JOINT_POS_RIGHT_ARM: "right-arm",
}

# Servo model calibration, mirroring PAN/TILT_SERVO_* in
# Core/Inc/servo_joint.h: (min_cdeg, max_cdeg, min_pulse_us, max_pulse_us).
# The full rated travel of each servo MODEL -- not what any joint is allowed
# to command. Lets a tool show the pulse width a given angle will produce.
PAN_SERVO_CALIB = (0, 27000, 500, 2500)
TILT_SERVO_CALIB = (0, 18000, 500, 2500)

# Commandable angle window per joint position, in degrees on the VENDOR
# scale -- pan 0..270, tilt 0..180, each measured from that servo's own zero
# end, exactly as the vendor's datasheet and example code number them. Tilt
# mid-travel (level, for a square bracket) is 90. Mirrors kJointLimits[] in
# Core/Src/servo_joint.c -- keep the two in step.
#
# The firmware remains the authority: it re-validates every angle and NACKs
# OUT_OF_RANGE regardless of what this table says. The table exists so the
# test scripts can DERIVE angles that are in or out of range instead of
# hardcoding literals, which is how test_servo.py came to be asserting
# against a neck tilt window (30..90) the firmware had already stopped
# having.
JOINT_LIMITS_DEG = {
    # TEMPORARY: neck tilt opened to full travel for calibration, mirroring
    # the same temporary row in kJointLimits[]. Restore to (85.0, 95.0), or to
    # whatever sweep_axis.py measures, once the stops are known.
    JOINT_POS_NECK: {"pan": (0.0, 270.0), "tilt": (0.0, 180.0)},
    JOINT_POS_LEFT_ARM: {"pan": (0.0, 270.0), "tilt": (30.0, 90.0)},
    JOINT_POS_RIGHT_ARM: {"pan": (0.0, 270.0), "tilt": (30.0, 90.0)},
}

# Home ("reset") pose per joint position, mirroring s_reset_pan_cdeg[] /
# s_reset_tilt_cdeg[] in Core/Src/protocol.c. CMD_RESET_JOINTS drives every
# joint here, and CMD_GET_STATUS reads these back afterwards.
#
# Note this is NOT simply the midpoint of each window: the neck's pan homes
# to 180.00, well off its 135.00 centre, because that is the pose the bracket
# was measured at. Only the tilt axes happen to home to their window midpoint.
JOINT_HOME_DEG = {
    JOINT_POS_NECK: {"pan": 180.0, "tilt": 90.0},
    JOINT_POS_LEFT_ARM: {"pan": 135.0, "tilt": 60.0},
    JOINT_POS_RIGHT_ARM: {"pan": 135.0, "tilt": 60.0},
}


def axis_limit(position: int, axis: str) -> tuple:
    """(min_deg, max_deg) for one axis of one joint position."""
    return JOINT_LIMITS_DEG[position][axis]


def axis_mid(position: int, axis: str) -> float:
    """Midpoint of an axis's commandable window. Always a legal angle, so
    it is the safe filler for the axis a test isn't currently exercising --
    holding the other axis at a hardcoded constant is what silently turned
    range checks into no-ops when a window moved."""
    lo, hi = axis_limit(position, axis)
    return (lo + hi) / 2.0


def axis_just_below(position: int, axis: str, margin: float = 5.0) -> float:
    """An angle below an axis's floor -- must be rejected OUT_OF_RANGE."""
    return axis_limit(position, axis)[0] - margin


def axis_just_above(position: int, axis: str, margin: float = 5.0) -> float:
    """An angle above an axis's ceiling -- must be rejected OUT_OF_RANGE."""
    return axis_limit(position, axis)[1] + margin

CMD_NAMES = {
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_MOVE_JOINT_TO: "MOVE_JOINT_TO",
    CMD_STOP: "STOP",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_SET_FAN_SPEED: "SET_FAN_SPEED",
    CMD_SET_FAN_RGB: "SET_FAN_RGB",
    CMD_RESET_JOINTS: "RESET_JOINTS",
    CMD_WHOAMI: "WHOAMI",
    CMD_ACK: "ACK",
    CMD_NACK: "NACK",
    CMD_STATUS: "STATUS",
    CMD_IDENTITY: "IDENTITY",
}

# Must match ProtoNackReason in Core/Inc/protocol.h.
NACK_BAD_CRC = 0x01
NACK_BAD_LENGTH = 0x02
NACK_UNKNOWN_CMD = 0x03
NACK_OUT_OF_RANGE = 0x04
NACK_ESTOPPED = 0x05

NACK_REASONS = {
    NACK_BAD_CRC: "BAD_CRC",
    NACK_BAD_LENGTH: "BAD_LENGTH",
    NACK_UNKNOWN_CMD: "UNKNOWN_CMD",
    NACK_OUT_OF_RANGE: "OUT_OF_RANGE",
    NACK_ESTOPPED: "ESTOPPED",
}


# --- assertions ----------------------------------------------------------
# Checks are ACCUMULATED rather than raised. A hardware-in-the-loop run
# should report every failure in one pass instead of stopping at the first,
# and an exception mid-script would leave the board wherever it was -- often
# e-stopped, sometimes with the fan spinning -- instead of running the
# cleanup the tail of each script performs. results_exit_code() turns the
# tally into a process exit status at the end.

_RESULTS = []


def reset_results():
    """Clears the tally. Call once at the start of a run; test_protocol.py
    does this so its aggregate summary covers all four subsystems."""
    _RESULTS.clear()


def record(ok: bool, label: str, detail: str = ""):
    _RESULTS.append((bool(ok), label, detail))


def results_exit_code(title: str = "RESULTS") -> int:
    """Prints the tally and returns 0 if every check passed, else 1."""
    failed = [r for r in _RESULTS if not r[0]]
    total = len(_RESULTS)
    print()
    print("=" * 68)
    if total == 0:
        print(f"{title}: no checks ran")
    else:
        print(f"{title}: {total - len(failed)}/{total} checks passed")
    for _, label, detail in failed:
        print(f"  FAIL  {label}: {detail}")
    print("=" * 68)
    return 1 if failed else 0


def describe_response(resp) -> str:
    """One-line rendering of a (cmd, payload) response, for failure text."""
    if resp is None:
        return "no response"
    cmd, payload = resp
    name = CMD_NAMES.get(cmd, hex(cmd))
    if cmd == CMD_NACK and len(payload) >= 2:
        return f"{name} {NACK_REASONS.get(payload[1], hex(payload[1]))}"
    return name


class _Expect:
    """An expected response: a human-readable description plus a check that
    returns None on a match, or a string explaining the mismatch."""

    __slots__ = ("describe", "_check")

    def __init__(self, describe, check):
        self.describe = describe
        self._check = check

    def check(self, resp):
        return self._check(resp)


def expect_ack(orig_cmd: int = None) -> _Expect:
    """ACK, optionally verifying which command it echoes back."""

    def check(resp):
        if resp is None:
            return "no response"
        cmd, payload = resp
        if cmd != CMD_ACK:
            return f"got {describe_response(resp)}"
        if orig_cmd is not None:
            got = payload[0] if payload else None
            if got != orig_cmd:
                return f"ACK echoed orig_cmd={hex(got) if got is not None else '(none)'}"
        return None

    suffix = "" if orig_cmd is None else f" for {CMD_NAMES.get(orig_cmd, hex(orig_cmd))}"
    return _Expect("ACK" + suffix, check)


def expect_nack(reason: int, orig_cmd: int = None) -> _Expect:
    """NACK carrying a specific reason code. The reason matters: range is
    validated before the e-stop flag in handle_move_joint_to(), so a frame
    meant to prove the e-stop rejects it can come back OUT_OF_RANGE and look
    like a pass to anything that only checks 'was it a NACK'."""

    def check(resp):
        if resp is None:
            return "no response"
        cmd, payload = resp
        if cmd != CMD_NACK:
            return f"got {describe_response(resp)}"
        if len(payload) < 2:
            return f"NACK payload too short ({len(payload)} bytes)"
        if payload[1] != reason:
            return f"got NACK {NACK_REASONS.get(payload[1], hex(payload[1]))}"
        if orig_cmd is not None and payload[0] != orig_cmd:
            return f"NACK echoed orig_cmd={hex(payload[0])}"
        return None

    return _Expect(f"NACK {NACK_REASONS.get(reason, hex(reason))}", check)


def expect_no_response() -> _Expect:
    """Frame silently dropped -- what a bad CRC must produce."""

    def check(resp):
        return None if resp is None else f"got {describe_response(resp)}"

    return _Expect("no response (frame dropped)", check)


def expect_status(estopped: int = None, joints: dict = None, tol_deg: float = 0.005) -> _Expect:
    """CMD_STATUS, optionally checking the e-stop flag and any subset of the
    reported joint angles. joints maps position -> (pan_deg, tilt_deg); None
    for either axis skips that axis."""

    def check(resp):
        if resp is None:
            return "no response"
        cmd, payload = resp
        if cmd != CMD_STATUS:
            return f"got {describe_response(resp)}"
        if not payload:
            return "STATUS payload empty"
        if estopped is not None and payload[0] != estopped:
            return f"estopped={payload[0]}, wanted {estopped}"
        body = payload[1:]
        for pos, (want_pan, want_tilt) in (joints or {}).items():
            off = pos * 4
            if len(body) < off + 4:
                return f"STATUS carries no slot for position {pos}"
            got_pan = int.from_bytes(body[off : off + 2], "little", signed=True) / 100.0
            got_tilt = int.from_bytes(body[off + 2 : off + 4], "little", signed=True) / 100.0
            name = JOINT_POS_NAMES.get(pos, pos)
            if want_pan is not None and abs(got_pan - want_pan) > tol_deg:
                return f"{name} pan={got_pan:g}, wanted {want_pan:g}"
            if want_tilt is not None and abs(got_tilt - want_tilt) > tol_deg:
                return f"{name} tilt={got_tilt:g}, wanted {want_tilt:g}"
        return None

    bits = []
    if estopped is not None:
        bits.append(f"estopped={estopped}")
    for pos, (p, t) in sorted((joints or {}).items()):
        axes = ", ".join(
            f"{a}={v:g}" for a, v in (("pan", p), ("tilt", t)) if v is not None
        )
        bits.append(f"{JOINT_POS_NAMES.get(pos, pos)} {axes}")
    return _Expect("STATUS" + (" " + "; ".join(bits) if bits else ""), check)


def expect_identity(role: int = None, proto_version: int = None) -> _Expect:
    """CMD_IDENTITY, optionally checking the role and wire-protocol version.

    An ACK here is a failure, not a pass: CMD_WHOAMI is answered with the
    identity frame itself, and a board that merely ACKed would leave the caller
    knowing the link works but not which board is on it -- which is the whole
    question being asked."""

    def check(resp):
        if resp is None:
            return "no response"
        cmd, payload = resp
        if cmd != CMD_IDENTITY:
            return f"got {describe_response(resp)}"
        if len(payload) < 5:
            return f"IDENTITY payload too short ({len(payload)} bytes)"
        ident = parse_identity_payload(payload)
        if role is not None and ident["role"] != role:
            return (
                f"role={ident['role_name']} (0x{ident['role']:02x}) running firmware "
                f"{ident['firmware']}, wanted {ROLE_NAMES.get(role, hex(role))} "
                "-- WRONG BOARD on this port"
            )
        if proto_version is not None and ident["proto_version"] != proto_version:
            return f"protocol v{ident['proto_version']}, wanted v{proto_version}"
        return None

    bits = []
    if role is not None:
        bits.append(f"role={ROLE_NAMES.get(role, hex(role))}")
    if proto_version is not None:
        bits.append(f"proto=v{proto_version}")
    return _Expect("IDENTITY" + (" " + " ".join(bits) if bits else ""), check)


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


def send_and_show(
    ser: serial.Serial,
    label: str,
    cmd: int,
    payload: bytes = b"",
    corrupt_crc: bool = False,
    delay_s: float = DEFAULT_DELAY_S,
    keep_alive: bool = True,
    expect=None,
):
    """keep_alive=False skips the post-command heartbeat pings during the
    delay -- use it when a test deliberately wants the e-stop to stay
    engaged (e.g. right after CMD_STOP), since a heartbeat would clear it.

    expect= takes one of the expect_*() objects above. The response is then
    checked against it, PASS/FAIL is printed inline, and the result is added
    to the tally results_exit_code() reports on. Omitting it leaves the call
    as print-only, which is right for commands sent purely to set up state
    (arming, cleanup) rather than to assert something."""
    frame = build_frame(cmd, payload, corrupt_crc=corrupt_crc)
    print(f"-> {label}: {frame.hex(' ')}")
    ser.write(frame)

    resp = read_frame(ser)
    if resp is None:
        print("<- (no response / timeout)")
    else:
        _print_response(resp)

    if expect is not None:
        problem = expect.check(resp)
        record(problem is None, label, problem or "")
        if problem is None:
            print(f"   PASS  ({expect.describe})")
        else:
            print(f"   FAIL  expected {expect.describe}; {problem}")

    if keep_alive:
        hold_alive(ser, delay_s)
    else:
        time.sleep(delay_s)
    return resp


def _print_response(resp):
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
    elif resp_cmd == CMD_IDENTITY and len(resp_payload) >= 5:
        ident = parse_identity_payload(resp_payload)
        print(
            f"<- {name} role={ident['role_name']} (0x{ident['role']:02x}) "
            f"fw={ident['firmware']} proto=v{ident['proto_version']}"
        )
    else:
        print(f"<- {name} payload={resp_payload.hex(' ')}")


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


def whoami(ser: serial.Serial, delay_s: float = 0.0, expect=None):
    """Asks the board what it is and returns the decoded identity, or None.

    Sends no heartbeat, and keep_alive is off: identity has to be answerable
    while the board sits in its boot-time e-stop, since the Jetson checks it
    before arming anything (see CMD_WHOAMI in Core/Inc/protocol.h). Arming
    first would hide a handler that wrongly gated the reply on the e-stop."""
    resp = send_and_show(
        ser, "WHOAMI", CMD_WHOAMI, keep_alive=False, delay_s=delay_s, expect=expect
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
    send_and_show(ser, "HEARTBEAT", CMD_HEARTBEAT, expect=expect_ack(CMD_HEARTBEAT))
