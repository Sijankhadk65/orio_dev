"""Serial link to the STM32 motion board — the neck and arm servos.

The wire format is the STM32's, not ours: see `orio-stm-motion/Core/Inc/
protocol.h` for the authoritative frame layout and opcodes, and
`orio-stm-motion/tools/proto_client.py` for the bench-test client this is
ported from. Framing repeated here rather than imported because that lives in
a separate repository and this one must stand alone — the same reason
`drivetrain.py` carries its own copy.

    [STX=0xAA][LEN][CMD][PAYLOAD...][CRC16_LO][CRC16_HI]     CRC-16/CCITT-FALSE

The motion primitive is `CMD_MOVE_JOINT_TO`: one joint *position* (neck, left
arm, right arm), each a pan/tilt pair moved together in a single frame. Angles
are on the vendor's scale — pan 0..270, tilt 0..180, each measured from that
servo's own zero end — sent in hundredths of a degree.

## Firmware behaviours that dictate the shape of this module

**Identity can only be confirmed on the wire.** Both STM32 boards present their
ST-LINK VCP as USB 0483:374b, so both land on /dev/ttyACM* in enumeration order,
and the udev symlinks `config.py` defaults to are the only thing telling them
apart by name. A symlink is a convenience, not a guarantee: it silently matches
nothing if a board is swapped for one with a different ST-LINK serial, and it
cannot catch motion firmware flashed onto the drivetrain board. Both boards
share this framing and CRC, so a `CMD_MOVE_JOINT_TO` that reaches the drivetrain
board passes CRC and decodes as a valid frame of the wrong kind — the wheels
take a joint angle as throttle, with nothing on the console. So `connect()` asks
`CMD_WHOAMI` and refuses to return a usable link unless the board answers
`ROLE_MOTION`, and it asks before the first heartbeat, because the heartbeat is
what arms the board.

**A pose is only held while the heartbeats keep coming.** This is the difference
that matters most against `drivetrain.py`. On the drivetrain, an e-stop zeroes a
*commanded speed* and the wheels simply stop. Here, `handle_frame`'s e-stop path
runs `stop_all_joints()` → `ServoJoint_Stop()` → `Servo_Stop()`, which is
`HAL_TIM_PWM_Stop` — the servo's pulse train ceases and the joint is *released*,
with no holding torque. The firmware's watchdog latches that e-stop
`PROTO_HEARTBEAT_TIMEOUT_MS` (500 ms) after the last `CMD_HEARTBEAT`.

So a caller that connects, commands a pose and disconnects does not leave the
neck where it put it: half a second later the servos go slack and it sags under
its own weight. Holding a pose means holding this link open for as long as the
pose is wanted, which is what `_heartbeat_loop` is for and why `move_to()`
callers are expected to keep the object alive rather than fire and forget.

**Range is validated against limits that move.** `handle_move_joint_to` checks
every angle against `kJointLimits[]` in `Core/Src/servo_joint.c` and answers
`NACK_OUT_OF_RANGE` without moving anything. That table is edited as the
mechanical stops get characterised — the neck's tilt window in particular is
currently opened to the servo's full travel *temporarily*, for calibration, and
is meant to return to a narrow band once the stops are known. This module
deliberately keeps no copy of those limits: duplicating them is how a client
comes to believe in a window the firmware has already stopped having. The
firmware is the authority, so `move_to()` waits for its verdict and returns the
rejection rather than assuming the move happened.

**Rejections are silent unless you read them.** A NACK is answered and otherwise
ignored — the joint simply does not move, with nothing on the console to say
why. `_reader_loop` parses the return traffic so `last_rejection` can be
surfaced instead of discarded.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)

STX = 0xAA

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

NACK_REASONS = {
    0x01: "BAD_CRC",
    0x02: "BAD_LENGTH",
    0x03: "UNKNOWN_CMD",
    0x04: "OUT_OF_RANGE",
    0x05: "ESTOPPED",
}

# ProtoRole in Core/Inc/protocol.h. Numbered identically on both boards --
# deliberately, since the point of the value is to recognise the board this link
# did NOT want to be talking to.
ROLE_DRIVETRAIN = 0x01
ROLE_MOTION = 0x02

ROLE_NAMES = {
    ROLE_DRIVETRAIN: "drivetrain",
    ROLE_MOTION: "motion",
}

# Wire-protocol version this module is written against (PROTO_VERSION in the
# firmware's protocol.h, the same value on both boards). It moves only when the
# framing or a payload layout changes incompatibly.
PROTO_VERSION = 1

# The identity handshake. The timeout is short because a board that is listening
# at all answers from its main loop within a millisecond or two; the retries are
# what matter, since opening the ST-LINK VCP resets the target on some
# host/driver combinations and the first WHOAMI can land while the board is
# still rebooting.
IDENTITY_TIMEOUT_S = 0.2
IDENTITY_ATTEMPTS = 3

# Must match ServoJointPosition_t in Core/Inc/servo_joint.h.
JOINT_NECK = 0
JOINT_LEFT_ARM = 1
JOINT_RIGHT_ARM = 2

JOINT_NAMES = {
    JOINT_NECK: "neck",
    JOINT_LEFT_ARM: "left-arm",
    JOINT_RIGHT_ARM: "right-arm",
}

# Comfortably inside the firmware's 500 ms watchdog, with room for a couple of
# dropped frames before it trips -- and before a held pose goes slack.
HEARTBEAT_INTERVAL_S = 0.2

# How long to wait for the board's ACK/NACK for a move. Generous next to the
# millisecond the firmware takes to answer: the reply says whether the angles
# were accepted, and a caller that gave up early would report a move as
# unconfirmed purely because the link was briefly busy.
REPLY_TIMEOUT_S = 0.5

# The motion profile the firmware runs every joint axis along. Must match
# SERVO_JOINT_SLEW_CDEG_PER_S and SERVO_JOINT_ACCEL_CDEG_PER_S2 in
# Core/Inc/servo_joint.h (which are in hundredths of a degree).
JOINT_CRUISE_DEG_PER_S = 120.0
JOINT_ACCEL_DEG_PER_S2 = 400.0


def travel_time_s(delta_deg: float) -> float:
    """How long one axis takes to travel `delta_deg`, ramps included.

    Mirrored from the firmware only to predict when a move is over.
    `CMD_MOVE_JOINT_TO` is ACKed the moment the angles are *accepted* and
    nothing reports arrival, so anything that needs the joint to be still —
    a camera about to look through the head — has to work this out itself.

    The firmware accelerates to `JOINT_CRUISE_DEG_PER_S` and brakes back to a
    stop, so a move costs its cruise time plus one whole ramp. A move too short
    to reach cruise never pays that: it is all ramp, and takes the triangular
    profile's `2 * sqrt(delta / accel)` instead. Both axes of a joint travel at
    once, so a pan-and-tilt move takes the longer of the two, not the sum.

    Runs a hair short of the firmware's real travel — up to 33 ms, measured
    against the profile itself, because the firmware integrates the ramp in
    integer hundredths of a degree per millisecond and loses the remainder.
    Anything looking through the head waits `config.SEEK_SETTLE_S` for the
    exposure afterwards regardless, which more than covers it.
    """
    delta = abs(delta_deg)
    ramp_deg = JOINT_CRUISE_DEG_PER_S**2 / (2.0 * JOINT_ACCEL_DEG_PER_S2)
    if delta <= 2.0 * ramp_deg:
        return 2.0 * math.sqrt(delta / JOINT_ACCEL_DEG_PER_S2)
    return delta / JOINT_CRUISE_DEG_PER_S + JOINT_CRUISE_DEG_PER_S / JOINT_ACCEL_DEG_PER_S2


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE. Must match `crc16_ccitt()` in Core/Src/protocol.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    length = 1 + len(payload)
    crc = crc16_ccitt(bytes([length, cmd]) + payload)
    return bytes([STX, length, cmd]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def move_joint_payload(position: int, pan_deg: float, tilt_deg: float) -> bytes:
    """[position][pan_cdeg int16 LE][tilt_cdeg int16 LE] — both servos of one
    joint pair move in a single frame, in hundredths of a degree."""
    return (
        bytes([position])
        + round(pan_deg * 100).to_bytes(2, "little", signed=True)
        + round(tilt_deg * 100).to_bytes(2, "little", signed=True)
    )


class IdentityError(RuntimeError):
    """The board on this port is not the motion board, or would not say what it is.

    Deliberately fatal. This is the one failure that must not soften into a
    warning: every frame this module sends is also a well-formed frame on the
    drivetrain board, so a link that carried on regardless would hand the wheels
    a joint angle as throttle and report nothing.
    """


@dataclass(frozen=True)
class Identity:
    """What a board answers CMD_WHOAMI with."""

    role: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    proto_version: int

    @property
    def role_name(self) -> str:
        return ROLE_NAMES.get(self.role, hex(self.role))

    @property
    def firmware(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    def __str__(self) -> str:
        return f"{self.role_name} board, firmware {self.firmware}, protocol v{self.proto_version}"


@dataclass(frozen=True)
class Rejection:
    """A NACK from the board — the reason a command did nothing."""

    cmd: int
    reason: str
    timestamp: float

    def __str__(self) -> str:
        return f"{CMD_NAMES.get(self.cmd, hex(self.cmd))} rejected: {self.reason}"


@dataclass(frozen=True)
class JointAngles:
    pan_deg: float
    tilt_deg: float

    def __str__(self) -> str:
        return f"pan {self.pan_deg:g}°, tilt {self.tilt_deg:g}°"


@dataclass(frozen=True)
class Status:
    estopped: bool
    joints: dict
    timestamp: float


class Motion:
    """An open link to the motion board, kept alive by a heartbeat thread.

    Use as a context manager. Note what `close()` can and cannot promise: unlike
    the drivetrain, there is no way to leave this board holding a pose after the
    link goes away — the firmware releases every servo when it e-stops, and it
    e-stops on its own 500 ms after the last heartbeat. Closing therefore always
    means letting the joints go slack; keeping them where you put them means
    keeping this object alive.
    """

    def __init__(self, port: str, baud: int = 115200) -> None:
        self._port_name = port
        self._baud = baud
        self._serial = None
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._last_rejection: Rejection | None = None
        self._last_status: Status | None = None
        self._identity: Identity | None = None
        self._identity_seen = threading.Event()
        self._status_seen = threading.Event()
        # Set by the reader for any ACK/NACK, so a command can wait for the
        # board's verdict on it. Heartbeat ACKs land here too and are the reason
        # _await_reply() matches on the echoed orig_cmd rather than the first
        # reply to arrive: at 5 Hz a heartbeat ACK will otherwise answer for the
        # move that was sent alongside it.
        self._reply_seen = threading.Event()
        self._last_reply: tuple[int, bytes] | None = None
        self._acks = 0

    # ── lifecycle ────────────────────────────────────────────────────────────

    def connect(self) -> "Motion":
        import serial  # local import so importing this module needs no pyserial

        self._serial = serial.Serial(self._port_name, self._baud, timeout=0.1)
        time.sleep(0.2)  # let the port settle before the first write
        self._serial.reset_input_buffer()

        # The reader starts alone, and first: the identity check needs return
        # traffic parsed, and has to have finished before anything arms the
        # board. Starting the heartbeat alongside it would arm whatever is on
        # the port before we knew what it was.
        self._start_thread(self._reader_loop)
        try:
            self._verify_identity()
        except Exception:
            self._abandon()
            raise

        self._start_thread(self._heartbeat_loop)
        # The board boots e-stopped and stays there until the first heartbeat,
        # and while e-stopped it NACKs every move. Send one here and wait for
        # the ACK rather than trusting the thread's next tick, so a caller that
        # commands a pose immediately after connect() is not racing the arming.
        self._await_reply(CMD_HEARTBEAT, build_frame(CMD_HEARTBEAT))
        if self._acks == 0:
            log.warning("no ACK from the motion board yet on %s", self._port_name)
        return self

    def _abandon(self) -> None:
        """Drop an unverified link: stop the reader and close the port.

        Not close(), deliberately. close() sends CMD_STOP, and on a board whose
        identity is unknown or wrong that is a frame of unknown meaning — the
        exact thing this handshake exists to prevent. Nothing needs stopping in
        any case: no heartbeat has been sent, so the board is still sitting in
        the e-stop it booted into."""
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        # Unlike close(), this leaves the object reusable: a caller that fixes
        # the port and retries connect() gets a live reader rather than threads
        # that see an already-set stop event and exit on their first check.
        self._stop_event.clear()

    def _verify_identity(self) -> None:
        """Confirm the board on this port is the motion board, or raise.

        Runs before the first heartbeat, so a board that fails the check is never
        armed."""
        identity = self._request_identity()
        if identity is None:
            raise IdentityError(
                f"no CMD_IDENTITY from the board on {self._port_name} after "
                f"{IDENTITY_ATTEMPTS} attempts — expected the "
                f"{ROLE_NAMES[ROLE_MOTION]} board. Either nothing is "
                "listening on that port, or the firmware on it predates "
                "CMD_WHOAMI."
            )
        if identity.role != ROLE_MOTION:
            raise IdentityError(
                f"wrong board on {self._port_name}: expected role "
                f"{ROLE_NAMES[ROLE_MOTION]} (0x{ROLE_MOTION:02x}), got "
                f"{identity.role_name} (0x{identity.role:02x}) running firmware "
                f"{identity.firmware}. Refusing the link — every command it would "
                "carry decodes as a valid frame on that board."
            )
        if identity.proto_version != PROTO_VERSION:
            log.warning(
                "%s speaks protocol v%d; this client is written against v%d",
                self._port_name,
                identity.proto_version,
                PROTO_VERSION,
            )
        log.info("identified %s on %s", identity, self._port_name)

    def _request_identity(self) -> Identity | None:
        """Ask CMD_WHOAMI until the board answers, or the attempts run out."""
        for attempt in range(1, IDENTITY_ATTEMPTS + 1):
            self._identity_seen.clear()
            self._write(build_frame(CMD_WHOAMI))
            if self._identity_seen.wait(IDENTITY_TIMEOUT_S):
                return self._identity
            log.debug(
                "no identity from %s on attempt %d/%d",
                self._port_name,
                attempt,
                IDENTITY_ATTEMPTS,
            )
        return None

    def close(self) -> None:
        """Release the joints and leave the board latched in e-stop.

        Heartbeats cease first, deliberately: with them stopped, `CMD_STOP`'s
        e-stop cannot be cleared by a straggler ping, and the firmware's own
        watchdog independently latches within 500 ms even if the STOP frame is
        lost on the wire.

        There is no variant of this that parks the joints powered. `CMD_STOP`
        cuts the servo PWM, and so does the watchdog — the STOP is sent anyway
        because releasing now, deterministically, beats releasing at whatever
        moment the watchdog happens to trip. A joint left somewhere gravity acts
        on will drop when this runs.
        """
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._serial is not None:
            try:
                self._write(build_frame(CMD_STOP))
                time.sleep(0.05)
            except Exception:  # a dying link must not mask the caller's own error
                log.exception("failed to send the shutdown stop")
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "Motion":
        # Idempotent: callers that need to handle a failed open separately
        # connect() first and then use the already-open link as a context
        # manager, and connecting twice would open the port twice and start a
        # second pair of threads.
        if self._serial is None:
            self.connect()
        return self

    def __exit__(self, *exc_info) -> bool:
        self.close()
        return False

    # ── commands ─────────────────────────────────────────────────────────────

    def move_to(
        self, position: int, pan_deg: float, tilt_deg: float, timeout_s: float = REPLY_TIMEOUT_S
    ) -> Rejection | None:
        """Move one joint's pan and tilt together. Returns None if the board
        accepted the angles, or the `Rejection` explaining why it did not.

        Angles are NOT clamped here. The firmware validates them against limits
        that are still being characterised (see the module docstring), so the
        only honest answer about whether a pose is reachable is the board's, and
        clamping would turn a rejected angle into a silently different pose.
        `NACK_OUT_OF_RANGE` means nothing moved.
        """
        rejection = self._await_reply(
            CMD_MOVE_JOINT_TO,
            build_frame(CMD_MOVE_JOINT_TO, move_joint_payload(position, pan_deg, tilt_deg)),
            timeout_s=timeout_s,
        )
        if rejection is not None:
            log.warning(
                "%s move to pan %g°, tilt %g° refused: %s",
                JOINT_NAMES.get(position, position),
                pan_deg,
                tilt_deg,
                rejection.reason,
            )
        return rejection

    def stop(self) -> None:
        """E-stop the board, releasing every servo. See the module docstring:
        while this object's heartbeat thread runs, the e-stop is cleared again
        within 200 ms — but the joints have already gone slack and been resumed
        wherever they fell to, so this is not a way to pause a pose."""
        self._write(build_frame(CMD_STOP))

    def reset_joints(self) -> Rejection | None:
        """Drive every joint to its firmware-defined home pose."""
        return self._await_reply(CMD_RESET_JOINTS, build_frame(CMD_RESET_JOINTS))

    def request_status(self) -> None:
        """Ask for joint angles. The answer arrives asynchronously in
        `last_status` — this does not block waiting for it."""
        self._write(build_frame(CMD_GET_STATUS))

    def read_status(self, timeout_s: float = REPLY_TIMEOUT_S) -> Status | None:
        """Ask for joint angles and wait for the reply, so a caller can confirm
        a pose actually took rather than assuming the ACK meant it arrived."""
        self._status_seen.clear()
        self._write(build_frame(CMD_GET_STATUS))
        if self._status_seen.wait(timeout_s):
            return self._last_status
        return None

    @property
    def last_rejection(self) -> Rejection | None:
        return self._last_rejection

    @property
    def last_status(self) -> Status | None:
        return self._last_status

    @property
    def identity(self) -> Identity | None:
        """What the board said it was during connect(), for callers that want to
        log what they are commanding. Never None on an open link: connect()
        raises rather than return one without it."""
        return self._identity

    def take_rejection(self) -> Rejection | None:
        """Consume the last rejection, so a caller polling this reports each
        one exactly once."""
        rejection, self._last_rejection = self._last_rejection, None
        return rejection

    # ── internals ────────────────────────────────────────────────────────────

    def _await_reply(
        self, cmd: int, frame: bytes, timeout_s: float = REPLY_TIMEOUT_S
    ) -> Rejection | None:
        """Send a frame and wait for the ACK or NACK that echoes it back.

        Returns None on an ACK — or on a timeout, which is deliberately not
        reported as a rejection: a lost reply says nothing about whether the
        command was applied, and inventing a refusal would be a worse lie than
        staying quiet. Real refusals always answer.
        """
        deadline = time.monotonic() + timeout_s
        self._reply_seen.clear()
        self._write(frame)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                log.warning(
                    "no reply to %s from %s within %.2gs",
                    CMD_NAMES.get(cmd, hex(cmd)),
                    self._port_name,
                    timeout_s,
                )
                return None
            if not self._reply_seen.wait(remaining):
                continue
            self._reply_seen.clear()
            reply = self._last_reply
            if reply is None:
                continue
            reply_cmd, payload = reply
            # Match on the echoed original command: ACKs for the heartbeat
            # thread's pings are interleaved with these and would otherwise be
            # mistaken for the answer to this frame.
            if not payload or payload[0] != cmd:
                continue
            if reply_cmd == CMD_NACK and len(payload) >= 2:
                return Rejection(
                    cmd=payload[0],
                    reason=NACK_REASONS.get(payload[1], hex(payload[1])),
                    timestamp=time.monotonic(),
                )
            return None

    def _start_thread(self, target) -> None:
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _write(self, frame: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("motion link is not open — call connect() first")
        with self._write_lock:
            self._serial.write(frame)

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.wait(HEARTBEAT_INTERVAL_S):
            try:
                self._write(build_frame(CMD_HEARTBEAT))
            except Exception:
                log.exception(
                    "heartbeat failed — the board will e-stop within 500 ms and "
                    "every joint will go slack"
                )
                return

    def _reader_loop(self) -> None:
        """Parse return traffic so ACK/NACK/STATUS/IDENTITY are observed rather
        than discarded. Without this the input buffer would also grow unbounded
        over a long session.

        The identity handshake latches its reply here rather than reading the
        port itself: two readers on one port race for bytes, and whichever lost
        would see half a frame."""
        while not self._stop_event.is_set():
            try:
                frame = self._read_frame()
            except Exception:
                if not self._stop_event.is_set():
                    log.exception("motion read failed")
                return
            if frame is None:
                continue
            cmd, payload = frame
            if cmd == CMD_ACK:
                self._acks += 1
                self._last_reply = (cmd, payload)
                self._reply_seen.set()
            elif cmd == CMD_NACK and len(payload) >= 2:
                self._last_rejection = Rejection(
                    cmd=payload[0],
                    reason=NACK_REASONS.get(payload[1], hex(payload[1])),
                    timestamp=time.monotonic(),
                )
                self._last_reply = (cmd, payload)
                self._reply_seen.set()
            elif cmd == CMD_STATUS and payload:
                self._last_status = _parse_status(payload)
                self._status_seen.set()
            elif cmd == CMD_IDENTITY and len(payload) >= 5:
                self._identity = _parse_identity(payload)
                self._identity_seen.set()

    def _read_frame(self):
        """One frame, or None on timeout / bad CRC. Byte-at-a-time resync on
        STX, matching the firmware's own parser."""
        ser = self._serial
        if ser is None:
            return None
        b = ser.read(1)
        if not b:
            return None
        if b[0] != STX:
            return None  # resync: keep scanning for a start byte
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
            log.warning("motion response CRC mismatch, frame ignored")
            return None
        return body[0], body[1:]


def _parse_identity(payload: bytes) -> Identity:
    """[role][fw_major][fw_minor][fw_patch][proto_version]."""
    return Identity(
        role=payload[0],
        fw_major=payload[1],
        fw_minor=payload[2],
        fw_patch=payload[3],
        proto_version=payload[4],
    )


def _parse_status(payload: bytes) -> Status:
    """[estopped] then, per joint position: [pan_cdeg 2B][tilt_cdeg 2B]."""
    body = payload[1:]
    joints = {}
    for i in range(0, len(body) - 3, 4):
        position = i // 4
        joints[JOINT_NAMES.get(position, position)] = JointAngles(
            pan_deg=int.from_bytes(body[i : i + 2], "little", signed=True) / 100.0,
            tilt_deg=int.from_bytes(body[i + 2 : i + 4], "little", signed=True) / 100.0,
        )
    return Status(estopped=bool(payload[0]), joints=joints, timestamp=time.monotonic())
