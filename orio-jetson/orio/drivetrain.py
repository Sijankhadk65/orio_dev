"""Serial link to the STM32 drivetrain board — the motion half of the robot.

The wire format is the STM32's, not ours: see `orio-stm-drivetrain/Core/Inc/
protocol.h` for the authoritative frame layout and opcodes, and
`orio-stm-drivetrain/tools/proto_client.py` for the bench-test client this is
ported from. Framing repeated here rather than imported because that lives in
a separate repository and this one must stand alone.

    [STX=0xAA][LEN][CMD][PAYLOAD...][CRC16_LO][CRC16_HI]     CRC-16/CCITT-FALSE

The one motion primitive the firmware exposes is `Wheel_SetSpeeds(left, right)`
in per-mille of duty, -1000..1000. There is no "forward" or "turn" anywhere in
the firmware; a direction is a choice of sign pair made up here.

## Four firmware behaviours that dictate the shape of this module

**Identity can only be confirmed on the wire.** Both STM32 boards present their
ST-LINK VCP as USB 0483:374b, so both land on /dev/ttyACM* in enumeration order,
and the udev symlinks `config.py` defaults to are the only thing telling them
apart by name. A symlink is a convenience, not a guarantee: it silently matches
nothing if a board is swapped for one with a different ST-LINK serial, and it
cannot catch drivetrain firmware flashed onto the motion board. Both boards share
this framing and CRC, so a `CMD_SET_DRIVE` that reaches the motion board passes
CRC and decodes as a valid frame of the wrong kind — its servos slew to whatever
a wheel duty pair happens to mean, with nothing on the console. So `connect()`
asks `CMD_WHOAMI` and refuses to return a usable link unless the board answers
`ROLE_DRIVETRAIN`, and it asks before the first heartbeat, because the heartbeat
is what arms the board.

**Heartbeats are a separate clock from commands.** Only `CMD_HEARTBEAT` touches
the watchdog timer (`protocol.c`, `handle_frame`); `CMD_SET_DRIVE` does not. So
a caller that drives continuously and never pings still gets e-stopped after
`PROTO_HEARTBEAT_TIMEOUT_MS` (500 ms). Hence `_heartbeat_loop`: a daemon thread
pinging at 5 Hz, independent of whatever the caller is doing.

**`stop()` is not a latch while this object is alive.** `CMD_STOP` sets the
firmware's e-stop, but the very next heartbeat clears it again. That is not a
bug in either side — `Wheel_Stop()` zeroes the *commanded* speeds too, so the
`Wheel_Resume()` that follows resumes zero and the wheels stay still. The
practical consequence: use `set_drive(0, 0)` for routine stops, and treat
`stop()` as "stop now, and stay stopped only because nothing re-commands you".
A latched e-stop means ceasing heartbeats, which is what `close()` does.

**Rejections are silent unless you read them.** A `CMD_SET_DRIVE` sent while
e-stopped is answered with `NACK_ESTOPPED` and otherwise ignored — the wheels
simply do not move, with nothing on the console to say why. `_reader_loop`
parses the return traffic so `last_rejection` can be surfaced instead of
discarded.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)

STX = 0xAA

CMD_HEARTBEAT = 0x01
CMD_SET_DRIVE = 0x02
CMD_STOP = 0x03
CMD_GET_STATUS = 0x04
CMD_WHOAMI = 0x08

CMD_ACK = 0x80
CMD_NACK = 0x81
CMD_STATUS = 0x82
CMD_IDENTITY = 0x83

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

# Must match WheelSide_t in Core/Inc/wheel.h.
WHEEL_SIDE_NAMES = {0: "left", 1: "right"}

MAX_PERMILLE = 1000

# Comfortably inside the firmware's 500 ms watchdog, with room for a couple of
# dropped frames before it trips.
HEARTBEAT_INTERVAL_S = 0.2


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


def set_drive_payload(left_permille: int, right_permille: int) -> bytes:
    """[left int16 LE][right int16 LE], each -1000..1000 for -100..100% duty."""
    return left_permille.to_bytes(2, "little", signed=True) + right_permille.to_bytes(
        2, "little", signed=True
    )


class IdentityError(RuntimeError):
    """The board on this port is not the drivetrain, or would not say what it is.

    Deliberately fatal. This is the one failure that must not soften into a
    warning: every frame this module sends is also a well-formed frame on the
    motion board, so a link that carried on regardless would drive the neck and
    arms with wheel duty values and report nothing.
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
class WheelTelemetry:
    erpm: int
    current_a: float
    v_in: float
    fault_code: int
    valid: bool


@dataclass(frozen=True)
class Status:
    estopped: bool
    wheels: dict
    timestamp: float


class Drivetrain:
    """An open link to the drivetrain board, kept alive by a heartbeat thread.

    Use as a context manager — the `close()` path is the only thing that
    guarantees the wheels are commanded to zero and left latched in e-stop when
    the caller goes away for any reason, including an exception.
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
        self._acks = 0

    # ── lifecycle ────────────────────────────────────────────────────────────

    def connect(self) -> "Drivetrain":
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
        # The board boots e-stopped and stays there until the first heartbeat.
        # The heartbeat thread is already running, so this is really just a
        # chance to notice a dead link before the caller starts driving.
        self._write(build_frame(CMD_HEARTBEAT))
        time.sleep(0.3)
        if self._acks == 0:
            log.warning("no ACK from the drivetrain board yet on %s", self._port_name)
        return self

    def _abandon(self) -> None:
        """Drop an unverified link: stop the reader and close the port.

        Not close(), deliberately. close() sends CMD_SET_DRIVE and CMD_STOP, and
        on a board whose identity is unknown or wrong those are frames of unknown
        meaning — the exact thing this handshake exists to prevent. Nothing needs
        stopping in any case: no heartbeat has been sent, so the board is still
        sitting in the e-stop it booted into."""
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
        """Confirm the board on this port is the drivetrain, or raise.

        Runs before the first heartbeat, so a board that fails the check is never
        armed."""
        identity = self._request_identity()
        if identity is None:
            raise IdentityError(
                f"no CMD_IDENTITY from the board on {self._port_name} after "
                f"{IDENTITY_ATTEMPTS} attempts — expected the "
                f"{ROLE_NAMES[ROLE_DRIVETRAIN]} board. Either nothing is "
                "listening on that port, or the firmware on it predates "
                "CMD_WHOAMI."
            )
        if identity.role != ROLE_DRIVETRAIN:
            raise IdentityError(
                f"wrong board on {self._port_name}: expected role "
                f"{ROLE_NAMES[ROLE_DRIVETRAIN]} (0x{ROLE_DRIVETRAIN:02x}), got "
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
        """Stop the wheels and leave the board latched in e-stop.

        Heartbeats cease first, deliberately: with them stopped, `CMD_STOP`'s
        e-stop cannot be cleared by a straggler ping, and the firmware's own
        watchdog independently latches within 500 ms even if the STOP frame is
        lost on the wire.
        """
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        if self._serial is not None:
            try:
                self._write(build_frame(CMD_SET_DRIVE, set_drive_payload(0, 0)))
                self._write(build_frame(CMD_STOP))
                time.sleep(0.05)
            except Exception:  # a dying link must not mask the caller's own error
                log.exception("failed to send the shutdown stop")
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "Drivetrain":
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

    def set_drive(self, left_permille: int, right_permille: int) -> None:
        """Command both wheels. Values are clamped, not rejected: the firmware
        NACKs anything outside ±1000 and drives nothing, which as a failure mode
        is strictly worse than driving at the maximum the caller asked to
        approach."""
        left = max(-MAX_PERMILLE, min(MAX_PERMILLE, int(left_permille)))
        right = max(-MAX_PERMILLE, min(MAX_PERMILLE, int(right_permille)))
        self._write(build_frame(CMD_SET_DRIVE, set_drive_payload(left, right)))

    def stop(self) -> None:
        """Zero the wheels and e-stop the board. See the module docstring: while
        this object's heartbeat thread runs, the e-stop is cleared again within
        200 ms — the wheels stay stopped regardless, because the firmware clears
        the commanded speeds before resuming."""
        self._write(build_frame(CMD_STOP))

    def request_status(self) -> None:
        """Ask for telemetry. The answer arrives asynchronously in
        `last_status` — this does not block waiting for it."""
        self._write(build_frame(CMD_GET_STATUS))

    @property
    def last_rejection(self) -> Rejection | None:
        return self._last_rejection

    @property
    def last_status(self) -> Status | None:
        return self._last_status

    @property
    def identity(self) -> Identity | None:
        """What the board said it was during connect(), for callers that want to
        log what they are driving. Never None on an open link: connect() raises
        rather than return one without it."""
        return self._identity

    def take_rejection(self) -> Rejection | None:
        """Consume the last rejection, so a caller polling this reports each
        one exactly once."""
        rejection, self._last_rejection = self._last_rejection, None
        return rejection

    # ── internals ────────────────────────────────────────────────────────────

    def _start_thread(self, target) -> None:
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _write(self, frame: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("drivetrain link is not open — call connect() first")
        with self._write_lock:
            self._serial.write(frame)

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.wait(HEARTBEAT_INTERVAL_S):
            try:
                self._write(build_frame(CMD_HEARTBEAT))
            except Exception:
                log.exception("heartbeat failed — the board will e-stop within 500 ms")
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
                    log.exception("drivetrain read failed")
                return
            if frame is None:
                continue
            cmd, payload = frame
            if cmd == CMD_ACK:
                self._acks += 1
            elif cmd == CMD_NACK and len(payload) >= 2:
                self._last_rejection = Rejection(
                    cmd=payload[0],
                    reason=NACK_REASONS.get(payload[1], hex(payload[1])),
                    timestamp=time.monotonic(),
                )
            elif cmd == CMD_STATUS and payload:
                self._last_status = _parse_status(payload)
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
            log.warning("drivetrain response CRC mismatch, frame ignored")
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
    """[estopped] then, per side: [erpm 4B][current_ca 2B][v_in_dv 2B][fault][valid]."""
    body = payload[1:]
    wheels = {}
    for i in range(0, len(body) - 9, 10):
        side = i // 10
        wheels[WHEEL_SIDE_NAMES.get(side, side)] = WheelTelemetry(
            erpm=int.from_bytes(body[i : i + 4], "little", signed=True),
            current_a=int.from_bytes(body[i + 4 : i + 6], "little", signed=True) / 100.0,
            v_in=int.from_bytes(body[i + 6 : i + 8], "little", signed=True) / 10.0,
            fault_code=body[i + 8],
            valid=bool(body[i + 9]),
        )
    return Status(estopped=bool(payload[0]), wheels=wheels, timestamp=time.monotonic())
