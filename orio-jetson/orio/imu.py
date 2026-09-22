"""The body's attitude, from a BNO085 streaming UART-RVC.

Perception only: this module reads, it never commands. The sensor is wired
straight to the Jetson's 40-pin UART (`/dev/ttyTHS1`) and is strapped into
UART-RVC mode, where it does nothing but transmit a fixed 19-byte packet at
100 Hz — no protocol, no configuration, nothing to send it.

## What a reading means on THIS robot

The datasheet's axis names do not survive this mount and must not be used.
The BNO085 sits ~35 cm above the drive axle with its **+Y axis pointing
forward**, and with that mounting the datasheet's "pitch is rotation about Y"
(p.21) is not what the part reports. Measured on the robot 2026-09-22 by
tipping it and watching both the angles and the accelerometer:

  * **pitch** is the nose going up/down, and **ay** moves with it
  * **roll** is the body leaning left/right, and **ax** moves with it
  * **+Z is up**: `az` reads about +985 mg sitting still

so `Reading` names its fields for the body, not for the part. Signs and the
rest-pose offsets come from `tools/imu_calibrate.py` — this module applies
them, it does not guess them.

## Yaw is a heading for a manoeuvre, not for a session

Yaw is measured from power-on, not from north (UART-RVC carries no magnetic
reference, and two hub motors would ruin one anyway). It held to 0.03 deg over
55 s sitting still, so it is trustworthy across a single turn; re-zero at the
start of each manoeuvre rather than trusting it to mean anything an hour
later. `zero_heading()` exists for exactly that.

## Nothing here is safety-critical

A missing or stale IMU does not stop the robot: the turns fall back to being
timed, the way they were before this sensor existed. That is the opposite of
`stereo.py`, where no reading means no driving — the difference is that sight
is what makes driving safe, and this only makes it accurate.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass

log = logging.getLogger(__name__)

# The packet: 0xAAAA, index, yaw/pitch/roll (int16, 0.01 deg), x/y/z accel
# (int16, mg), MI, MR, reserved, checksum. Datasheet p.21-22.
HEADER = b"\xaa\xaa"
PACKET_LEN = 19
_BODY = struct.Struct("<Bhhhhhh")  # index + 6 int16, the first 13 body bytes

BAUD = 115200
RATE_HZ = 100.0


@dataclass(frozen=True)
class Reading:
    """One packet, named for the body rather than for the part.

    Angles are degrees after the mount offsets have been taken out; `yaw` is
    relative to the last `zero_heading()`. Accelerations stay in the sensor's
    own axes, in mg, because every use of them (contact spikes, lift) is about
    a change rather than a direction.
    """

    heading_deg: float  # + is a left turn, once IMU_YAW_SIGN is set
    pitch_deg: float  # + is nose up
    roll_deg: float  # + is leaning right
    ax_mg: float
    ay_mg: float
    az_mg: float
    index: int
    timestamp: float

    @property
    def accel_mg(self) -> float:
        """Total specific force. ~1000 mg at rest, ~0 in free fall."""
        return (self.ax_mg**2 + self.ay_mg**2 + self.az_mg**2) ** 0.5


def parse(packet: bytes) -> tuple[int, float, float, float, int, int, int] | None:
    """One 19-byte packet -> (index, yaw, pitch, roll, ax, ay, az), or None.

    Angles in degrees, accelerations in mg. None means the checksum failed,
    which is the same rule the STM32 links use: a bad frame is no frame, never
    a frame with a guess in it.
    """
    if len(packet) != PACKET_LEN or not packet.startswith(HEADER):
        return None
    body = packet[2:18]
    if sum(body) & 0xFF != packet[18]:
        return None
    index, yaw, pitch, roll, ax, ay, az = _BODY.unpack(body[:13])
    return index, yaw / 100.0, pitch / 100.0, roll / 100.0, ax, ay, az


class RvcReader:
    """An open serial port with a thread draining it into the latest reading.

    Use as a context manager. `reading` is whatever arrived most recently, or
    None if nothing has yet; it carries its own timestamp so callers can decide
    for themselves what counts as too old. Bad checksums and short reads are
    counted rather than raised — at 100 Hz, one dropped packet is not an event.
    """

    def __init__(
        self,
        port: str = "/dev/ttyTHS1",
        baud: int = BAUD,
        pitch_offset_deg: float = 0.0,
        roll_offset_deg: float = 0.0,
        pitch_sign: int = 1,
        roll_sign: int = 1,
        yaw_sign: int = 1,
        on_reading=None,
    ) -> None:
        # `on_reading` sees EVERY packet, in the reader thread. Polling `reading`
        # misses most of them: a serial read returns several packets at once, so
        # a poller only ever sees the last of each burst (~30 of every 100). Live
        # driving wants the latest and nothing else; anything measuring — the
        # calibration tool, a drift log — needs them all.
        self._on_reading = on_reading
        self._port_name = port
        self._baud = baud
        self._pitch_offset = pitch_offset_deg
        self._roll_offset = roll_offset_deg
        self._pitch_sign = pitch_sign
        self._roll_sign = roll_sign
        self._yaw_sign = yaw_sign

        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._reading: Reading | None = None
        self._yaw_zero = 0.0

        self.packets = 0
        self.bad_checksums = 0
        self.dropped = 0  # gaps in the sensor's own index counter
        self._last_index: int | None = None

    def __enter__(self) -> RvcReader:
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        import serial

        self._serial = serial.Serial(self._port_name, self._baud, timeout=0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="imu-rvc", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    @property
    def reading(self) -> Reading | None:
        with self._lock:
            return self._reading

    def fresh(self, max_age_s: float, now: float | None = None) -> Reading | None:
        """The latest reading if it is younger than `max_age_s`, else None.

        Callers fall back to timed behaviour on None. They do not stop.
        """
        reading = self.reading
        if reading is None:
            return None
        now = time.monotonic() if now is None else now
        return reading if (now - reading.timestamp) <= max_age_s else None

    def zero_heading(self) -> bool:
        """Make the current heading 0. Returns False if there is nothing to zero.

        Called at the start of a manoeuvre, because yaw is only meaningful
        against a recent reference — see the module docstring.
        """
        with self._lock:
            if self._reading is None:
                return False
            self._yaw_zero += self._reading.heading_deg
            return True

    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._serial.read(64)
            except Exception:
                log.exception("IMU serial read failed; reader stopping")
                return
            if not chunk:
                continue
            buf += chunk
            # Resync on the header every time rather than trusting alignment:
            # the stream is free-running, so the first read lands mid-packet.
            while len(buf) >= PACKET_LEN:
                start = buf.find(HEADER)
                if start < 0:
                    buf = buf[-1:]
                    break
                if start:
                    buf = buf[start:]
                    continue
                if len(buf) < PACKET_LEN:
                    break
                fields = parse(buf[:PACKET_LEN])
                if fields is None:
                    self.bad_checksums += 1
                    buf = buf[1:]  # not a real packet boundary; hunt on
                    continue
                self._publish(fields)
                buf = buf[PACKET_LEN:]

    def _publish(self, fields) -> None:
        index, yaw, pitch, roll, ax, ay, az = fields
        self.packets += 1
        if self._last_index is not None:
            gap = (index - self._last_index) % 256 - 1
            if gap > 0:
                self.dropped += gap
        self._last_index = index

        reading = Reading(
            heading_deg=_wrap180(self._yaw_sign * yaw - self._yaw_zero),
            pitch_deg=self._pitch_sign * (pitch - self._pitch_offset),
            roll_deg=self._roll_sign * (roll - self._roll_offset),
            ax_mg=float(ax),
            ay_mg=float(ay),
            az_mg=float(az),
            index=index,
            timestamp=time.monotonic(),
        )
        with self._lock:
            self._reading = reading
        if self._on_reading is not None:
            try:
                self._on_reading(reading)
            except Exception:
                # A broken consumer must not take the reader down with it.
                log.exception("IMU on_reading callback failed")


def _wrap180(deg: float) -> float:
    """Fold an angle into [-180, 180). Yaw wraps; every difference must."""
    return (deg + 180.0) % 360.0 - 180.0
