"""The low ToF fan — two VL53L5CX 8x8 arrays at wheel height, looking forward.

Perception only, and the same contract as `stereo.py`: this module answers
"what is in front of me and how far" as an `ObstacleMap` over the same sector
grid, and stops there. It emits no motion and knows nothing about the STM32.
`avoid.Sensor` fuses its map with the cameras' (`sectors.fuse`), and `Avoider`
is not touched at all — which was the point of fusing at the map rather than at
the pixels.

## What these see that the cameras cannot

Two volumes, neither of which is a tuning problem:

* **Below the frame.** The stereo band is a crop and the ground-plane
  classification can only recover view the cameras actually deliver. Something
  3 cm off the floor and 20 cm ahead is under the lens, not in it.
* **Inside 0.25 m.** `STEREO_MIN_RANGE_M` is not a policy, it is a 60 mm
  baseline: disparity saturates and the cameras genuinely cannot triangulate
  closer. These parts range from about 2 cm.

They also work in the dark and against a blank wall, which is exactly where
SGBM is weakest — it has no texture to match and reports unknown, and unknown
blocks the robot.

## Where they are wired, and why it is the Jetson

The 40-pin I2C, one sensor per bus, which is the whole reason there is no
address dance: both parts boot at 0x29 and two buses means no LPn sequencing,
no GPIO, no mux, and no volatile address to re-apply after every power cycle.

Neither STM32 hosts them. The motion board cannot: ST's ULD driver carries an
~84 KB firmware blob it pushes over I2C at every power-on, into a part with
32 KB of flash and 12 KB of RAM. The drivetrain board has the silicon but not
the wire — `PROTO_MAX_PAYLOAD` is 24 bytes against 384 bytes of grid per
reading, and raising it is a wire-format change on the link that carries the
e-stop heartbeat. The full argument, including the one thing that WOULD belong
on the MCU (a collision cutout, which does not need these sensors to live
there), is in docs/avoidance-plan.md.

## Three things that are silent when they are wrong

1. **"No return" is not max range.** Every zone carries a `target_status`, and
   only a few values mean the distance is believed (`TOF_TRUSTED_STATUS`). A
   low-confidence zone flattened to 4.0 m reads to the policy as CLEAR, and the
   failure mode is a robot driving confidently into a dark sofa. Untrusted
   becomes NaN here and unknown in the map.
2. **A sensor pointing somewhere else measures somewhere else** while reporting
   the same numbers. The mount pose in config is a measurement, not a
   preference; the sensors are mounted LEVEL and see floor in their lower rows
   by design, and it is the height classification that removes it, not the
   mount angle.
3. **Zone 0 is the sensor's own top-left**, and which physical corner that is
   depends on how the breakout is turned on the bracket. `TOF_FLIP_H/V` fixes
   it in software once somebody has waved a hand at it in `tools/tof_debug.py`.

## Rates and startup — measured 2026-09-17, with the pair wired

The two buses are not equals, and everything below follows from that. Bus 7
runs at **400 kHz** and bus 1 at **100 kHz**, both read straight from the
device tree; bus 1 is the carrier board's own, shared with its USB-C PD
controller (0x25) and power monitor (0x40).

* **Startup: about 11.5 s for the pair**, and it is not a hang. The ~84 KB
  firmware blob takes **2.73 s** on bus 7 and **8.78 s** on bus 1 — four times
  as long for a quarter of the clock. The opens are sequential because parallel
  ones SEGFAULT (see `_OPEN_LOCK`), so the two costs add.
* **Rates: 15.3 Hz on bus 7, 4.7 Hz on bus 1.** The 8x8 grid caps at 15 Hz and
  the fast bus reaches it; the slow one cannot move ~1 KB of results per frame
  at 100 kHz and delivers a third of that. Each sensor therefore gets its own
  reader thread — polled from one, the slow sensor's `get_data()` blocks the
  fast one and BOTH fall to 4 Hz.
* **Staleness still has room, but less than the 67 ms this once claimed.** The
  fused map is stamped with the OLDER of the two sensors, so the fan's age is
  the slow sensor's ~213 ms frame period against a `TOF_STALE_S` of 0.5 s —
  roughly two frames of margin rather than seven. It holds, and it is worth
  knowing before anyone lowers that threshold or adds a third sensor to bus 1.

Raising bus 1 to 400 kHz would collapse both problems at once, and it is a
device-tree change on a bus two carrier-board drivers already own — a decision
to make deliberately, not a tweak to slip in.
"""

from __future__ import annotations

import logging
import threading
import time

from . import config
from .sectors import ObstacleMap, SectorGeometry, fill_clear, fuse

log = logging.getLogger(__name__)

# ST's per-zone verdict on its own measurement. 5 is a trusted range; 6 (target
# detected but no wrap-around check) and 9 (valid, large pulse) are usable with
# caveats. The rest — and 255 "no target" above all — mean UNKNOWN, which is
# never the same as clear. Kept as names rather than magic numbers because the
# whole safety argument of this module is in which ones are on the list.
STATUS_RANGE_VALID = 5
STATUS_RANGE_NOWRAP = 6
STATUS_RANGE_VALID_LARGE_PULSE = 9
STATUS_RANGE_NO_TARGET = 255

_IMPORT_HELP = (
    "the VL53L5CX driver is not installed. It is in the project's dependencies "
    "(`vl53l5cx-ctypes`, which builds ST's ULD from source and takes a few "
    "seconds on aarch64) — run `uv sync`. Set ORIO_TOF=0 to run without the "
    "ToF fan; obstacle avoidance then falls back to stereo alone."
)


# Constructing a VL53L5CX is NOT REENTRANT, and it fails by killing the
# process rather than by raising. Two threads inside the ctypes wrapper's
# `__init__` segfault the interpreter — reproduced 3/3 on 2026-09-17 with the
# pair wired, both threads dying in `is_alive()`, which is the first call that
# drives the I2C callbacks and comes BEFORE the firmware upload starts. Neither
# locking only the constructor nor deferring `init()` with `skip_init=True`
# helped: the upload crashes in parallel too.
#
# This matters more than an ordinary race because `avoid.Sensor.start()` wraps
# the ToF open in `try/except` precisely so a missing fan degrades to
# stereo-only — and a SIGSEGV is not an exception. It takes the robot with it,
# past every guard written to stop exactly that.
#
# Module-level rather than per-detector, because the unsafety is in the driver:
# a bench tool opening a third sensor beside a running `Body` would hit the
# same thing. Steady-state reads were measured safe from separate threads in
# the same session, so this covers the open ONLY and `ToFDetector` still gives
# each sensor its own reader.
_OPEN_LOCK = threading.Lock()


def _driver():
    """The ctypes wrapper around ST's ULD, imported lazily.

    Lazy for the same reason the camera imports are: `config` and this module
    are imported by things that will never open a sensor, and a missing
    optional driver must not be an import error at the top of the app.
    """
    try:
        import vl53l5cx_ctypes
    except ImportError as exc:  # pragma: no cover - environment-dependent
        raise RuntimeError(_IMPORT_HELP) from exc
    return vl53l5cx_ctypes


def zone_angles(rows: int, cols: int, fov_deg: float, flip_h: bool = False,
                flip_v: bool = False):
    """Azimuth and elevation of each zone centre, in the sensor's own frame.

    Row-major from the sensor's top-left, which is the order the ULD fills
    `distance_mm`. Azimuth is positive to the RIGHT and elevation positive UP,
    matching `SectorGeometry`.

    The zones are treated as equal ANGULAR spans across `fov_deg`, which is how
    ST specifies the array (45 x 45 deg for the 8x8 grid, so 5.625 deg per
    zone). These are the zone CENTRE ANGLES only — turning them plus a reported
    distance into a point is `ToFArray`'s job, and it is not the one line it
    looks like. See `ToFArray.__init__`: the ULD reports distance along the
    optical axis, not along the zone's own line of sight.
    """
    import numpy as np

    step_az = fov_deg / cols
    step_el = fov_deg / rows
    c = np.arange(cols) - (cols - 1) / 2.0
    r = np.arange(rows) - (rows - 1) / 2.0
    az = c * step_az
    el = -r * step_el  # row 0 is the TOP of the grid, which is the highest
    if flip_h:
        az = az[::-1]
    if flip_v:
        el = el[::-1]
    az_grid, el_grid = np.meshgrid(az, el)
    return az_grid.ravel(), el_grid.ravel()


class ToFSensor:
    """One VL53L5CX: lazy open, a lock, explicit `close()`.

    `read()` returns ranges in metres with NaN wherever the sensor did not
    vouch for the zone, or `None` when no new frame is ready yet — the 15 Hz
    grid is slower than anything that will poll it, and returning a stale frame
    as though it were new is how a frozen sensor becomes invisible.
    """

    def __init__(self, bus: int, address: int = 0x29, name: str = "tof",
                 resolution: int = config.TOF_RESOLUTION,
                 freq_hz: int = config.TOF_FREQ_HZ) -> None:
        self.bus = bus
        self.address = address
        self.name = name
        self.resolution = resolution
        self.freq_hz = freq_hz
        self.rows = self.cols = int(round(resolution ** 0.5))
        if self.rows * self.cols != resolution:
            raise ValueError(f"resolution {resolution} is not a square grid")
        self._dev = None
        self._smbus = None
        self._lock = threading.Lock()

    def open(self) -> None:
        """Push the firmware and start ranging. Seconds, not milliseconds."""
        with self._lock:
            if self._dev is not None:
                return
            vl53 = _driver()
            from smbus2 import SMBus

            started = time.monotonic()
            smbus = SMBus(self.bus)
            try:
                # _OPEN_LOCK, not politeness: see its comment above. Everything
                # up to start_ranging() is serialised against every other
                # sensor in the process.
                with _OPEN_LOCK:
                    # The constructor is what uploads ST's ~84 KB blob, and it
                    # raises rather than returning a dud handle if nothing
                    # answers at the address.
                    dev = vl53.VL53L5CX(i2c_addr=self.address, i2c_dev=smbus)
                    dev.set_resolution(self.resolution)
                    dev.set_ranging_frequency_hz(self.freq_hz)
                    # Closest, not strongest: this is a collision guard, so a
                    # dim near return beats a bright far one every time.
                    dev.set_target_order(vl53.TARGET_ORDER_CLOSEST)
                    dev.start_ranging()
            except Exception:
                smbus.close()
                raise
            self._smbus = smbus
            self._dev = dev
            log.info(
                "%s: VL53L5CX ready on /dev/i2c-%d at 0x%02x (%dx%d @ %d Hz, "
                "firmware upload took %.1f s)",
                self.name, self.bus, self.address, self.rows, self.cols,
                self.freq_hz, time.monotonic() - started,
            )

    def read(self):
        """`(ranges_m, statuses)` for one fresh frame, or `None` if not ready.

        Ranges are NaN for every zone whose `target_status` is not trusted and
        every zone outside the range gate. That is deliberate and it is the
        whole safety property of this module: unknown must arrive at the policy
        AS unknown, not as a comfortable four metres.
        """
        import numpy as np

        with self._lock:
            if self._dev is None:
                raise RuntimeError(f"{self.name}: read() before open()")
            if not self._dev.data_ready():
                return None
            data = self._dev.get_data()
            n = self.resolution
            # ctypes gives [target][zone]; one target per zone is the ULD's
            # build-time NB_TARGET_PER_ZONE and this reads the first.
            distance_mm = np.array(data.distance_mm[0][:n], dtype=np.float32)
            status = np.array(data.target_status[0][:n], dtype=np.uint8)

        ranges = distance_mm / 1000.0
        trusted = np.isin(status, list(config.TOF_TRUSTED_STATUS))
        ranges[~trusted] = np.nan
        ranges[(ranges < config.TOF_MIN_RANGE_M) | (ranges > config.TOF_MAX_RANGE_M)] = np.nan
        return ranges, status

    def close(self) -> None:
        with self._lock:
            if self._dev is not None:
                try:
                    self._dev.stop_ranging()
                except Exception:  # pragma: no cover - a dead bus on the way out
                    log.debug("%s: stop_ranging failed on close", self.name, exc_info=True)
                self._dev = None
            if self._smbus is not None:
                self._smbus.close()
                self._smbus = None


class ToFArray:
    """A `ToFSensor` plus the mount pose that turns its zones into robot-frame
    points. One of these per physical sensor."""

    def __init__(self, sensor: ToFSensor, *, height_m: float, pitch_deg: float,
                 yaw_deg: float, sectors: int = config.STEREO_SECTORS,
                 hfov_deg: float = config.STEREO_HFOV_DEG) -> None:
        self.sensor = sensor
        self.height_m = height_m
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        import numpy as np

        az, el = zone_angles(sensor.rows, sensor.cols, config.TOF_FOV_DEG,
                             config.TOF_FLIP_H, config.TOF_FLIP_V)
        self.az_deg, self.el_deg = az, el

        # THE ULD REPORTS DISTANCE ALONG THE OPTICAL AXIS, NOT ALONG THE ZONE'S
        # OWN LINE OF SIGHT, and the array behaves as a pinhole rather than as a
        # fan of equally-spaced rays. Both were measured, not assumed: against a
        # flat wall at 0.86 m the line-of-sight reading leaves a textbook
        # bullseye in the plane residual — dead flat at the edges and 44 mm
        # proud in the middle, 25 mm RMS — and treating the same numbers as
        # axial collapses that structure to 0.4 mm and 3.3 mm RMS, which is the
        # sensor's own noise (2026-09-17, both sensors).
        #
        # Uncorrected, a corner zone's range comes out 12% SHORT and its
        # elevation 1.1 deg too high. Twelve per cent reads as an obstacle
        # nearer than it is, which sounds like the safe direction and is not:
        # this feeds the height-above-floor classification, where a misplaced
        # floor point at the edge of the grid becomes an obstacle and the robot
        # refuses to cruise down an empty corridor.
        #
        # `stereo.py:_ground_geometry` has always done exactly this for the
        # depth map — pinhole ray, then `depth * norm` to get line-of-sight
        # range. This is the same two lines, and the two sensors now agree on
        # what a point is.
        tx = np.tan(np.radians(az))
        ty = np.tan(np.radians(el))
        norm = np.sqrt(tx * tx + ty * ty + 1.0)
        # Axial -> line-of-sight, applied to every range in `reduce`.
        self.range_scale = norm.astype(np.float32)

        # Fixed at construction: the zones do not move and neither does the
        # bracket, so all of the trigonometry is done exactly once, here.
        self.geometry = SectorGeometry(
            np.degrees(np.arctan2(tx, 1.0)),
            np.degrees(np.arcsin(ty / norm)),
            height_m=height_m, pitch_deg=pitch_deg, yaw_deg=yaw_deg,
            sectors=sectors, hfov_deg=hfov_deg,
        )

    @property
    def name(self) -> str:
        return self.sensor.name

    def sense(self) -> ObstacleMap | None:
        """One frame reduced to sectors, or `None` if no frame was ready."""
        frame = self.sensor.read()
        return None if frame is None else self.reduce(frame[0])

    def reduce(self, ranges) -> ObstacleMap:
        """Ranges already read -> sector map.

        Split from `sense()` so a caller holding a frame can reduce THAT frame
        rather than fetching another: `tools/tof_debug.py` draws the grid and
        the sectors side by side, and they have to be the same reading or the
        view is comparing two different moments.

        `ranges` are the ULD's axial distances; `range_scale` turns them into
        the line-of-sight ranges `SectorGeometry` is defined on.
        """
        return ObstacleMap(
            sectors=self.geometry.reduce(
                ranges * self.range_scale,
                floor_tol_m=config.STEREO_FLOOR_TOL_M,
                ceiling_m=config.ROBOT_HEIGHT_M,
                min_valid_frac=config.STEREO_MIN_VALID_FRAC,
                source=self.name,
            ),
            timestamp=time.time(),
            # A ToF reports metres natively. There is no calibration to be
            # missing, which is a real advantage over the cameras and the reason
            # this does not propagate stereo's uncertainty flag.
            calibrated=True,
        )


def arrays_from_config(sectors: int = config.STEREO_SECTORS,
                       hfov_deg: float = config.STEREO_HFOV_DEG) -> list[ToFArray]:
    """The pair as configured on the robot — buses, addresses and mount poses."""
    n = len(config.TOF_BUSES)
    for name, values in (
        ("ORIO_TOF_ADDRESSES", config.TOF_ADDRESSES),
        ("ORIO_TOF_NAMES", config.TOF_NAMES),
        ("ORIO_TOF_HEIGHTS_M", config.TOF_HEIGHTS_M),
        ("ORIO_TOF_PITCHES_DEG", config.TOF_PITCHES_DEG),
        ("ORIO_TOF_YAWS_DEG", config.TOF_YAWS_DEG),
    ):
        if len(values) != n:
            raise ValueError(
                f"{name} has {len(values)} entries but ORIO_TOF_BUSES has {n}. "
                "Every sensor needs a bus, an address, a name and a measured "
                "mount pose — a missing one would silently inherit its "
                "neighbour's geometry."
            )
    return [
        ToFArray(
            ToFSensor(bus=bus, address=addr, name=name),
            height_m=h, pitch_deg=p, yaw_deg=y, sectors=sectors, hfov_deg=hfov_deg,
        )
        for bus, addr, name, h, p, y in zip(
            config.TOF_BUSES, config.TOF_ADDRESSES, config.TOF_NAMES,
            config.TOF_HEIGHTS_M, config.TOF_PITCHES_DEG, config.TOF_YAWS_DEG,
        )
    ]


class ToFDetector:
    """The pair as one source: `start()`, read `.map`, `close()`.

    Mirrors `stereo.ObstacleDetector`'s shape, with one deliberate difference —
    it publishes from its OWN thread rather than being polled synchronously.
    The reason is the same one that put the stereo pipeline on a thread: a
    sensor that wedges must not wedge the loop holding the robot's watchdog
    alive. A stalled reader simply stops republishing, `.map` ages, and
    `avoid.Sensor` drops it from the fusion and carries on with the cameras.

    It also means the two sensors' 15 Hz grids never block the 30 Hz stereo
    loop waiting for a frame that is not ready yet.
    """

    def __init__(self, arrays: list[ToFArray] | None = None,
                 sectors: int = config.STEREO_SECTORS,
                 hfov_deg: float = config.STEREO_HFOV_DEG) -> None:
        self._arrays = arrays if arrays is not None else arrays_from_config(sectors, hfov_deg)
        self._map: ObstacleMap | None = None
        # When the fan last had something to say, on the MONOTONIC clock. This
        # and not the map's own timestamp is what liveness is measured against
        # — see `fresh_map`.
        self._published_at: float | None = None
        self._latest: dict[str, ObstacleMap] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        # Per-sensor, because there is now one reader thread per sensor and a
        # shared error string would have each of them erasing the other's.
        self._errors: dict[str, str] = {}
        self._open_error: str | None = None

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(a.name for a in self._arrays)

    @property
    def error(self) -> str | None:
        """Every sensor's current complaint, joined — `None` when all is well.

        A property rather than an attribute because the readers are one thread
        per sensor: a shared `self.error = None` at the top of each loop would
        have each sensor cheerfully clearing the other's failure, and the one
        that matters — a bus that has stopped answering while its neighbour is
        fine — is exactly the one that would vanish.
        """
        parts = [self._open_error] if self._open_error else []
        with self._lock:
            parts += [v for v in self._errors.values() if v]
        return "; ".join(parts) or None

    def start(self) -> None:
        """Open every sensor, then start publishing. Raises if none opened.

        The opens are SEQUENTIAL and that is a hardware finding, not a
        preference — see `_OPEN_LOCK`. It costs the two firmware uploads end to
        end: measured 2.73 s on bus 7 and 8.78 s on bus 1 (2026-09-17), the
        difference being the BUS CLOCK rather than the part. Bus 7 runs at
        400 kHz and bus 1 at 100 kHz, both straight from the device tree, and
        an ~84 KB blob at a quarter of the clock takes four times as long.
        Raising bus 1 is a device-tree change on a bus shared with the carrier
        board's USB-C PD controller and power monitor, so it is a decision
        rather than a tweak.

        A sensor that fails is left out and the rest carry on — half a fan is
        still more than the cameras had.
        """
        failed: dict[str, BaseException] = {}
        opened: list[ToFArray] = []
        for array in self._arrays:
            try:
                array.sensor.open()
                opened.append(array)
            except BaseException as exc:  # noqa: BLE001 - reported, not swallowed
                failed[array.name] = exc

        self._arrays = opened
        if failed:
            self._open_error = "; ".join(
                f"{n}: {type(e).__name__}: {e}" for n, e in failed.items()
            )
        if not self._arrays:
            raise RuntimeError(
                f"no ToF sensor opened ({self.error}). Check the wiring against "
                "docs/avoidance-plan.md section 3, then `i2cdetect -y -r "
                f"{config.TOF_BUSES[0]}` — a VL53L5CX answers at 0x29."
            )

        # One reader per sensor, not one loop over both. The buses run at
        # different clocks, so a shared thread spends most of its time inside
        # the slow sensor's ~1 KB `get_data()` and drags the fast one down to
        # match: measured 15.3 Hz and 4.7 Hz on their own threads, against
        # 4 Hz EACH when polled from one. What that buys is detection latency
        # on the sensor that can deliver it, and reads from separate threads
        # were measured safe where opens were not.
        self._threads = [
            threading.Thread(target=self._read_loop, args=(a,),
                             name=f"tof-{a.name}", daemon=True)
            for a in self._arrays
        ]
        for t in self._threads:
            t.start()

    def _read_loop(self, array: ToFArray) -> None:
        """One sensor's reader. Reads, stores, republishes the fused pair."""
        # Poll rather than wire up INT/GPIO1: at these rates the saving is two
        # GPIOs and some solder against a few hundred microseconds of I2C per
        # miss.
        period = 1.0 / max(config.TOF_FREQ_HZ * 4, 1)
        while not self._stop.is_set():
            try:
                omap = array.sense()
                if omap is not None:
                    with self._lock:
                        self._latest[array.name] = omap
                        self._errors.pop(array.name, None)
                    self._republish()
            except Exception as exc:  # noqa: BLE001 - a failing bus must not spin
                with self._lock:
                    self._errors[array.name] = f"{type(exc).__name__}: {exc}"
                self._stop.wait(0.5)
                continue
            self._stop.wait(period)

    def _republish(self) -> None:
        """Fuse the per-sensor maps into one, over the shared sector grid.

        The pair is fused here rather than in `avoid.Sensor` so that what leaves
        this module is one map from one source — the fan — and the sensors'
        splay stays this module's business. `min()` per sector, same rule as
        everywhere else: the two overlap in the middle sectors and the pessimist
        wins there.

        Staleness is applied PER SENSOR, here, and that is not a refinement —
        it is what keeps the fan usable on this robot. `fuse` stamps with the
        oldest map it is given, so one quiet sensor drags the whole fan's
        timestamp past `TOF_STALE_S` and `fresh_map()` withholds BOTH. That is
        not hypothetical: the bus-1 sensor is GIL-starved by its faster
        neighbour and was measured going 900 ms between frames inside this
        detector, against 329 ms when nothing else was running (2026-09-17).
        Under the old rule the entire fan blinked out for those stretches and
        avoidance silently fell back to stereo — blind low and blind inside
        0.25 m, which is the whole reason these parts are here.

        Dropping only the stale sensor degrades where the fault is. Its sectors
        go UNKNOWN rather than clear, stereo still covers them through the
        fusion in `avoid.Sensor`, and the sensor that is still talking keeps
        the volume no camera reaches.
        """
        now = time.time()
        with self._lock:
            maps = [
                self._latest[a.name]
                for a in self._arrays
                if a.name in self._latest
                and now - self._latest[a.name].timestamp <= config.TOF_STALE_S
            ]
        # Nothing fresh: publish nothing, and let the last map age out of
        # `fresh_map()` on its own. Republishing a stale fusion under a new
        # timestamp would be the one unforgivable version of this function.
        if not maps:
            return
        # Fused OUTSIDE the lock: `fuse` is pure and the readers must not queue
        # behind each other's arithmetic.
        fused = fill_clear(fuse(*maps))
        with self._lock:
            self._map = fused
            self._published_at = time.monotonic()

    @property
    def map(self) -> ObstacleMap | None:
        """The latest fused map, or `None` before the first frame."""
        with self._lock:
            return self._map

    def fresh_map(self, max_age_s: float = config.TOF_STALE_S) -> ObstacleMap | None:
        """The latest map if it is recent enough to fuse, else `None`.

        The age gate lives here rather than in the policy on purpose: a ToF that
        has gone quiet must DROP OUT of the fusion, leaving stereo to decide
        alone, and must not age the fused map into `AVOID_STALE_S` and halt the
        robot. A new sensor that can stop the robot is a new way for the robot
        to be stopped, and this one is an addition to a guard that already works.

        It asks WHEN THE FAN LAST PUBLISHED, not how old the map's own
        timestamp is, and the difference is not pedantry. `fuse` stamps with
        the oldest contributing sensor, so a map built from a reading 0.49 s
        old is itself stamped 0.49 s old — and against a 0.5 s gate the fan
        would blink out on arithmetic alone, while both sensors were ranging
        happily. `_republish` has already dropped every contributor older than
        `TOF_STALE_S`, so anything published is by construction made of fresh
        readings; what is left to ask is whether publishing is still happening
        at all. When both sensors go quiet it stops, this returns `None`, and
        stereo decides alone.

        The map keeps its honest oldest-contributor timestamp, because that is
        the right answer for anyone fusing it further.
        """
        with self._lock:
            omap, published = self._map, self._published_at
        if omap is None or published is None:
            return None
        if time.monotonic() - published > max_age_s:
            return None
        return omap

    def close(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        for array in self._arrays:
            array.sensor.close()
