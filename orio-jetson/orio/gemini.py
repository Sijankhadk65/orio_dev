"""The Orbbec Gemini 336L: Orio's one camera, for depth and colour both.

A single USB3 device replaces the IMX219-83 CSI pair. The difference that
shapes this module is where depth comes from: the Gemini computes it ON THE
CAMERA (Orbbec's MX6800 ASIC, active stereo on a 95 mm baseline) and hands the
Jetson finished metres. There is no matcher to run, no calibration to produce,
no eye/sensor mapping to get backwards, and no exposure drift between two
independently auto-exposing sensors. The factory calibration is the calibration.

It also aligns depth to colour in hardware (D2C), so the colour frame and the
depth map come out pixel-for-pixel on the same grid. That is what lets the
obstacle map and the object detector share one picture: a YOLO box drawn on the
colour frame covers exactly the depth pixels behind it.

## One camera, many readers

`GeminiCamera` owns the SDK pipeline and a grab thread that keeps only the
latest frameset. Readers never call into the SDK themselves: `read()` returns
the newest `Frames` published after the sequence number the caller last saw.
That is what makes it safe for the avoidance thread and the vision tool to
share the device — two callers pulling on `wait_for_frames` would split the
stream between them, and each would see half the frame rate.

`shared()` is the process-wide instance. Everything on the robot goes through
it; constructing a second `GeminiCamera` against the same device fails at
`start()`, because the SDK will not open one device twice.

Datasheet: `orbbec_gemini_330_series.pdf` in the knowledge base (p.9-11 specs,
p.39 coordinate frames, p.72 the 336L mechanical drawing).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from . import config

log = logging.getLogger(__name__)

# Orbbec's USB vendor id, for error messages pointing at `lsusb`.
ORBBEC_VID = 0x2BC5


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics of the frame `read()` returns, in its own pixels.

    After D2C the depth map lives on the COLOUR camera's grid, so these are the
    colour intrinsics, scaled to the configured resolution by the SDK.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @property
    def hfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.width / 2.0 / self.fx))

    @property
    def vfov_deg(self) -> float:
        return 2.0 * math.degrees(math.atan(self.height / 2.0 / self.fy))


@dataclass(frozen=True)
class Frames:
    """One synchronised capture.

    `color` is BGR uint8 (H, W, 3). `depth` is float32 metres (H, W), NaN where
    the camera reported nothing or the value fell outside the configured range
    gate — unknown must never read as a distance. The two share one grid.
    """

    color: object
    depth: object
    timestamp: float  # time.monotonic() when the frameset arrived
    seq: int


def _to_bgr(frame, width: int, height: int):
    """Colour frame -> BGR ndarray, whatever format the stream negotiated."""
    import cv2
    import numpy as np
    import pyorbbecsdk as ob

    data = np.asanyarray(frame.get_data())
    fmt = frame.get_format()
    if fmt == ob.OBFormat.RGB:
        return cv2.cvtColor(data.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if fmt == ob.OBFormat.BGR:
        return data.reshape(height, width, 3).copy()
    if fmt in (ob.OBFormat.YUYV, ob.OBFormat.YUY2):
        return cv2.cvtColor(data.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUY2)
    if fmt == ob.OBFormat.MJPG:
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("Gemini: could not decode an MJPG colour frame")
        return img
    raise RuntimeError(f"Gemini: unsupported colour format {fmt}")


def route_sdk_log(ob) -> None:
    """Keep the SDK's log file in one place.

    Left alone, the SDK writes `Log/OrbbecSDK.log.txt` into whatever directory
    the process happens to run from, so every tool run from a new cwd leaves a
    fresh `Log/` behind. Warnings and up go to `orio-jetson/Log/` (gitignored).
    """
    log_dir = config.ROOT / "Log"
    log_dir.mkdir(exist_ok=True)
    ob.Context.set_logger_to_file(ob.OBLogLevel.WARNING, str(log_dir))


class GeminiCamera:
    """Depth + colour from the Gemini 336L, depth aligned onto the colour grid.

    Lazy: nothing touches USB until `start()` (or the first `read()`).
    """

    # Colour formats to ask for, best first. RGB costs one cheap channel swap;
    # MJPG is last because decoding it is real CPU on the Jetson.
    _COLOR_FORMATS = ("RGB", "BGR", "YUYV", "MJPG")

    def __init__(
        self,
        width: int = config.GEMINI_WIDTH,
        height: int = config.GEMINI_HEIGHT,
        fps: int = config.GEMINI_FPS,
    ) -> None:
        self._width = width
        self._height = height
        self._fps = fps
        self._pipeline = None
        self._align = None  # software D2C filter, only if hardware D2C is unavailable
        self._filters: list = []  # host-side depth filters, applied in order
        self.intrinsics: Intrinsics | None = None
        self.device_name = ""
        self.depth_setup = ""
        self._latest: Frames | None = None
        self._error: Exception | None = None
        self._cond = threading.Condition()
        self._start_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ── opening ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        with self._start_lock:
            if self._thread is not None:
                return
            self._open()
            self._stop.clear()
            self._thread = threading.Thread(target=self._grab_loop, name="gemini", daemon=True)
            self._thread.start()

    def _color_profile(self, pipeline):
        import pyorbbecsdk as ob

        profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        for name in self._COLOR_FORMATS:
            try:
                return profiles.get_video_stream_profile(
                    self._width, self._height, getattr(ob.OBFormat, name), self._fps
                )
            except Exception:  # noqa: BLE001 - the SDK raises when no profile matches
                continue
        raise RuntimeError(
            f"Gemini: no colour stream at {self._width}x{self._height} @ {self._fps} fps "
            "in any supported format. Run tools/gemini_check.py to list what the "
            "camera offers, and set ORIO_GEMINI_WIDTH/HEIGHT/FPS to one of them."
        )

    def _depth_profile(self, pipeline, color_profile):
        """(profile, hardware_d2c) — the depth stream to pair with `color_profile`.

        Hardware D2C only works for depth modes the camera can align onto that
        colour mode, and the SDK will list exactly those. If there are none (or
        config asks for software), fall back to the SDK's AlignFilter on the host.
        """
        import pyorbbecsdk as ob

        if config.GEMINI_HW_ALIGN:
            d2c = pipeline.get_d2c_depth_profile_list(color_profile, ob.OBAlignMode.HW_MODE)
            candidates = [
                d2c.get_stream_profile_by_index(i).as_video_stream_profile()
                for i in range(d2c.get_count())
            ]
            candidates = [
                p for p in candidates
                if p.get_format() == ob.OBFormat.Y16 and p.get_fps() == self._fps
            ]
            if candidates:
                # Closest to the colour size: more depth pixels than the colour
                # grid can hold is wasted USB bandwidth, fewer is lost detail.
                best = min(candidates, key=lambda p: abs(p.get_width() - self._width))
                return best, True
            log.warning("Gemini: no hardware D2C depth mode for this colour mode — "
                        "aligning in software instead")

        profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        try:
            return profiles.get_video_stream_profile(
                self._width, self._height, ob.OBFormat.Y16, self._fps
            ), False
        except Exception:  # noqa: BLE001
            return profiles.get_default_video_stream_profile(), False

    def _open(self) -> None:
        try:
            import pyorbbecsdk as ob
        except ImportError as exc:
            raise RuntimeError(
                "pyorbbecsdk is not installed — run `uv sync` in orio-jetson"
            ) from exc

        route_sdk_log(ob)
        ctx = ob.Context()
        if ctx.query_devices().get_count() == 0:
            raise RuntimeError(
                "no Orbbec camera found. Check `lsusb | grep -i 2bc5` — nothing there "
                "means a cable or port problem (it wants a USB3 port); present but not "
                "found here usually means the udev rule is missing, see "
                "udev/99-orbbec.rules."
            )

        pipeline = ob.Pipeline()
        device = pipeline.get_device()
        info = device.get_device_info()
        self.device_name = f"{info.get_name()} (fw {info.get_firmware_version()})"
        self._configure_depth(device)

        color = self._color_profile(pipeline)
        depth, hw_d2c = self._depth_profile(pipeline, color)

        cfg = ob.Config()
        cfg.enable_stream(color)
        cfg.enable_stream(depth)
        if hw_d2c:
            cfg.set_align_mode(ob.OBAlignMode.HW_MODE)
        else:
            self._align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
        # Deliver colour and depth together, or not at all: a frameset missing
        # one of them is useless to anything that pairs pixels with metres.
        pipeline.enable_frame_sync()
        pipeline.start(cfg)
        self._pipeline = pipeline

        i = color.get_intrinsic()
        # The SDK reports intrinsics at the profile's own resolution, which is
        # the one configured — but scale defensively in case it ever doesn't.
        sx = self._width / float(i.width) if i.width else 1.0
        sy = self._height / float(i.height) if i.height else 1.0
        self.intrinsics = Intrinsics(
            fx=i.fx * sx, fy=i.fy * sy, cx=i.cx * sx, cy=i.cy * sy,
            width=self._width, height=self._height,
        )
        drift = abs(self.intrinsics.hfov_deg - config.STEREO_HFOV_DEG)
        if drift > 2.0:
            # The ToF fan and the bump memory lay their sectors out on
            # STEREO_HFOV_DEG so the three maps fuse sector-for-sector. If the
            # camera's real field of view disagrees, the camera's sectors are
            # somewhere else than theirs.
            log.warning(
                "Gemini hfov is %.1f deg but STEREO_HFOV_DEG is %.1f — set "
                "ORIO_STEREO_HFOV_DEG to match, or the ToF and camera sectors will "
                "not line up", self.intrinsics.hfov_deg, config.STEREO_HFOV_DEG,
            )
        log.info(
            "Gemini open: %s, colour %dx%d %s + depth %dx%d, %s D2C, hfov %.1f deg; %s",
            self.device_name, self._width, self._height, color.get_format(),
            depth.get_width(), depth.get_height(), "hardware" if hw_d2c else "software",
            self.intrinsics.hfov_deg, self.depth_setup,
        )

    def _configure_depth(self, device) -> None:
        """Preset, on-camera noise removal, and the host filter chain.

        Measured choices — see GEMINI_TEMPORAL in config for the numbers. The
        filters are fresh instances per open: the temporal filter carries
        history, and history from before a reopen is not this scene.
        """
        import pyorbbecsdk as ob

        if config.GEMINI_PRESET:
            try:
                device.load_preset(config.GEMINI_PRESET)
            except Exception as exc:  # noqa: BLE001 - a bad name must not cost the camera
                log.warning("Gemini: could not load preset %r (%s) — keeping the "
                            "device's current one, %r", config.GEMINI_PRESET, exc,
                            device.get_current_preset_name())
        device.set_bool_property(ob.OBPropertyID.OB_PROP_DEPTH_SOFT_FILTER_BOOL,
                                 config.GEMINI_NOISE_REMOVAL)

        wanted = []
        spatial = {"fast": "SpatialFastFilter", "advanced": "SpatialAdvancedFilter"}
        if config.GEMINI_SPATIAL in spatial:
            wanted.append(spatial[config.GEMINI_SPATIAL])
        elif config.GEMINI_SPATIAL not in ("off", "0", "none", ""):
            log.warning("Gemini: ORIO_GEMINI_SPATIAL=%r is not fast/advanced/off — "
                        "no spatial filter", config.GEMINI_SPATIAL)
        if config.GEMINI_TEMPORAL:
            wanted.append("TemporalFilter")  # after spatial: smooth space, then time

        available = {
            f.get_name(): f
            for f in device.get_sensor(ob.OBSensorType.DEPTH_SENSOR).get_recommended_filters()
        }
        self._filters = []
        for name in wanted:
            f = available.get(name)
            if f is None:
                log.warning("Gemini: this device offers no %s — skipped", name)
                continue
            f.enable(True)
            self._filters.append(f)
        self.depth_setup = (
            f"preset {device.get_current_preset_name()!r}, "
            f"camera noise removal {'on' if config.GEMINI_NOISE_REMOVAL else 'off'}, "
            f"host filters: {', '.join(f.get_name() for f in self._filters) or 'none'}"
        )

    def _filter(self, depth):
        for f in self._filters:
            depth = f.process(depth)
            if depth is None:
                return None
        return depth.as_depth_frame() if self._filters else depth

    # ── grabbing ─────────────────────────────────────────────────────────────

    def _depth_metres(self, frame):
        import numpy as np

        h, w = frame.get_height(), frame.get_width()
        raw = np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(h, w)
        # get_depth_scale() is millimetres per unit; the Gemini's default is 1.
        depth = raw.astype(np.float32) * (frame.get_depth_scale() / 1000.0)
        # 0 is the camera's "no depth here". Outside the range gate is unknown
        # too, not far or near: below min-Z the stereo cannot triangulate at all.
        depth[(raw == 0) | (depth < config.STEREO_MIN_RANGE_M)
              | (depth > config.STEREO_MAX_RANGE_M)] = np.nan
        if (h, w) != (self._height, self._width):
            import cv2

            # Nearest, never linear: interpolating across an edge invents depth
            # halfway between the obstacle and the wall behind it.
            depth = cv2.resize(depth, (self._width, self._height),
                               interpolation=cv2.INTER_NEAREST)
        return depth

    def _grab_loop(self) -> None:
        seq = 0
        failures = 0
        while not self._stop.is_set():
            try:
                frames = self._pipeline.wait_for_frames(1000)
                if frames is None:
                    failures += 1
                    if failures >= 3:
                        raise RuntimeError("Gemini delivered no frames for 3 s")
                    continue
                if self._align is not None:
                    frames = self._align.process(frames)
                    if frames is None:
                        continue
                    frames = frames.as_frame_set()
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if color is None or depth is None:
                    continue
                # After alignment, so the filters see the grid the map is built on.
                depth = self._filter(depth)
                if depth is None:
                    continue
                out = Frames(
                    color=_to_bgr(color, color.get_width(), color.get_height()),
                    depth=self._depth_metres(depth),
                    timestamp=time.monotonic(),
                    seq=seq + 1,
                )
            except Exception as exc:  # noqa: BLE001 - published to readers, not swallowed
                with self._cond:
                    self._error = exc
                    self._cond.notify_all()
                if self._stop.wait(0.5):  # a dead camera should not spin the CPU
                    break
                continue
            seq += 1
            failures = 0
            with self._cond:
                self._latest, self._error = out, None
                self._cond.notify_all()

    def read(self, after: int = 0, timeout: float = 2.0) -> Frames:
        """The newest frameset with `seq > after`, waiting up to `timeout`.

        Pass the `seq` of the last frameset you used to be sure of a fresh one;
        pass 0 to take whatever is newest. Raises rather than returning stale
        frames, so a wedged camera shows up as an error in the caller — which for
        the avoidance thread means a stale reading, and a robot that stops.
        """
        self.start()
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._latest is None or self._latest.seq <= after:
                if self._error is not None:
                    raise RuntimeError(f"Gemini: {self._error}") from self._error
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"Gemini: no new frame within {timeout:.1f} s")
                self._cond.wait(remaining)
            return self._latest

    def close(self) -> None:
        with self._start_lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                self._thread = None
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                finally:
                    self._pipeline = None
            self._latest, self._error, self._align = None, None, None
            self._filters = []


_shared: GeminiCamera | None = None
_shared_lock = threading.Lock()


def shared() -> GeminiCamera:
    """The process-wide camera. Every reader on the robot goes through this."""
    global _shared
    with _shared_lock:
        if _shared is None:
            _shared = GeminiCamera()
        return _shared
