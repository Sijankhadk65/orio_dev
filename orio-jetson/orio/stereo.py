"""Stereo depth and obstacle detection from the IMX219-83's two sensors.

Perception only. This module answers "what is in front of me and how far",
and deliberately stops there — it emits no motion, and knows nothing about the
STM32. Turning an `ObstacleMap` into drive commands is a separate concern (see
"Where this stops" below).

The pipeline, per frame pair:

    both sensors (Argus) -> rectify -> SGBM disparity -> depth -> sector map

`StereoCamera` owns the two capture pipelines, `DepthEstimator` turns a pair
into metric depth, and `obstacles()` reduces a depth map to the handful of
numbers a planner actually wants: the nearest obstacle in each of a few
vertical sectors across the field of view.

Two hardware quirks are baked in, both measured on the bench rather than
assumed, and both silent failures if you get them wrong:

* **The eye/sensor mapping is a cabling fact, not a constant.** Argus sensor 0
  is currently the *left* camera, but the ribbons were crossed until
  2026-09-05, when sensor 1 was. Get it backwards and every disparity comes out
  negative, which reads as blank depth rather than as an error. Re-measure the
  sign of the x-shift after touching the CSI cables.
* **The sensors are not row-aligned.** There is a consistent vertical offset
  — 9 px at 240 px tall on the binned capture mode. SGBM assumes rectified,
  row-aligned input, so this has to be removed or matching collapses.
* **The capture mode is load-bearing.** 1640x1232 is the binned full-array
  mode; the obvious 1920x1080 is a 1.71x crop with no binning, which measured
  4.7x the sensor noise and cost half the valid depth. It also silently
  changes the focal length and field of view, and with them every distance and
  sector angle. See STEREO_CAPTURE_WIDTH in config.
* **The eyes auto-expose independently** and drift far apart (56% brightness
  mismatch measured). SGBM is not illumination-invariant, so the right eye is
  releveled onto the left before matching — see `_match_exposure`.

## Where this stops

`ObstacleMap` is the handoff point. It carries distances and clearances, not
velocities: no part of this module decides how fast to go, when to stop, or
which way to turn. That policy — and the `CMD_DRIVE` framing that would carry
it to the STM32 — is intentionally not implemented here.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from . import config

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Sector:
    """One angular slice of the view ahead.

    `distance_m` is the *near* edge of what occupies this sector — a robust low
    percentile of the sector's depths, not the mean, because the closest thing
    is what you hit. `None` means unknown, which is not the same as clear.
    """

    index: int
    angle_deg: float  # sector centre, negative = left of straight ahead
    distance_m: float | None
    valid_frac: float

    @property
    def known(self) -> bool:
        return self.distance_m is not None


@dataclass(frozen=True)
class ObstacleMap:
    """Nearest obstacle per sector, plus the summary a planner would ask for.

    Deliberately free of any notion of speed or steering — see the module
    docstring. This is a description of the world, not a decision about it.
    """

    sectors: tuple[Sector, ...]
    timestamp: float
    calibrated: bool

    @property
    def nearest(self) -> Sector | None:
        """The closest known obstacle anywhere in view."""
        known = [s for s in self.sectors if s.known]
        return min(known, key=lambda s: s.distance_m) if known else None

    def clearance_ahead(self, sectors: int = 3) -> float | None:
        """Nearest known obstacle in the middle `sectors` — the path straight on."""
        mid = len(self.sectors) // 2
        half = sectors // 2
        window = self.sectors[max(0, mid - half) : mid + half + 1]
        known = [s.distance_m for s in window if s.known]
        return min(known) if known else None

    def describe(self) -> str:
        """One-line human summary, for logs and the debug overlay."""
        n = self.nearest
        if n is None:
            return "no depth (unknown everywhere)"
        side = "ahead" if abs(n.angle_deg) < 10 else ("left" if n.angle_deg < 0 else "right")
        cal = "" if self.calibrated else " (uncalibrated, approximate)"
        return f"nearest {n.distance_m:.2f} m {side}{cal}"


def _argus_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    """GStreamer pipeline for one sensor, via the Jetson ISP.

    Mirrors `vision._argus_pipeline`; kept separate because stereo wants a
    single-buffer queue (latest frame, never a backlog) so the two eyes stay
    as close in time as possible — and because stereo needs the *binned*
    capture mode, where the mono path does not care.

    The capture resolution requested here selects the sensor mode, and that
    choice dominates depth quality: see STEREO_CAPTURE_WIDTH in config for the
    measurements. `width`/`height` are the size the ISP scales down to.
    """
    src = f"nvarguscamerasrc sensor-id={sensor_id}"
    # Pinning both sensors to identical exposure and gain is the fix-at-source
    # for their independent auto-exposure drifting apart. Off by default: a
    # fixed exposure suits a fixed room, and auto is the safer general default.
    if config.STEREO_EXPOSURE_NS and config.STEREO_GAIN:
        e, g = config.STEREO_EXPOSURE_NS, config.STEREO_GAIN
        src += (
            f' exposuretimerange="{e} {e}" gainrange="{g} {g}"'
            ' ispdigitalgainrange="1 1" aelock=true awblock=true'
        )
    return (
        f"{src} ! "
        f"video/x-raw(memory:NVMM),width={config.STEREO_CAPTURE_WIDTH},"
        f"height={config.STEREO_CAPTURE_HEIGHT},framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw,width={width},height={height},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1"
    )


class StereoCamera:
    """Both sensors as one synchronised source. `read()` returns (left, right).

    The two pipelines are independent — Argus gives no hardware sync — so
    frames are grabbed back to back and retrieved afterwards, which measures
    ~11 ms of skew. At walking pace that is a fraction of a centimetre of
    parallax error, but it is the reason this is not a metrology instrument.
    """

    def __init__(
        self,
        left_sensor_id: int = config.STEREO_LEFT_SENSOR_ID,
        right_sensor_id: int = config.STEREO_RIGHT_SENSOR_ID,
        width: int = config.STEREO_WIDTH,
        height: int = config.STEREO_HEIGHT,
        fps: int = config.STEREO_FPS,
    ) -> None:
        self._ids = (left_sensor_id, right_sensor_id)
        self._width = width
        self._height = height
        self._fps = fps
        self._caps: list = []
        self._lock = threading.Lock()

    def _ensure_open(self) -> None:
        if self._caps:
            return
        import cv2  # heavy import, kept lazy

        caps = []
        for sid in self._ids:
            cap = cv2.VideoCapture(
                _argus_pipeline(sid, self._width, self._height, self._fps),
                cv2.CAP_GSTREAMER,
            )
            if not cap.isOpened():
                cap.release()
                for c in caps:
                    c.release()
                raise RuntimeError(
                    f"could not open Argus sensor {sid}. Both sensors must be free — "
                    "the vision tool's ObjectDetector holds one if it is running, and "
                    "OpenCV must have GStreamer support (see tools/link_system_cv2.py)."
                )
            caps.append(cap)
        self._caps = caps

    def read(self):
        """Grab one (left, right) pair. Raises if either eye fails."""
        with self._lock:
            self._ensure_open()
            # Grab both before retrieving either: retrieve() does the expensive
            # colour conversion, and doing it between the grabs would widen the
            # time skew between the eyes for no reason.
            for cap in self._caps:
                cap.grab()
            frames = []
            for sid, cap in zip(self._ids, self._caps):
                ok, frame = cap.retrieve()
                if not ok:
                    # Contention shows up here, not at open(): a second client
                    # opens the sensor happily and only starves at read time.
                    # The vision tool holds one eye whenever it is running.
                    raise RuntimeError(
                        f"could not read a frame from Argus sensor {sid} — the pipeline "
                        "opened but delivered nothing, which usually means another "
                        "process already holds this sensor. Stereo needs BOTH eyes, so "
                        "stop the vision tool (ORIO_VISION_DEBUG=0, or the main app) "
                        "before running stereo."
                    )
                frames.append(frame)
            return frames[0], frames[1]

    def close(self) -> None:
        with self._lock:
            for cap in self._caps:
                cap.release()
            self._caps = []


def _match_exposure(src, ref):
    """Rescale `src`'s intensities onto `ref`'s mean and standard deviation.

    The two sensors auto-expose independently and end up looking materially
    different (see STEREO_PHOTOMETRIC_MATCH in config). SGBM compares raw
    intensities, so that difference alone destroys matches. A global affine fit
    is deliberately the crudest thing that works: it cannot invent or destroy
    texture, only relevel it, so it cannot manufacture a false match the way a
    local operator like CLAHE can.
    """
    import numpy as np

    a, b = ref.astype(np.float32), src.astype(np.float32)
    scale = a.std() / max(float(b.std()), 1e-6)
    return np.clip((b - b.mean()) * scale + a.mean(), 0, 255).astype(np.uint8)


class DepthEstimator:
    """Rectifies a stereo pair and turns it into metric depth.

    Uses `tools/calibrate_stereo.py` output when present. Without it, falls
    back to published optics plus the measured vertical offset: the *shape* of
    the depth map is right and obstacles rank correctly, but the absolute
    metres carry real error. `calibrated` says which mode is in effect, and
    everything downstream propagates that flag rather than hiding it.
    """

    def __init__(
        self,
        width: int = config.STEREO_WIDTH,
        height: int = config.STEREO_HEIGHT,
        calibration: Path = config.STEREO_CALIBRATION,
        baseline_m: float = config.STEREO_BASELINE_M,
    ) -> None:
        self._width = width
        self._height = height
        self._baseline_m = baseline_m
        self._maps = None  # rectification maps, when calibrated
        self._rect_valid = None  # where those maps actually have source data
        self._matcher = None
        self.calibrated = False
        self._focal_px = config.STEREO_FALLBACK_FOCAL_PX_AT_640 * (width / 640.0)
        self._vshift = int(round(config.STEREO_FALLBACK_VSHIFT_FRAC * height))
        # Uncalibrated the frame is uncropped, so the datasheet FOV is right;
        # `_load_calibration` replaces this with the rectified value.
        self.hfov_deg = config.STEREO_HFOV_DEG
        self._load_calibration(calibration)

    def _load_calibration(self, path: Path) -> None:
        if not path.exists():
            log.warning(
                "no stereo calibration at %s — depth is approximate. Run "
                "tools/calibrate_stereo.py for trustworthy metres.",
                path,
            )
            return
        import cv2
        import numpy as np

        data = np.load(path)
        size = (self._width, self._height)
        # Calibration is stored at the resolution it was captured at; intrinsics
        # scale linearly with image size, so a rebuild at another matching
        # resolution stays valid rather than silently skewing depth.
        scale = self._width / float(data["image_width"])
        K1, K2 = data["K1"] * scale, data["K2"] * scale
        K1[2, 2] = K2[2, 2] = 1.0
        # T is in MILLIMETRES: calibrate_stereo.py builds its board model in mm
        # so the solved baseline is directly comparable to the published 60 mm.
        # It has to become metres here, because T sets the units of P2[0,3] (and
        # of Q) and this module speaks metres throughout. Feeding mm straight in
        # does not produce a visibly silly number — it makes every depth 1000x
        # too large, which the STEREO_MAX_RANGE_M gate then turns into NaN, so
        # the symptom is "unknown everywhere" rather than "distances look wrong".
        #
        # alpha=-1 is OpenCV's default scaling, not alpha=0. alpha=0 crops to the
        # rectangle both rectified images fully cover, and on this rig that is a
        # 1.66x zoom: it throws away a third of the horizontal view (73.8 -> 48.7
        # deg) and inflates the focal length, which in turn demands more
        # disparities and so widens the structurally-blind left band. Measured
        # A/B on live frames, alpha=-1 wins on every axis that matters —
        #
        #   alpha  hfov    numDisp  near     valid px  blind band  sectors seen
        #    0.0   48.7 deg   96    0.23 m    34.4%      30%          4 of 7
        #   -1.0   74.1 deg   64    0.21 m    38.7%      20%          6 of 7
        #
        # The cost is invalid pixels near the frame edges, which SGBM cannot
        # match anyway and which therefore already read as unknown depth.
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, data["D1"], K2, data["D2"], size, data["R"], data["T"] / 1000.0,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1,
        )
        self._maps = (
            cv2.initUndistortRectifyMap(K1, data["D1"], R1, P1, size, cv2.CV_16SC2),
            cv2.initUndistortRectifyMap(K2, data["D2"], R2, P2, size, cv2.CV_16SC2),
        )
        # Rectification leaves part of the output frame with no source pixel, and
        # remap fills that with BLACK — in both eyes. SGBM then matches black
        # against black and returns a confident disparity for a region that
        # contains no information at all, which is the one thing this module must
        # never do: unknown must not come back as a measurement. Measured at 0.8%
        # of the frame reporting a fabricated ~1.65 m, a third of it inside the
        # obstacle band. Remapping a solid image finds the region exactly.
        ones = np.full((self._height, self._width), 255, np.uint8)
        self._rect_valid = (cv2.remap(ones, *self._maps[0], cv2.INTER_LINEAR) > 0) & (
            cv2.remap(ones, *self._maps[1], cv2.INTER_LINEAR) > 0
        )
        self._focal_px = float(P1[0, 0])
        self._baseline_m = abs(float(P2[0, 3] / P2[0, 0]))
        # Sector angles must come from the rectified focal, not the datasheet
        # FOV: rectification rescales the frame (see the alpha note above), so
        # the two agree only by accident. Reading 73 deg off a 48.7 deg frame
        # puts every obstacle further out to the side than it really is.
        self.hfov_deg = 2.0 * math.degrees(math.atan(self._width / 2.0 / self._focal_px))
        self.calibrated = True
        log.info(
            "stereo calibration loaded: focal %.1f px, baseline %.1f mm, hfov %.1f deg",
            self._focal_px, self._baseline_m * 1000, self.hfov_deg,
        )

    def _ensure_matcher(self):
        if self._matcher is None:
            import cv2

            # numDisparities must be a multiple of 16, and it sets the NEAR
            # limit: closer than focal*baseline/numDisparities cannot be
            # measured. Derive it from the optics actually in effect rather than
            # from the frame width, because rectification rescales the focal
            # length and would otherwise push the near limit out past
            # STEREO_MIN_RANGE_M — going blind at exactly the distances an
            # obstacle map is for. It is a real trade: the leftmost
            # numDisparities columns can never match, so this also sets how much
            # of the left edge is structurally unknown. Capped at half the width,
            # past which the blind band costs more than the near range is worth.
            needed = self._focal_px * self._baseline_m / config.STEREO_MIN_RANGE_M
            num_disp = 16 * min(
                max(1, math.ceil(needed / 16)), max(1, self._width // 32)
            )
            block = 7
            self._matcher = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=num_disp,
                blockSize=block,
                P1=8 * block * block,
                P2=32 * block * block,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=2,
                disp12MaxDiff=1,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
            )
        return self._matcher

    def rectify(self, left, right):
        """(left, right) BGR pair -> the same pair as SGBM will see it.

        Split out of `depth` because anything drawing depth alongside an image
        has to draw it over *this* image, not the raw frame. Rectification moves
        content by a long way — the principal point lands 85 px off centre on
        this rig — so overlaying a depth map on the unrectified capture lines up
        nothing with anything.
        """
        import cv2
        import numpy as np

        if self._maps is not None:
            left = cv2.remap(left, *self._maps[0], cv2.INTER_LINEAR)
            right = cv2.remap(right, *self._maps[1], cv2.INTER_LINEAR)
        elif self._vshift:
            # Uncalibrated: strip the measured rigid row offset so SGBM's
            # row-alignment assumption at least approximately holds.
            right = np.roll(right, -self._vshift, axis=0)
        return left, right

    def depth(self, left, right, rectified: bool = False):
        """(left, right) BGR pair -> float32 depth in metres, NaN where unknown.

        Pass `rectified=True` if the pair has already been through `rectify`,
        so a caller that needs the rectified frames too does not pay for the
        remap twice.
        """
        import cv2
        import numpy as np

        if not rectified:
            left, right = self.rectify(left, right)

        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        if config.STEREO_PHOTOMETRIC_MATCH:
            gr = _match_exposure(gr, gl)
        disp = self._ensure_matcher().compute(gl, gr).astype(np.float32) / 16.0

        with np.errstate(divide="ignore", invalid="ignore"):
            depth = (self._focal_px * self._baseline_m) / disp
        # Anything outside the usable range is unknown, not far away. A 60 mm
        # baseline simply cannot speak to it.
        depth[disp <= 0] = np.nan
        depth[(depth < config.STEREO_MIN_RANGE_M) | (depth > config.STEREO_MAX_RANGE_M)] = np.nan
        if self._rect_valid is not None:
            depth[~self._rect_valid] = np.nan
        elif self._vshift:
            # The uncalibrated path rolls the right eye, which wraps the top rows
            # around to the bottom. Same problem, same answer.
            depth[-self._vshift :, :] = np.nan
        return depth


def obstacles(
    depth,
    calibrated: bool,
    sectors: int = config.STEREO_SECTORS,
    hfov_deg: float = config.STEREO_HFOV_DEG,
) -> ObstacleMap:
    """Reduce a depth map to the nearest obstacle in each vertical sector.

    Only the band of the frame that could hold something the robot would
    collide with is considered (see STEREO_BAND_*): the ceiling and the floor
    underfoot are always "close" and would otherwise dominate every reading.

    Distance per sector is the 10th percentile of valid depths, not the
    minimum — a single mismatched pixel at 0.3 m would otherwise stop the robot
    dead — and a sector with too few valid pixels reports `None` (unknown), never
    a distance. Unknown must not read as clear.
    """
    import numpy as np

    h, w = depth.shape
    band = depth[int(config.STEREO_BAND_TOP * h) : int(config.STEREO_BAND_BOTTOM * h), :]
    edges = np.linspace(0, w, sectors + 1).astype(int)

    out: list[Sector] = []
    for i in range(sectors):
        col = band[:, edges[i] : edges[i + 1]]
        valid = col[np.isfinite(col)]
        frac = valid.size / col.size if col.size else 0.0
        distance = (
            float(np.percentile(valid, 10))
            if frac >= config.STEREO_MIN_VALID_FRAC and valid.size
            else None
        )
        centre = (edges[i] + edges[i + 1]) / 2.0
        angle = (centre / w - 0.5) * hfov_deg
        out.append(Sector(index=i, angle_deg=angle, distance_m=distance, valid_frac=frac))

    return ObstacleMap(sectors=tuple(out), timestamp=time.time(), calibrated=calibrated)


class ObstacleDetector:
    """Convenience wrapper: camera + estimator, `sense()` per reading.

    Mirrors `vision.ObjectDetector`'s shape — lazy open, a lock so concurrent
    callers never share the capture handles, explicit `close()`.
    """

    def __init__(self) -> None:
        self._camera = StereoCamera()
        self._estimator = DepthEstimator()
        self._lock = threading.Lock()

    def sense(self) -> ObstacleMap:
        """One stereo reading, reduced to the sector map."""
        with self._lock:
            left, right = self._camera.read()
            depth = self._estimator.depth(left, right)
        return obstacles(
            depth,
            calibrated=self._estimator.calibrated,
            hfov_deg=self._estimator.hfov_deg,
        )

    def sense_with_frames(self):
        """`(ObstacleMap, left, depth)` — for the debug view's overlay.

        The frame returned is the RECTIFIED left eye, pixel-aligned with the
        depth map, so the two can be drawn side by side and compared.
        """
        with self._lock:
            left, right = self._camera.read()
            left, right = self._estimator.rectify(left, right)
            depth = self._estimator.depth(left, right, rectified=True)
        omap = obstacles(
            depth,
            calibrated=self._estimator.calibrated,
            hfov_deg=self._estimator.hfov_deg,
        )
        return omap, left, depth

    def close(self) -> None:
        self._camera.close()
