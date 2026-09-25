"""Depth and obstacle detection from the Gemini 336L.

Perception only. This module answers "what is in front of me and how far",
and deliberately stops there — it emits no motion, and knows nothing about the
STM32. Turning an `ObstacleMap` into drive commands is a separate concern (see
"Where this stops" below).

The pipeline, per frame:

    Gemini (depth computed on the camera, aligned to colour) -> sector map

The camera does the stereo — see `orio/gemini.py`. What is left here is the
reduction: `DepthReducer` turns a metric depth map into the handful of numbers
a planner actually wants, the nearest obstacle in each of a few vertical
sectors across the field of view.

## Two reductions, not one

`obstacles()` is the original and still the default: keep a fixed BAND of image
rows and take a robust low percentile of the depths in each sector's columns.
The band is a crop, and it earns its keep by excluding the floor — which is
always "close" and would otherwise have the robot believe it is permanently
blocked — but it excludes the floor by throwing away the bottom of the frame,
and with it everything short enough to sit down there. A box, a shoe, a cable
spool: tall enough to stop the wheels, too low to be looked at.

`DepthReducer.obstacles()` is the answer to that (`STEREO_GROUND_PLANE`, off
until the rig is measured — see config). It reprojects each pixel into the
robot frame and keeps it if it stands between the floor tolerance and the
robot's own height, so the floor eliminates itself by GEOMETRY rather than by
cropping and the whole lower frame becomes usable. It is authoritative only as
far as `STEREO_HEIGHT_TRUST_M`; past that the band answers, and the two fuse by
`min()`. The shared arithmetic — and the ToF fan's half of it — lives in
`orio/sectors.py`.

## Where this stops

`ObstacleMap` is the handoff point. It carries distances and clearances, not
velocities: no part of this module decides how fast to go, when to stop, or
which way to turn. That policy lives in `avoid.py`.
"""

from __future__ import annotations

import threading
import time

from . import config
from .gemini import Intrinsics, shared
from .sectors import ObstacleMap, Sector, SectorGeometry, fill_clear, fuse

# `Sector` and `ObstacleMap` are DEFINED in orio/sectors.py and re-exported
# here, unchanged. They moved when the ToF fan arrived: a second sensor has to
# emit the same shape, and importing this module to get the dataclass would drag
# the camera SDK in behind it. Every `from .stereo import ObstacleMap` still
# works, which is the point.
__all__ = [
    "ObstacleDetector",
    "ObstacleMap",
    "Sector",
    "DepthReducer",
    "obstacles",
]


class DepthReducer:
    """Metric depth map -> `ObstacleMap`, given the intrinsics it was taken with.

    The Gemini is factory-calibrated and reports metres natively, so every map
    this produces is `calibrated=True`: there is no approximate mode any more.
    """

    calibrated = True

    def __init__(self, intrinsics: Intrinsics) -> None:
        self._k = intrinsics
        self.hfov_deg = intrinsics.hfov_deg
        self._ground: tuple | None = None  # cached ray geometry, see _ground_geometry

    def _ground_geometry(self, shape):
        """Cached ray directions for the ground-plane reduction, per frame size.

        Every pixel's azimuth and elevation are fixed by the intrinsics: only
        the ranges change between frames. Building the `SectorGeometry` once and
        keeping it is what makes classifying by height affordable at 30 Hz —
        rebuilt per frame it is several milliseconds of trigonometry over 10^5
        pixels.

        Returns `(geometry, range_multiplier, stride)`. The multiplier turns
        DEPTH (the z component, which is what the camera reports) into RANGE
        along the ray, which is what the geometry expects. Confusing the two
        understates everything off-axis — by more than a third at the corner of
        a 94 deg frame.
        """
        import numpy as np

        h, w = shape
        stride = max(1, config.STEREO_GROUND_STRIDE)
        if self._ground is not None and self._ground[3] == (h, w, stride):
            return self._ground[:3]

        k = self._k
        us = np.arange(0, w, stride, dtype=np.float64)
        vs = np.arange(0, h, stride, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)
        # Pinhole ray, z normalised to 1. Image v grows DOWNWARD and the sensor
        # frame's y is UP, hence the sign.
        dx = (uu - k.cx) / k.fx
        dy = -(vv - k.cy) / k.fy
        norm = np.sqrt(dx * dx + dy * dy + 1.0)
        geom = SectorGeometry(
            np.degrees(np.arctan2(dx, 1.0)),
            np.degrees(np.arcsin(dy / norm)),
            height_m=config.STEREO_CAM_HEIGHT_M,
            pitch_deg=config.STEREO_CAM_PITCH_DEG,
            sectors=config.STEREO_SECTORS,
            hfov_deg=self.hfov_deg,
        )
        self._ground = (geom, norm.astype(np.float32), stride, (h, w, stride))
        return self._ground[:3]

    def obstacles(self, depth) -> ObstacleMap:
        """Depth map -> sector map, by whichever reductions are switched on.

        With `STEREO_GROUND_PLANE` off this is exactly the row band and nothing
        else.

        With it on, TWO reductions run and fuse by `min()`:

        * the ground plane, over the whole frame, out to `STEREO_HEIGHT_TRUST_M`
          — which is where the low obstacles live;
        * the row band, unchanged, which keeps answering past that range.

        They are not alternatives and the fusion is not a fallback: each is
        authoritative over a different volume, and `min()` is the conservative
        combination for the same reason it is between camera and ToF. Sectors
        that are still unknown afterwards take `clear_m` as an answer of last
        resort (see `sectors.fill_clear`).
        """
        band = obstacles(depth, calibrated=True, hfov_deg=self.hfov_deg,
                         source="stereo-band")
        if not config.STEREO_GROUND_PLANE:
            return band

        geom, mult, stride = self._ground_geometry(depth.shape)
        ranges = depth[::stride, ::stride] * mult
        ground = ObstacleMap(
            sectors=geom.reduce(
                ranges,
                floor_tol_m=config.STEREO_FLOOR_TOL_M,
                ceiling_m=config.ROBOT_HEIGHT_M,
                min_valid_frac=config.STEREO_MIN_VALID_FRAC,
                trust_m=config.STEREO_HEIGHT_TRUST_M,
                source="stereo-ground",
            ),
            timestamp=band.timestamp,
            calibrated=True,
        )
        return fill_clear(fuse(band, ground))

    def classify(self, depth):
        """Per-pixel class for the debug view, at the reduction's own stride.

        `(classes, stride)` — 0 unknown, 1 floor, 2 obstacle, 3 above the robot.
        """
        geom, mult, stride = self._ground_geometry(depth.shape)
        classes = geom.classify(
            depth[::stride, ::stride] * mult,
            floor_tol_m=config.STEREO_FLOOR_TOL_M,
            ceiling_m=config.ROBOT_HEIGHT_M,
        )
        return classes.reshape(depth[::stride, ::stride].shape), stride


def obstacles(
    depth,
    calibrated: bool,
    sectors: int = config.STEREO_SECTORS,
    hfov_deg: float = config.STEREO_HFOV_DEG,
    source: str = "stereo-band",
) -> ObstacleMap:
    """Reduce a depth map to the nearest obstacle in each vertical sector.

    The ROW BAND reduction: only the band of the frame that could hold
    something the robot would collide with is considered (see STEREO_BAND_*):
    the ceiling and the floor underfoot are always "close" and would otherwise
    dominate every reading.

    Its blind spot is structural and is why `DepthReducer.obstacles()` exists:
    an obstacle below the band's bottom edge is not merely far, it is discarded
    before the sector map is built. This stays a free function because it needs
    nothing but a depth map, which is what makes it testable and what makes the
    regression path one env var wide.

    Distance per sector is the 10th percentile of valid depths, not the
    minimum — a single bad pixel at 0.3 m would otherwise stop the robot dead —
    and a sector with too few valid pixels reports `None` (unknown), never a
    distance. Unknown must not read as clear.
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
        out.append(Sector(index=i, angle_deg=angle, distance_m=distance,
                          valid_frac=frac, source=source))

    return ObstacleMap(sectors=tuple(out), timestamp=time.time(), calibrated=calibrated)


class ObstacleDetector:
    """Convenience wrapper: shared camera + reducer, `sense()` per reading.

    Reads from `gemini.shared()`, so it never holds the camera exclusively:
    the vision tool can take pictures from the same device while this runs.
    Each detector tracks the last frameset it used, so consecutive `sense()`
    calls never reduce the same frame twice.
    """

    def __init__(self, camera=None) -> None:
        self._camera = camera if camera is not None else shared()
        self._reducer: DepthReducer | None = None
        self._seq = 0
        self._lock = threading.Lock()

    @property
    def reducer(self) -> DepthReducer:
        """The reducer, built once the camera has reported its intrinsics."""
        with self._lock:
            return self._ensure_reducer()

    def _ensure_reducer(self) -> DepthReducer:
        if self._reducer is None:
            self._camera.start()
            self._reducer = DepthReducer(self._camera.intrinsics)
        return self._reducer

    def _next(self):
        reducer = self._ensure_reducer()
        frames = self._camera.read(after=self._seq)
        self._seq = frames.seq
        return reducer, frames

    def sense(self) -> ObstacleMap:
        """One reading, reduced to the sector map."""
        with self._lock:
            reducer, frames = self._next()
        return reducer.obstacles(frames.depth)

    def sense_with_frames(self):
        """`(ObstacleMap, color, depth)` — the map plus what it was made from.

        `color` is the BGR colour frame, pixel-aligned with `depth` by the
        camera's D2C, so a detection box on one covers the same pixels on the
        other. Shared with the camera's other readers: treat both as read-only.
        """
        with self._lock:
            reducer, frames = self._next()
        return reducer.obstacles(frames.depth), frames.color, frames.depth

    def close(self) -> None:
        """Release the camera.

        The camera is shared, so this stops it for every reader; the next
        `read()` from anyone reopens it. Only the session's owner (`Body`, a
        bench tool's `main`) should call this, on the way out.
        """
        self._camera.close()
        with self._lock:
            self._reducer, self._seq = None, 0
