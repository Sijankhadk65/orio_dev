"""The sector map, and the one piece of geometry every sensor reaches it through.

`Sector` and `ObstacleMap` are the contract `orio/avoid.py` consumes. They used
to live in `stereo.py`, which was fine while the cameras were the only thing
that could see; they moved here when the ToF fan arrived, so that a second
sensor could emit the same shape without importing the stereo pipeline (and its
OpenCV, and its Argus) to do it. `stereo.py` re-exports both, so every existing
`from .stereo import ObstacleMap` still resolves.

Two functions on top of that:

* `SectorGeometry` / `points_to_sectors()` — (range, azimuth, elevation) ->
  (height above the floor, azimuth, ground distance), reduced to sectors. Both
  the depth map and the ToF grids go through this, deliberately: the stereo
  ground-plane classification and the ToF projection are the same arithmetic,
  and written twice they drift and only one gets the bug fix.
* `fuse()` — combine maps over the same sector grid, per sector, conservatively.

## Height, not image row

The older reduction kept a fixed band of image rows and called whatever was in
it an obstacle. That works only because the floor is reliably *outside* the
band, which makes it a crop rather than a classification: everything below the
band edge is discarded, including the 5 cm box the wheels will actually hit.

Classifying by height above the floor eliminates the floor by geometry instead.
The whole frame becomes usable, and a low obstacle becomes a legitimate reading.
The price is that the answer now depends on two numbers that must be *measured*
— how high the sensor sits and how far down it looks — and a degree or two of
pitch error tilts the fitted plane enough that distant floor reads as an
obstacle. That failure is silent and distinctive: the robot refuses to leave
`steer` for `cruise` in an empty room.

## "Nothing there" is not "cannot see"

`distance_m = None` means unknown, and unknown is never clear. Classifying by
height introduces a third state the row band never had: a sector where the
ground is plainly visible, out to some distance, with nothing standing on it.
Reporting that as `None` would be a lie in the dangerous direction — the policy
treats an unknown corridor as blocked, so an empty room would stop the robot
dead. `clear_m` carries it instead: *the ground under this sector was verified
free out to here*. It is not an obstacle distance and must never be minimised
against one; it is the answer of last resort, used only where no sensor saw
anything at all (see `stereo.obstacles`).

It is also more honest than it first looks. An obstacle too dark or too smooth
to match also *occludes the floor behind it*, so the observed ground stops at
the obstacle and `clear_m` lands at roughly its distance rather than past it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import config


@dataclass(frozen=True)
class Sector:
    """One angular slice of the view ahead.

    `distance_m` is the *near* edge of what occupies this sector — a robust low
    percentile of the sector's ranges, not the mean, because the closest thing
    is what you hit. `None` means unknown, which is not the same as clear.

    `clear_m` is the separate, weaker claim described in the module docstring:
    ground seen and verified free out to here, with nothing standing on it.
    `source` names the sensor the numbers came from, which is what settles the
    inevitable argument about which one is lying.
    """

    index: int
    angle_deg: float  # sector centre, negative = left of straight ahead
    distance_m: float | None
    valid_frac: float
    source: str = ""
    clear_m: float | None = None

    @property
    def known(self) -> bool:
        return self.distance_m is not None


@dataclass(frozen=True)
class ObstacleMap:
    """Nearest obstacle per sector, plus the summary a planner would ask for.

    Deliberately free of any notion of speed or steering — see `stereo.py`'s
    module docstring. This is a description of the world, not a decision
    about it.
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

    @property
    def sources(self) -> tuple[str, ...]:
        """Which sensor won each sector, left to right. `""` where none did."""
        return tuple(s.source for s in self.sectors)

    def describe(self) -> str:
        """One-line human summary, for logs and the debug overlay."""
        n = self.nearest
        if n is None:
            return "no depth (unknown everywhere)"
        side = "ahead" if abs(n.angle_deg) < 10 else ("left" if n.angle_deg < 0 else "right")
        cal = "" if self.calibrated else " (uncalibrated, approximate)"
        src = f" [{n.source}]" if n.source else ""
        return f"nearest {n.distance_m:.2f} m {side}{src}{cal}"


def sector_angles(sectors: int, hfov_deg: float) -> tuple[float, ...]:
    """Centre angle of each sector across `hfov_deg`, left to right.

    Identical to the value the old column-based reduction computed, so the grid
    does not move underneath `avoid.py` when a sensor starts reporting into it.
    """
    return tuple(((i + 0.5) / sectors - 0.5) * hfov_deg for i in range(sectors))


class SectorGeometry:
    """Ray directions -> sector index and the two multipliers a range needs.

    Split out from the reduction because it is *frame-invariant*. A depth map's
    per-pixel azimuth and elevation are fixed by the intrinsics, and a ToF
    array's are fixed by its zone layout; neither changes between readings,
    while the ranges change every reading. Doing the trigonometry once at open
    and keeping two float arrays turns a per-frame cost of several milliseconds
    of `sin`/`cos`/`atan2` over ~10^5 pixels into two multiplies.

    Conventions, all of which are easy to get backwards and silent when you do:

    * Sensor frame: +x right, +y up, +z forward along the optical axis.
    * `az_deg` is positive to the RIGHT of the optical axis, `el_deg` positive
      UP from it — the same sign convention `Sector.angle_deg` uses.
    * `pitch_deg` is positive looking DOWN, `yaw_deg` positive turned RIGHT.
      Roll is assumed zero; a rolled sensor needs a third rotation here.
    * `height_m` is the sensor's height above the floor it is measuring against.

    The sensor's lateral and forward offset from the robot's centre of rotation
    is deliberately NOT modelled. A sector is ~10 deg wide and the mounts sit
    within a few centimetres of the centre line, so the offset is well under
    half a sector at every range the map is used at. It stops being true if
    something is ever mounted out on a boom.
    """

    def __init__(
        self,
        az_deg,
        el_deg,
        *,
        height_m: float,
        pitch_deg: float,
        yaw_deg: float = 0.0,
        sectors: int = config.STEREO_SECTORS,
        hfov_deg: float = config.STEREO_HFOV_DEG,
    ) -> None:
        import numpy as np

        self.height_m = height_m
        self.sectors = sectors
        self.hfov_deg = hfov_deg
        self.angles = sector_angles(sectors, hfov_deg)

        az = np.radians(np.asarray(az_deg, dtype=np.float64).ravel())
        el = np.radians(np.asarray(el_deg, dtype=np.float64).ravel())
        if az.shape != el.shape:
            raise ValueError(f"az/el shape mismatch: {az.shape} vs {el.shape}")

        # Unit ray in the sensor frame.
        ce = np.cos(el)
        dx, dy, dz = ce * np.sin(az), np.sin(el), ce * np.cos(az)

        # Pitch down about the sensor's own right axis, then yaw right about the
        # vertical. Order matters: the mount pitches the sensor and the mount is
        # what is yawed, so pitch is applied in the sensor frame first.
        p = math.radians(pitch_deg)
        y1 = dy * math.cos(p) - dz * math.sin(p)
        z1 = dy * math.sin(p) + dz * math.cos(p)
        w = math.radians(yaw_deg)
        x2 = dx * math.cos(w) + z1 * math.sin(w)
        z2 = -dx * math.sin(w) + z1 * math.cos(w)

        # Per unit of range: how much height it gains, and how much ground it
        # covers. A ray angled down has a negative `up`, which is what makes the
        # floor fall out as "height ~ 0" rather than needing to be cropped away.
        self.up = y1.astype(np.float32)
        self.ground = np.hypot(x2, z2).astype(np.float32)

        # Bin by the ray's azimuth in the ROBOT frame, which is not the sensor's
        # azimuth plus the yaw once there is any pitch on the mount. Rays outside
        # the grid get -1 and are dropped: a splayed ToF array sees well past the
        # 7 sectors stereo defines, and widening the grid is a separate change
        # that would move `avoid.py`'s corridor arithmetic with it.
        robot_az = np.degrees(np.arctan2(x2, z2))
        width = hfov_deg / sectors
        idx = np.floor((robot_az + hfov_deg / 2.0) / width).astype(np.int32)
        idx[(robot_az < -hfov_deg / 2.0) | (robot_az >= hfov_deg / 2.0)] = -1
        self.index = idx
        self.robot_az = robot_az.astype(np.float32)

        # Denominator of `valid_frac`: how many rays LOOK at each sector, which
        # is fixed, versus how many returned a range, which is not.
        self.totals = np.bincount(idx[idx >= 0], minlength=sectors)[:sectors]

    @property
    def size(self) -> int:
        return int(self.index.size)

    def reduce(
        self,
        ranges,
        *,
        floor_tol_m: float = config.STEREO_FLOOR_TOL_M,
        ceiling_m: float = config.ROBOT_HEIGHT_M,
        min_valid_frac: float = config.STEREO_MIN_VALID_FRAC,
        max_range_m: float | None = None,
        trust_m: float | None = None,
        percentile: float = 10.0,
        clear_percentile: float = 90.0,
        source: str = "",
    ) -> tuple[Sector, ...]:
        """Ranges along those rays -> one `Sector` each. NaN range = no return.

        A point counts as an obstacle when it stands between `floor_tol_m` and
        `ceiling_m` above the floor: below that it is the ground, above it is a
        doorway lintel the robot drives under. The sector's distance is a low
        percentile of the obstacle points' GROUND distance — not the slant range,
        which would report a doorframe at head height as nearer than it is.

        Too few returns means `None`, never a distance: the `min_valid_frac`
        gate is what keeps a blank wall or a dark corner from reading as clear
        road. See the module docstring for why `clear_m` is not that gate's
        escape hatch but a separate, weaker claim.
        """
        import numpy as np

        r = np.asarray(ranges, dtype=np.float32).ravel()
        if r.size != self.index.size:
            raise ValueError(
                f"{r.size} ranges for {self.index.size} rays — the geometry was "
                "built for a different grid, which means someone changed the "
                "resolution without rebuilding it"
            )

        finite = np.isfinite(r)
        if max_range_m is not None:
            finite &= r <= max_range_m
        keep = finite & (self.index >= 0)

        idx = self.index[keep]
        rk = r[keep]
        height = self.height_m + rk * self.up[keep]
        ground = rk * self.ground[keep]

        is_obstacle = (height > floor_tol_m) & (height < ceiling_m)
        is_floor = height <= floor_tol_m
        if trust_m is not None:
            # Range-dependent trust. A height computed from a range that far out
            # is worth less than the sensor's own noise — one pixel of disparity
            # error is 16.5 cm at 1.5 m on this rig — so past `trust_m` a point
            # stops being allowed to CLAIM an obstacle. It still counts as floor,
            # and still counts toward `valid_frac`: the far floor is perfectly
            # good evidence that the ground is clear, and only the obstacle
            # decision is the one the noise ruins. Something further out than
            # this is the row band's business (see stereo.DepthEstimator).
            is_obstacle &= rk <= trust_m

        seen = np.bincount(idx, minlength=self.sectors)[: self.sectors]

        out: list[Sector] = []
        for i in range(self.sectors):
            total = int(self.totals[i])
            frac = (int(seen[i]) / total) if total else 0.0
            distance = clear = None
            if frac >= min_valid_frac:
                here = idx == i
                obstacles = ground[here & is_obstacle]
                if obstacles.size:
                    distance = float(np.percentile(obstacles, percentile))
                floor = ground[here & is_floor]
                if floor.size:
                    clear = float(np.percentile(floor, clear_percentile))
            out.append(
                Sector(
                    index=i,
                    angle_deg=self.angles[i],
                    distance_m=distance,
                    valid_frac=frac,
                    source=source,
                    clear_m=clear,
                )
            )
        return tuple(out)

    def classify(self, ranges, *, floor_tol_m: float = config.STEREO_FLOOR_TOL_M,
                 ceiling_m: float = config.ROBOT_HEIGHT_M):
        """Per-ray class for the debug views: 0 unknown, 1 floor, 2 obstacle,
        3 above the robot. Tuning the two heights blind is not worth attempting,
        so both `stereo_debug.py` and `tof_debug.py` colour by this.
        """
        import numpy as np

        r = np.asarray(ranges, dtype=np.float32).ravel()
        height = self.height_m + r * self.up
        out = np.zeros(r.shape, np.uint8)
        finite = np.isfinite(r)
        out[finite & (height <= floor_tol_m)] = 1
        out[finite & (height > floor_tol_m) & (height < ceiling_m)] = 2
        out[finite & (height >= ceiling_m)] = 3
        return out


def points_to_sectors(ranges, az_deg, el_deg, *, height_m: float, pitch_deg: float,
                      yaw_deg: float = 0.0, sectors: int = config.STEREO_SECTORS,
                      hfov_deg: float = config.STEREO_HFOV_DEG, source: str = "",
                      **reduce_kw) -> tuple[Sector, ...]:
    """One-shot `SectorGeometry` + `reduce`, for callers with no grid to cache.

    Prefer holding a `SectorGeometry` when the ray directions are fixed — for a
    depth map or a ToF array they are, and building one per frame throws away
    the entire point of the split.
    """
    geom = SectorGeometry(az_deg, el_deg, height_m=height_m, pitch_deg=pitch_deg,
                          yaw_deg=yaw_deg, sectors=sectors, hfov_deg=hfov_deg)
    return geom.reduce(ranges, source=source, **reduce_kw)


def fuse(*maps: "ObstacleMap | None") -> ObstacleMap:
    """Combine maps over the same sector grid. Per sector, the pessimist wins.

    `min()` of the known distances; known if EITHER source is known; unknown
    only if both are. That is the conservative choice and conservative is what a
    collision guard is for — with the cost, worth saying out loud, that one bad
    zone on one sensor stops the robot. The defence against that is the robust
    low percentile inside `reduce()`, not a vote here: a vote would need three
    sensors and would let two blind ones outvote the one that can see.

    The winning source is recorded per sector. `clear_m` takes the MAXIMUM
    instead, and for the opposite reason: it is a claim about ground somebody
    verified, so the sensor that saw furthest is the one with the information.
    Timestamps take the oldest, so a frozen sensor ages the fused map rather
    than hiding behind a fresh one.

    `None` maps are skipped, which is how a ToF that never opened degrades to
    stereo-only instead of halting the robot.
    """
    live = [m for m in maps if m is not None]
    if not live:
        raise ValueError("fuse() needs at least one map")
    if len(live) == 1:
        return live[0]

    n = len(live[0].sectors)
    if any(len(m.sectors) != n for m in live):
        raise ValueError(
            "fusing maps with different sector counts: "
            f"{[len(m.sectors) for m in live]}. Every source must project onto "
            "the one grid — see SectorGeometry."
        )

    out: list[Sector] = []
    for i in range(n):
        here = [m.sectors[i] for m in live]
        known = [s for s in here if s.known]
        best = min(known, key=lambda s: s.distance_m) if known else None
        clears = [s.clear_m for s in here if s.clear_m is not None]
        out.append(
            Sector(
                index=i,
                angle_deg=here[0].angle_deg,
                distance_m=None if best is None else best.distance_m,
                valid_frac=max(s.valid_frac for s in here),
                source=best.source if best is not None else "",
                clear_m=max(clears) if clears else None,
            )
        )

    return ObstacleMap(
        sectors=tuple(out),
        timestamp=min(m.timestamp for m in live),
        calibrated=all(m.calibrated for m in live),
    )


def fill_clear(omap: ObstacleMap, source_suffix: str = "-clear") -> ObstacleMap:
    """Answer of last resort: where NOTHING is known, fall back to `clear_m`.

    Applied after every other source has had its say, and only to sectors still
    reporting `None`. The policy treats an unknown corridor as blocked — rightly,
    since unknown is not clear — and classifying by height creates sectors that
    are unknown for the opposite reason to the usual one: not "I cannot see"
    but "I can see the ground and there is nothing on it". Left as `None`, an
    empty room would stop the robot dead, which is the regression the plan's
    Phase 5 watches for.

    A sector with no `clear_m` either is left alone. Genuinely blind stays
    blind, and the source string says which of the two happened.
    """
    if not any(s.distance_m is None and s.clear_m is not None for s in omap.sectors):
        return omap
    out = tuple(
        s
        if s.distance_m is not None or s.clear_m is None
        else Sector(
            index=s.index,
            angle_deg=s.angle_deg,
            distance_m=s.clear_m,
            valid_frac=s.valid_frac,
            source=f"{s.source}{source_suffix}",
            clear_m=s.clear_m,
        )
        for s in omap.sectors
    )
    return ObstacleMap(sectors=out, timestamp=omap.timestamp, calibrated=omap.calibrated)
