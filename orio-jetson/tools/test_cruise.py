#!/usr/bin/env python3
"""Check `body.Cruise` and the approach it drives, against fake boards.

    uv run python tools/test_cruise.py

No hardware, no cameras, no serial ports: a fake drivetrain records every duty
pair it is sent, a fake sensor answers with whatever depth the test wants, and
`Body` is handed both. Runs in a few seconds, so run it after touching
`body.py`, `avoid.py` or `seek.py`.

## Why this exists

A cruise is the one part of the drive stack that runs on its own thread while
somebody else does something slow, and the questions that shape it — does the
latch survive a look, does the deadman fire, does `stop()` deadlock against the
loop's own lock — are all about timing between two threads. None of them can be
answered by reading, and none of them need a robot to answer. What DOES need a
robot is whether the gait actually looks smooth and whether the policy steers
sensibly; nothing here speaks to either.

The second half simulates an approach end to end: a target three metres ahead
and off to one side, closing while the wheels are actually turning. That makes
the stutter measurable — the world accumulates how long the wheels spent at
zero, which is precisely what this change was meant to reduce.

## The fake world integrates on reads, not just on writes

A cruise sends a duty ONCE and the board holds it, which is the whole point of
the change and also the easiest thing to get wrong in a simulation. If the
world only advanced when a frame went out, a robot driving steadily would never
get anywhere and the test would "fail" for the best possible reason. So the
world advances on every read as well.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.avoid import Reading, avoider_from_config
from orio.bump import Bump, BumpMemory, StallDetector
from orio.drivetrain import Drivetrain, Status, WheelTelemetry
from orio.sectors import ObstacleMap, Sector, SectorGeometry, fill_clear, fuse
from orio.body import DIRECTIONS, NO_WHEELS, NOT_RENEWED, Body
from orio.seek import Seeker

# Three sectors, the same shape avoid.Sensor publishes.
SECTOR_ANGLES = (-31.8, 0.0, 31.8)
CLEAR = tuple((a, 5.0) for a in SECTOR_ANGLES)

# How fast the simulated robot closes on the target while driving forward, and
# how long the simulated detector takes. The detector figure is the one that
# matters: the old approach stopped the wheels for the whole of every inference,
# so its stutter scaled with this while a cruise's does not.
SPEED_M_PER_S = 0.6
DETECT_S = float(os.environ.get("DETECT_S", "0.12"))

FAILURES: list[str] = []


def check(name: str, passed: bool, detail: str = "") -> None:
    print(f"  {'ok  ' if passed else 'FAIL'}  {name}" + ("" if passed else f"  — {detail}"))
    if not passed:
        FAILURES.append(name)


class FakeLink:
    """A drivetrain that records instead of driving."""

    def __init__(self) -> None:
        self.sent: list[tuple[float, int, int]] = []
        self.lock = threading.Lock()
        # What the FSESCs are reporting, or None for a board that never
        # answers. Held as the wheel dict rather than a whole `Status` so
        # `last_status` can stamp it fresh on every poll, the way a live
        # board does: a fixture stamped once ages past BUMP_STATUS_STALE_S
        # mid-run and stops confirming for a reason that has nothing to do
        # with what is being tested.
        self.wheels = None
        self.status_requests = 0

    def set_drive(self, left: int, right: int) -> None:
        with self.lock:
            self.sent.append((time.monotonic(), left, right))

    def take_rejection(self):
        return None

    # `_drive_tick` polls wheel telemetry for stall detection. A link that
    # never answers is the normal case on this bench: `last_status` stays
    # None, the detector resets every tick and records nothing — which is the
    # property every test below silently depends on.
    def request_status(self) -> None:
        self.status_requests += 1

    # A PROPERTY, because that is what Drivetrain.last_status is. It was a
    # method here first, which made every test below agree with a call site
    # that could not work against the real board.
    @property
    def last_status(self):
        if self.wheels is None:
            return None
        return Status(estopped=False, wheels=self.wheels, timestamp=time.monotonic())

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass

    @property
    def standing(self) -> tuple[int, int] | None:
        """The duty pair the board is holding right now."""
        with self.lock:
            return self.sent[-1][1:] if self.sent else None

    def stopped(self) -> bool:
        return self.standing == (0, 0)

    def rolled_through(self, t0: float, t1: float) -> bool:
        """Was a non-zero duty standing for the whole of [t0, t1]?"""
        with self.lock:
            before = [s for s in self.sent if s[0] <= t0]
            during = [s for s in self.sent if t0 < s[0] <= t1]
        if not before or before[-1][1:] == (0, 0):
            return False
        return all(pair[1:] != (0, 0) for pair in during)


class FakeSensor:
    """A stereo pair that always sees the same thing."""

    def __init__(self, sectors=CLEAR, age_s: float = 0.0, bumps=None) -> None:
        self.sectors = sectors
        self.age_s = age_s
        self.bumps = bumps

    @property
    def reading(self) -> Reading:
        return Reading(self.sectors, 5.0, "fake", time.monotonic() - self.age_s)

    def close(self) -> None:
        pass


def fake_body(link=None, sensor=None) -> Body:
    """A `Body` wired to fakes. `_neck` stays None, which makes
    `_ensure_driving_pose()` a no-op — there is no head to aim."""
    body = Body()
    body._drive = link if link is not None else FakeLink()
    body._sensor = sensor if sensor is not None else FakeSensor()
    body._neck = None
    return body


def no_cruise_threads() -> bool:
    return not any(t.name == "cruise" for t in threading.enumerate())


# ── the cruise itself ────────────────────────────────────────────────────────


def test_latch_survives_a_look() -> None:
    """The point of the whole change: looking must not stop the wheels."""
    print("\nthe latch survives a slow look")
    body = fake_body()
    link = body._drive
    with body.cruise() as cruise:
        cruise.go("forward")
        time.sleep(0.15)
        start = time.monotonic()
        time.sleep(0.5)  # stand in for a detector pass, touching nothing
        check("no stop was sent while 'looking'", link.rolled_through(start, time.monotonic()),
              f"last sent {link.standing}")
        window = cruise.drain()
        check("the policy kept deciding throughout", "cruise" in window.states,
              f"states={window.states}")
        check("the window does not read blocked", not window.blocked)
    check("the wheels stop on leaving the block", link.stopped(), f"{link.standing}")
    check("no cruise thread is left behind", no_cruise_threads())


def test_hop_is_unchanged() -> None:
    """`hop()` is what the LLM's move tools use and this change must not alter
    it: still bounded by its span, still stops itself."""
    print("\nhop() is unchanged")
    body = fake_body()
    link = body._drive
    started = time.monotonic()
    hop = body.hop("forward", DIRECTIONS["forward"], 0.4)
    span = time.monotonic() - started
    check("it respects its span", 0.4 <= span < 0.6, f"{span:.2f}s")
    check("it stops the wheels", link.stopped(), f"{link.standing}")
    check("it reports the policy's state", hop.states == ("cruise",), f"{hop.states}")
    check("a clear road is not blocked", not hop.blocked)


def test_deadman() -> None:
    """A latch nobody renews must expire, or a wedged caller leaves the robot
    driving."""
    print("\nthe deadman expires an unrenewed latch")
    body = fake_body()
    link = body._drive
    cruise = body.cruise()
    cruise.go("forward")
    time.sleep(0.15)
    check("rolling before the deadman", not link.stopped())
    time.sleep(config.CRUISE_DEADMAN_S + 0.25)
    check("the wheels are stopped by it", link.stopped(), f"{link.standing}")
    window = cruise.drain()
    check("and the stop is reported", window.halted == NOT_RENEWED, f"halted={window.halted!r}")
    check("an expired window reads blocked", window.blocked)
    cruise.go("forward")
    time.sleep(0.15)
    check("a renewed latch drives again", not link.stopped(), f"{link.standing}")
    cruise.close()


def test_hold_is_a_burst() -> None:
    """The shape seek's pivots use: latch, wait, hold."""
    print("\nhold() stops the wheels and keeps the loop")
    body = fake_body()
    link = body._drive
    with body.cruise() as cruise:
        cruise.go("left")
        time.sleep(config.SEEK_TURN_BURST_S)
        cruise.hold()
        time.sleep(0.12)
        check("pivot signs reached the board",
              any(left < 0 < right for _, left, right in link.sent), f"{link.sent[:4]}")
        check("held means stopped", link.stopped(), f"{link.standing}")
        sent = len(link.sent)
        time.sleep(0.2)
        check("a held cruise does not re-send stops", len(link.sent) == sent,
              f"{len(link.sent) - sent} extra frames")
        cruise.go("forward")
        time.sleep(0.12)
        check("go() after hold() drives again", not link.stopped())


def test_stop_during_a_cruise() -> None:
    """`stop()` takes the lock the cruise thread takes every tick. It ends the
    cruise first, deliberately, and this is the test that says so."""
    print("\nstop() during a cruise")
    body = fake_body()
    link = body._drive
    cruise = body.cruise()
    cruise.go("forward")
    time.sleep(0.15)
    returned = threading.Event()
    threading.Thread(target=lambda: (body.stop(), returned.set()), daemon=True).start()
    check("stop() returns (no deadlock)", returned.wait(3.0))
    time.sleep(0.2)
    check("the wheels stay stopped", link.stopped(), f"{link.standing}")
    check("the cruise thread is gone", no_cruise_threads())


def test_exception_stops_the_wheels() -> None:
    print("\nan exception inside the block")
    body = fake_body()
    link = body._drive
    try:
        with body.cruise() as cruise:
            cruise.go("forward")
            time.sleep(0.15)
            raise RuntimeError("detector exploded")
    except RuntimeError:
        pass
    check("the wheels stop on the way out", link.stopped(), f"{link.standing}")
    check("no cruise thread is left behind", no_cruise_threads())


def test_blind_and_wheelless() -> None:
    print("\nthe cases where it must not drive at all")
    body = fake_body()
    body._drive = None
    with body.cruise() as cruise:
        cruise.go("forward")
        time.sleep(0.15)
        check("no drivetrain reports NO_WHEELS", cruise.drain().halted == NO_WHEELS)

    body = fake_body(sensor=FakeSensor(age_s=config.AVOID_STALE_S + 1.0))
    link = body._drive
    with body.cruise() as cruise:
        cruise.go("forward")
        time.sleep(0.15)
        window = cruise.drain()
        check("a stale reading halts the cruise", window.halted is not None,
              f"halted={window.halted!r}")
        check("and the wheels are not turning", link.standing in (None, (0, 0)),
              f"{link.standing}")


def test_close_is_idempotent() -> None:
    print("\nclose() twice")
    body = fake_body()
    link = body._drive
    cruise = body.cruise()
    cruise.go("forward")
    time.sleep(0.1)
    cruise.close()
    cruise.close()
    check("the wheels are stopped and stay stopped", link.stopped(), f"{link.standing}")


# ── the sector map: fusion and the geometry both sensors share ───────────────
#
# `fuse()` and `SectorGeometry` are pure and sector-shaped, so they belong here
# rather than on the robot: the cases that matter are two sources disagreeing,
# one knowing and one not, neither knowing, and one gone stale — none of which
# needs a camera, an I2C bus, or a box on the floor to provoke.


def sector_map(distances, source="x", clear=None, hfov=73.1, timestamp=None):
    """An ObstacleMap over `len(distances)` sectors. None means unknown."""
    n = len(distances)
    clear = clear if clear is not None else [None] * n
    return ObstacleMap(
        sectors=tuple(
            Sector(index=i, angle_deg=((i + 0.5) / n - 0.5) * hfov, distance_m=d,
                   valid_frac=1.0, source=source, clear_m=c)
            for i, (d, c) in enumerate(zip(distances, clear))
        ),
        timestamp=time.time() if timestamp is None else timestamp,
        calibrated=True,
    )


def test_fuse() -> None:
    print("\nfusing two sources into one sector map")
    stereo = sector_map([2.0, 3.0, None, None], source="stereo-band")
    tof = sector_map([0.4, None, 0.8, None], source="tof-left")
    fused = fuse(stereo, tof)
    got = [s.distance_m for s in fused.sectors]

    check("the pessimist wins a disagreement", got[0] == 0.4, f"{got[0]}")
    check("and the winner is named", fused.sectors[0].source == "tof-left",
          fused.sectors[0].source)
    check("a source that knows beats one that does not (stereo)", got[1] == 3.0, f"{got[1]}")
    check("a source that knows beats one that does not (tof)", got[2] == 0.8, f"{got[2]}")
    check("unknown to both stays unknown", got[3] is None, f"{got[3]}")
    check("no source claims an unknown sector", fused.sectors[3].source == "",
          fused.sectors[3].source)

    # The oldest reading sets the age, so a frozen sensor cannot hide behind a
    # fresh one — it ages the fused map instead.
    old = sector_map([1.0, 1.0, 1.0, 1.0], timestamp=time.time() - 10.0)
    check("the fused map is as old as its oldest source",
          fuse(stereo, old).timestamp == old.timestamp)

    # This is how the ToF fan degrades rather than halting: it simply is not
    # there, and fusion is the identity.
    check("a missing source fuses to the one that is left",
          fuse(stereo, None) is stereo)
    check("no source at all is an error, not a silent empty map",
          raises(ValueError, lambda: fuse(None, None)))
    check("grids that do not match are refused",
          raises(ValueError, lambda: fuse(stereo, sector_map([1.0] * 7))))


def test_clear_is_not_an_obstacle() -> None:
    print("\nverified-clear ground is the answer of last resort, never a reading")
    # Sector 0: an obstacle AND clear ground seen beyond it. The obstacle wins —
    # if `clear_m` could ever be minimised against a real reading, ground seen
    # past an obstacle would argue the obstacle away.
    omap = sector_map([0.5, None, None], clear=[3.0, 2.5, None])
    filled = fill_clear(omap)
    got = [s.distance_m for s in filled.sectors]
    check("a real obstacle is not replaced by clear ground", got[0] == 0.5, f"{got[0]}")
    check("an unknown sector with clear ground gets it", got[1] == 2.5, f"{got[1]}")
    check("genuinely blind stays blind", got[2] is None, f"{got[2]}")
    check("and the fallback says so in the source",
          filled.sectors[1].source.endswith("-clear"), filled.sectors[1].source)

    # Fusion runs BEFORE the fill, so a ToF obstacle still beats stereo's
    # "I can see the ground and it is empty".
    tof = sector_map([None, 0.3, None], source="tof-right")
    check("a ToF obstacle beats clear ground from the cameras",
          fill_clear(fuse(omap, tof)).sectors[1].distance_m == 0.3)


def test_ground_plane_geometry() -> None:
    """The claim Fix A rests on: a 5 cm box at 0.6 m is a reading, and the floor
    it stands on is not."""
    print("\nclassifying by height: a 5 cm box on a floor, seen by a pitched camera")
    import numpy as np

    h, pitch = 0.35, 20.0          # camera height and how far down it looks
    box_at, box_high = 0.60, 0.05  # a low obstacle, the kind the band discards

    az = np.repeat(np.arange(-36.0, 36.0, 1.0), 200)
    el = np.tile(np.arange(-35.0, 5.0, 0.2), 72)
    geom = SectorGeometry(az, el, height_m=h, pitch_deg=pitch, sectors=7, hfov_deg=73.1)

    def trace(with_box: bool):
        """Range to the first thing each ray hits: the floor, or the box."""
        up, ground = geom.up.astype(np.float64), geom.ground.astype(np.float64)
        r = np.full(up.shape, np.inf)
        down = up < 0
        r[down] = -h / up[down]  # the floor, at height 0
        if with_box:
            r_box = box_at / np.maximum(ground, 1e-9)
            hits = (h + r_box * up >= 0) & (h + r_box * up <= box_high)
            r[hits] = np.minimum(r[hits], r_box[hits])
        r[~np.isfinite(r)] = np.nan
        return r

    empty = geom.reduce(trace(with_box=False), floor_tol_m=0.03, ceiling_m=0.60,
                        source="floor-only")
    mid = empty[3]
    check("bare floor produces no obstacle at all", mid.distance_m is None,
          f"{mid.distance_m}")
    check("but the floor it saw is reported as verified clear",
          mid.clear_m is not None and mid.clear_m > 1.0, f"{mid.clear_m}")

    boxed = geom.reduce(trace(with_box=True), floor_tol_m=0.03, ceiling_m=0.60,
                        source="ground")
    mid = boxed[3]
    check("a 5 cm box at 0.60 m is seen", mid.distance_m is not None)
    check("and it is seen AT 0.60 m, by ground distance not slant range",
          mid.distance_m is not None and abs(mid.distance_m - box_at) < 0.03,
          f"{mid.distance_m}")
    check("the floor under it still does not count as an obstacle",
          all(s.distance_m is None or abs(s.distance_m - box_at) < 0.05 for s in boxed),
          f"{[None if s.distance_m is None else round(s.distance_m, 2) for s in boxed]}")

    # Above the robot is not an obstacle either: drop the ceiling below the box
    # and the same reading has to disappear.
    under = geom.reduce(trace(with_box=True), floor_tol_m=0.03, ceiling_m=0.02,
                        source="ground")
    check("anything taller than the robot is driven under, not around",
          under[3].distance_m is None, f"{under[3].distance_m}")


def test_tof_pose_lands_in_the_right_sector() -> None:
    """A yawed sensor reports into the robot's grid, not its own."""
    print("\na splayed ToF array projects into the shared sector grid")
    import numpy as np

    from orio.tof import zone_angles

    az, el = zone_angles(8, 8, 45.0)
    check("an 8x8 grid is 64 zones, 5.625 deg apart",
          az.size == 64 and abs(float(np.unique(az)[1] - np.unique(az)[0]) - 5.625) < 1e-6)

    for yaw, expect_side in ((-22.5, "left"), (22.5, "right")):
        geom = SectorGeometry(az, el, height_m=0.045, pitch_deg=0.0, yaw_deg=yaw,
                              sectors=7, hfov_deg=73.1)
        # A wall square across the fan at 1 m: every zone sees it at its own
        # slant range, and the map must report 1 m of GROUND distance.
        ranges = 1.0 / np.maximum(geom.ground, 1e-9)
        known = [s for s in geom.reduce(ranges, floor_tol_m=0.03, ceiling_m=0.6,
                                        source="tof") if s.known]
        check(f"the {expect_side} array reports something", bool(known))
        check(f"the {expect_side} array reports it at 1 m",
              all(abs(s.distance_m - 1.0) < 0.05 for s in known),
              f"{[round(s.distance_m, 2) for s in known]}")
        centre = sum(s.angle_deg for s in known) / len(known)
        check(f"and its sectors sit to the {expect_side}",
              (centre < 0) if yaw < 0 else (centre > 0), f"centre {centre:+.1f} deg")


def test_tof_axial_distance() -> None:
    """The ULD reports distance along the OPTICAL AXIS, not along each zone's ray.

    Measured against a flat wall at 0.86 m: read as line-of-sight range the
    plane residual is a 44 mm bullseye at 25 mm RMS; read as axial it is 0.4 mm
    and 3.3 mm, the sensor's own noise. `ToFArray` therefore builds pinhole rays
    and scales by `range_scale`, exactly as stereo.py has always done for the
    depth map.

    The test that would NOT catch this is the obvious one — synthesise ranges
    from the geometry and check they come back — because it is self-consistent
    under either convention. This one states the physics independently: for a
    LEVEL sensor facing a wall at axial distance D, the point in zone (az, el)
    sits at height D.tan(el) above the axis and D/cos(az) of ground away. Both
    are exact, and both are wrong by the sin-for-tan error under the old code —
    6% at the corner of the grid, and it feeds the floor classification.
    """
    print("\nthe ToF grid is axial distance through a pinhole, not slant range")
    import numpy as np

    from orio.tof import ToFArray, ToFSensor

    mount_h, dist = 0.30, 0.30
    array = ToFArray(ToFSensor(bus=-1, name="tof-test"), height_m=mount_h,
                     pitch_deg=0.0, yaw_deg=0.0, sectors=7, hfov_deg=73.1)
    rays = dist * array.range_scale          # axial -> line-of-sight
    heights = mount_h + rays * array.geometry.up
    grounds = rays * array.geometry.ground

    want_h = mount_h + dist * np.tan(np.radians(array.el_deg))
    want_g = dist / np.cos(np.radians(array.az_deg))
    check("a flat wall lands at the right height in every zone",
          bool(np.allclose(heights, want_h, atol=1e-4)),
          f"worst {np.abs(heights - want_h).max() * 1000:.2f} mm")
    check("and at the right ground distance in every zone",
          bool(np.allclose(grounds, want_g, atol=1e-4)),
          f"worst {np.abs(grounds - want_g).max() * 1000:.2f} mm")

    # The old convention got the centre right and the corners wrong, so a test
    # that only looked straight ahead would have passed throughout.
    naive = mount_h + dist * np.sin(np.radians(array.el_deg))
    corner = int(np.argmax(np.abs(array.el_deg) + np.abs(array.az_deg)))
    # Stated as a FRACTION of the range, because that is what it is: tan - sin
    # at the corner elevation, so 2% of the distance to whatever is there. At
    # 0.3 m that is 6 mm and easy to wave away; at 2 m it is 4 cm.
    error_frac = abs(naive[corner] - want_h[corner]) / dist
    check("reading it as slant range misplaces the corner by 2% of the range",
          error_frac > 0.02,
          f"{error_frac * 100:.1f}% ({abs(naive[corner] - want_h[corner]) * 1000:.1f} mm "
          f"at {dist:.2f} m)")


def test_tof_stale_drops_out() -> None:
    """A quiet ToF must leave the fusion, not age the map and halt the robot."""
    print("\na stale ToF map drops out of the fusion")
    from orio.tof import ToFDetector

    det = ToFDetector(arrays=[])
    det._map = sector_map([0.3, 0.3, 0.3], source="tof-left")
    det._published_at = time.monotonic()
    check("a fresh map is offered", det.fresh_map(max_age_s=0.5) is not None)

    # Liveness is "is the fan still publishing", not "how old is the oldest
    # reading inside it". `fuse` stamps with its oldest contributor, so a map
    # built from a 0.49 s reading is itself stamped 0.49 s old, and gating on
    # that would blink the fan out on arithmetic alone while both sensors were
    # ranging happily. `_republish` has already dropped stale contributors.
    det._map = sector_map([0.3, 0.3, 0.3], source="tof-left",
                          timestamp=time.time() - 0.49)
    check("an old contributing stamp does not itself withhold the map",
          det.fresh_map(max_age_s=0.5) is not None)

    det._published_at = time.monotonic() - 5.0
    check("a fan that stopped publishing is withheld",
          det.fresh_map(max_age_s=0.5) is None)
    check("and stereo alone is then the whole map",
          fuse(sector_map([2.0, 2.0, 2.0]), det.fresh_map(max_age_s=0.5)).sectors[0]
          .distance_m == 2.0)


def test_tof_one_quiet_sensor() -> None:
    """A starved sensor must not take its healthy neighbour out with it.

    The bus-1 sensor is GIL-starved by its faster neighbour and was measured
    going 900+ ms between frames (2026-09-17). Fusing on the oldest
    contributor, that dragged the WHOLE fan past TOF_STALE_S and avoidance fell
    back to stereo — blind low and blind inside 0.25 m, which is the entire
    reason the parts are there. Staleness is per sensor for that reason.
    """
    print("\none quiet ToF sensor drops out; the live one keeps the fan up")
    from orio.tof import ToFDetector

    class _Named:
        def __init__(self, name):
            self.name = name

    det = ToFDetector(arrays=[_Named("tof-left"), _Named("tof-right")])
    det._latest["tof-left"] = sector_map([0.3, 0.3, 0.3], source="tof-left")
    det._latest["tof-right"] = sector_map([0.1, 0.1, 0.1], source="tof-right",
                                          timestamp=time.time() - 5.0)
    det._republish()
    omap = det.fresh_map(max_age_s=0.5)
    check("the fan still publishes", omap is not None)
    check("the live sensor's reading is what it publishes",
          omap is not None and omap.sectors[0].distance_m == 0.3,
          None if omap is None else f"{omap.sectors[0].distance_m}")
    check("the quiet sensor's nearer reading is NOT carried forward",
          omap is not None and all(s.distance_m != 0.1 for s in omap.sectors))

    # Both quiet: nothing new is published, the last map ages, and the fan
    # leaves the fusion. Republishing a stale fusion under a fresh timestamp is
    # the one unforgivable version of this.
    det._latest["tof-left"] = sector_map([0.3, 0.3, 0.3], source="tof-left",
                                         timestamp=time.time() - 5.0)
    det._published_at = time.monotonic() - 5.0
    det._republish()
    check("both quiet means the fan withholds entirely",
          det.fresh_map(max_age_s=0.5) is None)


def raises(exc_type, fn) -> bool:
    try:
        fn()
    except exc_type:
        return True
    except Exception:
        return False
    return False


# ── the approach, end to end ─────────────────────────────────────────────────


@dataclass
class FakeDetection:
    label: str
    confidence: float
    bbox: tuple


class World:
    """A target that moves when the wheels do, and remembers the pauses.

    Doubles as both fake boards, so the depth the policy steers on and the
    depth the detector's box lands in come from one place — the same coupling
    `Sensor.snapshot()` exists to guarantee on the real robot.
    """

    def __init__(self, distance_m: float = 3.0, sector: int = 2) -> None:
        self.lock = threading.Lock()
        self.distance = distance_m
        self.sector = sector  # 0 left, 1 centre, 2 right
        self.standing = (0, 0)
        self.at = time.monotonic()
        self.pauses: list[float] = []

    def _advance(self) -> None:
        """Move the world on by however long the standing command has stood."""
        now = time.monotonic()
        elapsed, self.at = now - self.at, now
        if self.standing[0] > 0 and self.standing[1] > 0:
            self.distance = max(0.0, self.distance - SPEED_M_PER_S * elapsed)
        elif self.standing != (0, 0):
            self.sector = 1  # any pivot centres the target, eventually
        elif elapsed > 0.001:
            self.pauses.append(elapsed)

    def set_drive(self, left: int, right: int) -> None:
        with self.lock:
            self._advance()
            self.standing = (left, right)

    def take_rejection(self):
        return None

    def request_status(self) -> None:
        pass

    @property
    def last_status(self):
        return None

    bumps = None

    def stop(self) -> None:
        pass

    def close(self) -> None:
        pass

    @property
    def reading(self) -> Reading:
        with self.lock:
            self._advance()
            distance = self.distance
        return Reading(tuple((a, distance) for a in SECTOR_ANGLES), distance, "sim",
                       time.monotonic())

    def snapshot(self):
        with self.lock:
            self._advance()
        frame = type("Frame", (), {"shape": (480, 640, 3)})()
        return self.reading, frame

    def detect(self, frame) -> list[FakeDetection]:
        time.sleep(DETECT_S)
        with self.lock:
            sector = self.sector
        x0 = (640 // 3) * sector
        return [FakeDetection("person", 0.9, (x0, 100, x0 + 200, 400))]


def test_approach() -> None:
    print(f"\napproaching a person 3.0 m ahead and off to the right "
          f"(detector {DETECT_S:g}s/frame)")
    world = World()
    body = fake_body(link=world, sensor=world)
    started = time.monotonic()
    said = Seeker(body, world.detect).approach("person")
    took = time.monotonic() - started
    stopped = sum(world.pauses)
    longest = max(world.pauses, default=0.0)

    print(f"    said: {said!r}")
    print(f"    took {took:.1f}s to close to {world.distance:.2f} m; wheels stopped "
          f"{stopped:.2f}s over {len(world.pauses)} pauses, longest {longest:.2f}s")
    check("it arrived", said.startswith("went to the person"), said)
    check("it got inside SEEK_ARRIVE_M", world.distance <= config.SEEK_ARRIVE_M + 0.05,
          f"{world.distance:.2f} m")
    # The pivots are still bursts by design, so a pause the length of one look
    # is expected after each. Anything longer means a forward leg stopped to
    # think, which is the stutter this change removed.
    check("no pause outlasts a pivot and the look after it",
          longest <= config.SEEK_TURN_BURST_S + DETECT_S + 0.1, f"longest {longest:.2f}s")
    check("most of the run was spent moving", stopped < took * 0.35,
          f"{stopped:.2f}s stopped of {took:.1f}s")
    check("the wheels are left stopped", world.standing == (0, 0), f"{world.standing}")
    check("no cruise thread is left behind", no_cruise_threads())


# ── a jammed wheel, read as an obstacle ──────────────────────────────────────


def wheel(erpm=0, current_a=20.0, valid=True):
    return WheelTelemetry(erpm=erpm, current_a=current_a, v_in=36.0,
                          fault_code=0, valid=valid)


def status(left=None, right=None, at=None, age_s=0.0, estopped=False):
    """A `Status` stamped `age_s` before `at`. `at` is the simulated clock, not
    the wall clock: the detector compares the status stamp against the `now` it
    is given, so a fixture stamped once with `time.monotonic()` goes stale
    part-way through a simulated run and quietly stops confirming."""
    at = time.monotonic() if at is None else at
    return Status(
        estopped=estopped,
        wheels={"left": left if left is not None else wheel(erpm=3000, current_a=2.0),
                "right": right if right is not None else wheel(erpm=3000, current_a=2.0)},
        timestamp=at - age_s,
    )


def confirm(det, t0, make, duty=(300, 300), ticks=8, step=0.05):
    """Tick the detector forward and return the FIRST bump it confirmed.

    First, not last: the question every test below asks is whether the evidence
    was ever enough, and a detector that confirms and then stops (because the
    fixture aged out, or the wheel broke free) has still answered yes.

    `make(now)` builds the status for that tick, so a fixture can follow the
    simulated clock instead of being stamped once.
    """
    first = None
    for i in range(ticks):
        now = t0 + i * step
        got = det.update(now, duty, make(now))
        if got is not None and first is None:
            first = got
    return first


def jammed(**kw):
    """A both-wheels-jammed status factory, overridable per side."""
    return lambda now: status(
        left=kw.get("left", wheel(erpm=0, current_a=20.0)),
        right=kw.get("right", wheel(erpm=0, current_a=20.0)),
        at=now,
        age_s=kw.get("age_s", 0.0),
        estopped=kw.get("estopped", False),
    )


def left_jammed(**kw):
    return lambda now: status(left=wheel(erpm=0, current_a=20.0, **kw), at=now)


def test_stall_detection() -> None:
    print("\na stalled wheel is confirmed, and nothing else is")
    t0 = time.monotonic()

    det = StallDetector()
    check("a wheel drawing current and not turning is a stall",
          confirm(det, t0, left_jammed()) is not None)
    det = StallDetector()
    check("and it is reported on the side it happened",
          confirm(det, t0, left_jammed()).side == "left")

    # Each of the three conditions alone, and none of them is contact.
    det = StallDetector()
    check("current with the wheel turning is load, not contact",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=3000, current_a=20.0), at=now)) is None)
    # MEASURED 2026-09-18, and it retired the current condition: at 5% duty
    # driving draws 0.04 A and a jam draws 0.06, while eRPM separates 447 from
    # 0. A stall must confirm on the real jam figure, which is BELOW anything a
    # current threshold could have been set to.
    det = StallDetector()
    check("a jam at the MEASURED current (0.06 A) confirms",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=0, current_a=0.06), at=now)) is not None)
    det = StallDetector()
    check("and the measured DRIVING case (447 erpm, 0.04 A) does not",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=447, current_a=0.04), at=now)) is None)
    det = StallDetector()
    check("a free-running wheel's negative current noise cannot veto a stall",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=0, current_a=-0.01), at=now)) is not None)

    # The knob still works when a robot is driven hard enough for current to
    # separate; it is disabled by measurement, not removed.
    det = StallDetector(min_current_a=5.0)
    check("an explicit current threshold still vetoes below it",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=0, current_a=0.06), at=now)) is None)
    det = StallDetector(min_current_a=5.0)
    check("and still confirms above it",
          confirm(det, t0, lambda now: status(
              left=wheel(erpm=0, current_a=20.0), at=now)) is not None)

    # With current no longer vetoing, the e-stop flag is the only thing
    # separating "driving into a wall" from "refusing to drive at all".
    det = StallDetector()
    check("an e-stopped board is not a stall, however jammed it looks",
          confirm(det, t0, jammed(estopped=True)) is None)
    det = StallDetector()
    check("no duty commanded is never a stall",
          confirm(det, t0, left_jammed(), duty=(0, 0)) is None)

    # Trap 3: reversing into something is real, and there is nowhere to put it.
    det = StallDetector()
    check("stalling in REVERSE is not an obstacle ahead",
          confirm(det, t0, jammed(), duty=(-300, -300)) is None)

    # Trap 2, three ways: unknown is not stalled.
    det = StallDetector()
    check("telemetry flagged invalid cannot confirm",
          confirm(det, t0, left_jammed(valid=False)) is None)
    det = StallDetector()
    check("no status at all cannot confirm", confirm(det, t0, lambda now: None) is None)
    det = StallDetector()
    check("stale status cannot confirm",
          confirm(det, t0, jammed(age_s=config.BUMP_STATUS_STALE_S + 1.0)) is None)

    # The start-up transient: duty high, eRPM 0, current at its peak. Confirming
    # instantly would report a bump every time the robot moves off.
    det = StallDetector()
    make = left_jammed()
    check("a single tick of stall is not yet a bump",
          det.update(t0, (300, 300), make(t0)) is None)
    half = t0 + config.BUMP_CONFIRM_S * 0.5
    check("and it is still not one before BUMP_CONFIRM_S",
          det.update(half, (300, 300), make(half)) is None)
    past = t0 + config.BUMP_CONFIRM_S + 0.01
    check("but it is after", det.update(past, (300, 300), make(past)) is not None)

    # Between two hops the loop stops ticking and the robot may be moved or
    # carried. A timer started before that gap must not confirm across it.
    det = StallDetector()
    det.update(t0, (300, 300), make(t0))
    late = t0 + 10.0
    check("a timer does not survive a gap in the ticks",
          det.update(late, (300, 300), make(late)) is None)

    det = StallDetector()
    check("both wheels jammed reports as one two-sided bump",
          confirm(det, t0, jammed()).side == "both")

    # Regression: BUMP_MIN_DUTY was first set above the duty this robot
    # actually drives at, and the detector armed in no test and on no hop.
    # DRIVE_SPEED_MIN_PERCENT is 5, so 50 per-mille is the slowest CRUISE, and
    # `Avoider` scales that again by AVOID_MIN_SCALE and its turn factor while
    # steering. The floor has to sit under all of it.
    slowest_cruise = round(config.DRIVE_SPEED_MIN_PERCENT * 10)
    det = StallDetector()
    check("the detector arms at the slowest speed the robot can be driven at",
          confirm(det, t0, jammed(), duty=(slowest_cruise, slowest_cruise)) is not None,
          f"duty {slowest_cruise} vs floor {config.BUMP_MIN_DUTY}")
    crawling = max(1, round(slowest_cruise * config.AVOID_MIN_SCALE * 0.5))
    det = StallDetector()
    check("and at the slowest the avoider can scale it down to",
          confirm(det, t0, jammed(), duty=(crawling, crawling)) is not None,
          f"duty {crawling} vs floor {config.BUMP_MIN_DUTY}")


def test_fakes_match_the_real_link() -> None:
    """The fakes must have the same SHAPE as `Drivetrain`, not just the same names.

    `last_status` is a PROPERTY on the real link and was a method on FakeLink,
    so every test here agreed with three call sites that could not work against
    the board — `'Status' object is not callable`, found only once the hardware
    was powered up. A fake that disagrees with the real interface does not
    catch bugs, it certifies them.
    """
    print("\nthe fakes have the same shape as the real drivetrain")
    for name in ("last_status", "set_drive", "request_status", "take_rejection", "stop"):
        real = getattr(Drivetrain, name, None)
        check(f"Drivetrain still has {name}", real is not None)
        if real is None:
            continue
        for fake in (FakeLink, World):
            mine = getattr(fake, name, None)
            check(f"{fake.__name__}.{name} exists", mine is not None)
            if mine is None:
                continue
            check(f"{fake.__name__}.{name} is a property iff Drivetrain's is",
                  isinstance(mine, property) == isinstance(real, property),
                  f"fake property={isinstance(mine, property)}, "
                  f"real property={isinstance(real, property)}")


def test_bump_map() -> None:
    print("\na bump becomes sectors, and stops being them")
    t0 = time.monotonic()
    mem = BumpMemory(memory_s=2.0, sectors=7, hfov_deg=73.0)

    check("nothing remembered is no map at all", mem.fresh_map(t0) is None)

    mem.record(Bump(side="left", at=t0, current_a=20.0))
    omap = mem.fresh_map(t0)
    left = [s for s in omap.sectors if s.angle_deg < 0]
    right = [s for s in omap.sectors if s.angle_deg > 0]
    check("every sector on the bumped side is marked",
          all(s.distance_m == config.BUMP_DISTANCE_M for s in left))
    check("and none on the other side is", all(s.distance_m is None for s in right))
    check("the winning source says which sensor it was",
          all(s.source == "bump" for s in left))

    # Trap 1, the one that halts the robot on arithmetic alone if it is wrong.
    later = t0 + 1.5
    check("the map is stamped NOW, not when the contact happened",
          abs(mem.fresh_map(later).timestamp - later) < 1e-6)
    check("a bump older than its memory is not emitted at all",
          mem.fresh_map(t0 + 2.5) is None)

    # `_roomier_side()` scores a side by its BEST known sector, so a bump that
    # marked one sector would leave the robot free to pivot into what it hit.
    mem.record(Bump(side="left", at=t0, current_a=20.0))
    stereo = sector_map([4.0] * 7, source="stereo-band")
    fused = fuse(stereo, mem.fresh_map(t0))
    angles = [s.angle_deg for s in fused.sectors]
    got = [s.distance_m for s in fused.sectors]
    check("contact beats a clear stereo reading on the bumped side",
          all(d == 0.0 for a, d in zip(angles, got) if a < 0), f"{got}")
    check("and leaves the other side alone to be steered toward",
          all(d == 4.0 for a, d in zip(angles, got) if a > 0), f"{got}")

    reading = Reading(tuple(zip(angles, got)), None, "fused", time.monotonic())
    av = avoider_from_config()
    check("so the escape turns AWAY from the side that stalled",
          av._roomier_side(reading) == 1, str(av._roomier_side(reading)))

    mem.record(Bump(side="both", at=t0, current_a=20.0))
    everywhere = mem.fresh_map(t0)
    check("a two-sided bump marks the centre sector too",
          all(s.distance_m == config.BUMP_DISTANCE_M for s in everywhere.sectors))


def test_bump_reaches_the_policy() -> None:
    print("\nthe drive loop turns a jammed wheel into a stopped robot")
    mem = BumpMemory()
    link = FakeLink()
    body = fake_body(link=link, sensor=FakeSensor(bumps=mem))

    body.hop("forward", (1, 1), 0.2)
    check("with a quiet board, nothing is ever recorded", mem.last is None)
    check("and the loop did ask for telemetry", link.status_requests > 0)

    # Now the board answers, and answers jammed.
    link = FakeLink()
    link.wheels = {"left": wheel(erpm=0, current_a=20.0),
                   "right": wheel(erpm=0, current_a=20.0)}
    mem = BumpMemory()
    body = fake_body(link=link, sensor=FakeSensor(bumps=mem))

    started = time.monotonic()
    body.hop("forward", (1, 1), config.BUMP_CONFIRM_S + 0.3)
    bumped = mem.last is not None
    check("a jammed wheel under duty is recorded as a bump", bumped,
          f"after {time.monotonic() - started:.2f}s")
    if bumped:
        check("on both sides, because both wheels were jammed",
              mem.last.side == "both", mem.last.side)
        check("and the map it produces is the one the policy would see",
              all(s.source == "bump" for s in mem.fresh_map().sectors))

    # The other half of the contract: a wheel that is turning under the same
    # duty and the same current is a robot driving up a ramp, not into a wall.
    link = FakeLink()
    link.wheels = {"left": wheel(erpm=3000, current_a=20.0),
                   "right": wheel(erpm=3000, current_a=20.0)}
    mem = BumpMemory()
    body = fake_body(link=link, sensor=FakeSensor(bumps=mem))
    body.hop("forward", (1, 1), config.BUMP_CONFIRM_S + 0.3)
    check("a wheel turning under load is not a bump, however hard it is pulling",
          mem.last is None)


def main() -> int:
    print(f"AVOID_TICK_S={config.AVOID_TICK_S:g}  CRUISE_DEADMAN_S={config.CRUISE_DEADMAN_S:g}  "
          f"SEEK_LOOK_PERIOD_S={config.SEEK_LOOK_PERIOD_S:g}  "
          f"SEEK_TURN_BURST_S={config.SEEK_TURN_BURST_S:g}")
    for test in (
        test_latch_survives_a_look,
        test_hop_is_unchanged,
        test_deadman,
        test_hold_is_a_burst,
        test_stop_during_a_cruise,
        test_exception_stops_the_wheels,
        test_blind_and_wheelless,
        test_close_is_idempotent,
        test_fuse,
        test_clear_is_not_an_obstacle,
        test_ground_plane_geometry,
        test_tof_pose_lands_in_the_right_sector,
        test_tof_axial_distance,
        test_tof_stale_drops_out,
        test_tof_one_quiet_sensor,
        test_stall_detection,
        test_fakes_match_the_real_link,
        test_bump_map,
        test_bump_reaches_the_policy,
        test_approach,
    ):
        test()

    if FAILURES:
        print(f"\n{len(FAILURES)} failed: " + ", ".join(FAILURES))
        return 1
    print("\nall passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
