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
from orio.avoid import Reading
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

    def set_drive(self, left: int, right: int) -> None:
        with self.lock:
            self.sent.append((time.monotonic(), left, right))

    def take_rejection(self):
        return None

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

    def __init__(self, sectors=CLEAR, age_s: float = 0.0) -> None:
        self.sectors = sectors
        self.age_s = age_s

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
