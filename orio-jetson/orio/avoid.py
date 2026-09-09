"""The obstacle-avoidance policy, and the stereo thread that feeds it.

Lifted wholesale out of `tools/teleop_guarded.py`, which is where it was built,
tuned, and tested against every scenario the comments below cite. It lives here
now because it is no longer a bench tool's private business: every move the
robot makes goes through it (see `body.py`), and the teleop tool imports it from
here, so there is one policy rather than two that drift apart.

Two objects. `Sensor` runs the stereo pipeline on its own thread and publishes
the latest reading; `Avoider` turns a reading into a wheel command. The split
matters — a camera that wedges must not also wedge the loop holding the robot's
watchdog alive. A stalled thread simply stops republishing, the reading goes
stale, and the policy blocks. Failure points the safe way.

Tunables come from `config.AVOID_*` via `avoider_from_config()`. The teleop tool
passes its own CLI values to the constructor instead, so it can keep sweeping
them without the app's defaults moving underneath it.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from . import config
from .stereo import ObstacleDetector

# Orio measures 0.70 m across the drive wheels, so its body half-width is
# 0.35 m. The forward corridor adds a margin on top: depth is noisy at the
# sector edges, the cameras sit forward of the point the robot pivots about,
# and a chassis is never exactly as wide as its track.
ROBOT_WIDTH_M = 0.70
CORRIDOR_MARGIN_M = 0.05
DEFAULT_HALF_WIDTH_M = round(ROBOT_WIDTH_M / 2.0 + CORRIDOR_MARGIN_M, 3)

@dataclass(frozen=True)
class Reading:
    """The latest stereo reading, or the error that prevented one.

    `sectors` is (angle_deg, distance_m or None) per sector, left to right —
    the whole map, because choosing a heading needs the sectors either side of
    straight ahead, not just the clearance in front.
    """

    sectors: tuple[tuple[float, float | None], ...]
    clearance_m: float | None
    describe: str
    timestamp: float
    error: str | None = None

    @staticmethod
    def failed(exc: Exception) -> "Reading":
        return Reading(
            sectors=(),
            clearance_m=None,
            describe="stereo read failed",
            timestamp=time.monotonic(),
            error=f"{type(exc).__name__}: {exc}",
        )


class Sensor:
    """Runs the stereo pipeline on its own thread and publishes the latest
    reading.

    Threaded rather than inline because a reading costs ~33 ms (measured, and
    frame-rate bound), which would be felt as keyboard lag on a 30 ms tick — and
    more importantly because a camera that wedges must not also wedge the loop
    that is holding the robot's watchdog alive. A stalled thread simply stops
    republishing, the reading goes stale, and the guard blocks. Failure ends up
    pointing the safe way.
    """

    def __init__(self, guard_sectors: int = 3) -> None:
        self._guard_sectors = guard_sectors
        self._detector = ObstacleDetector()
        self._reading: Reading | None = None
        # The rectified left eye from the most recent reading. Published here
        # because these two cameras are the only ones there are: with avoidance
        # always on, this thread holds both sensors for the whole session, and
        # anything else wanting a picture (the vision tool) has to be handed
        # this frame rather than opening a handle Argus will not give it.
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.calibrated = False

    def start(self) -> None:
        # The first reading opens both Argus pipelines and takes ~2 s. Do it
        # here, before the terminal goes into cbreak mode, so any camera error
        # is readable and lands before the operator can press a key.
        omap, frame, _depth = self._detector.sense_with_frames()
        self.calibrated = omap.calibrated
        self._publish(omap, frame)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _publish(self, omap, frame) -> None:
        with self._lock:
            self._frame = frame
            self._reading = Reading(
                sectors=tuple((s.angle_deg, s.distance_m) for s in omap.sectors),
                clearance_m=omap.clearance_ahead(self._guard_sectors),
                describe=omap.describe(),
                timestamp=time.monotonic(),
            )

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                omap, frame, _depth = self._detector.sense_with_frames()
                self._publish(omap, frame)
            except Exception as exc:
                with self._lock:
                    self._reading = Reading.failed(exc)
                time.sleep(0.5)  # a failing camera should not spin the CPU

    @property
    def reading(self) -> Reading | None:
        with self._lock:
            return self._reading

    def snapshot(self):
        """`(Reading, frame)` from the same tick.

        Taking `.reading` and `.frame` separately can straddle a republish and
        pair a bounding box with depth measured from a different picture — which
        is exactly the pairing anything aiming at a detection depends on.
        """
        with self._lock:
            return self._reading, self._frame

    @property
    def frame(self):
        """The rectified left eye from the latest reading, or None.

        Shared, not copied: treat it as read-only. The sensor thread replaces
        the reference rather than writing into the array, so a reader gets
        either the old frame whole or the new one whole.
        """
        with self._lock:
            return self._frame

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._detector.close()


@dataclass(frozen=True)
class Decision:
    """What the avoider decided, and why."""

    left: int
    right: int
    state: str  # "cruise" | "steer" | "pivot" | "halted" | "off" | "manual"
    reason: str
    heading_deg: float = 0.0


class Avoider:
    """Turns a sector map into a steering command.

    A reactive policy, deliberately: it has no map, no memory of where it has
    been, and no goal beyond "keep going forward without hitting anything".
    That is the right amount of machinery for a bench tool, but it does set the
    limits worth knowing:

    * It wanders rather than travels. Having avoided something it carries on in
      whatever direction it ended up facing; nothing pulls it back to its
      original heading, because nothing here knows what that was.
    * A concave dead end is escaped only by luck and the back-off timer, not by
      reasoning. In simulation it does get out — but by reversing and trying
      another way, which is a different thing from knowing it is in a trap.
    * There is no rear or side sensing. The corridor test covers what the
      cameras can see ahead; the back-off reverses blind.
    """

    def __init__(
        self,
        stop_m: float,
        clear_m: float,
        min_scale: float,
        stale_s: float,
        turn_penalty: float,
        turn_gain: float,
        smooth: float,
        hysteresis_m: float,
        half_width: float,
        release_m: float,
        commit_clear_s: float,
        pivot_timeout_s: float,
        backoff_s: float,
    ) -> None:
        self.stop_m = stop_m
        self.clear_m = clear_m
        self.min_scale = min_scale
        self.stale_s = stale_s
        self.turn_penalty = turn_penalty
        self.turn_gain = turn_gain
        self.smooth = smooth
        self.hysteresis_m = hysteresis_m
        self.half_width = half_width
        self.release_m = release_m
        self.commit_clear_s = commit_clear_s
        self.enabled = True
        self.pivot_timeout_s = pivot_timeout_s
        self.backoff_s = backoff_s
        self._turn = 0.0  # smoothed steering state, permille
        self._committed_side = 0  # -1 left, +1 right, 0 none — damps oscillation
        self._blocked = False
        self._stuck_s = 0.0
        self._clear_s = 0.0
        self._must_clear = False
        self._last_tick: float | None = None
        self._backoff_until = 0.0

    def reset(self) -> None:
        self._turn = 0.0
        self._committed_side = 0
        self._blocked = False
        self._stuck_s = 0.0
        self._clear_s = 0.0
        self._must_clear = False
        self._backoff_until = 0.0

    # ── helpers ──────────────────────────────────────────────────────────────

    def _ahead(self, reading: Reading) -> float | None:
        """Nearest KNOWN obstacle inside the robot's forward corridor, or None
        if nothing in it can be seen (which is not the same as clear).

        The corridor is geometric, not a fixed count of centre sectors, and the
        difference is the difference between avoiding an obstacle and driving a
        shoulder into it. A sector counts when the point it reports lies within
        `half_width` of the centre line — `distance * sin(angle)`. That window
        widens in angle as things get closer, which is exactly the behaviour
        wanted: at 3 m a reading 21 deg off-axis is 1.1 m to the side and no
        threat, while at 0.5 m the same angle is 0.18 m away and squarely in
        the robot's path.

        Using the middle three sectors instead, as a stop-only guard could get
        away with, spans just +/-0.14 m at 0.5 m range — a fifth of a robot
        that is 0.70 m across the wheels.
        An obstacle being steered around would leave that window while still
        physically ahead, the clearance would jump to whatever lay beyond it,
        and the policy would accelerate into it. That is not hypothetical: it
        collided in every obstacle scenario in simulation before this changed.
        """
        near = None
        for angle, distance in reading.sectors:
            if distance is None:
                continue
            if abs(distance * math.sin(math.radians(angle))) <= self.half_width:
                near = distance if near is None else min(near, distance)
        return near

    def _pick(self, reading: Reading) -> tuple[float, float] | None:
        """Best (angle, distance) to head for, or None if nothing qualifies.

        Only KNOWN sectors are candidates. Unknown ones are skipped rather than
        scored as obstacles: sector 0 is permanently unknown (the rectification
        border), and treating that as a wall would bias every turn rightward.
        """
        best = None
        best_score = float("-inf")
        for angle, distance in reading.sectors:
            if distance is None or distance <= self.stop_m:
                continue
            # Distance, minus a penalty for how far off-axis it is, plus a
            # bonus for staying on the side already committed to.
            score = distance - self.turn_penalty * abs(angle) / 45.0
            if self._committed_side and (angle > 0) == (self._committed_side > 0):
                score += self.hysteresis_m
            if score > best_score:
                best_score, best = score, (angle, distance)
        return best

    def _mix(self, linear: int, turn: float, duty: int) -> tuple[int, int]:
        """Differential mix. `turn` > 0 steers LEFT, matching the sign of the
        firmware's wheel pair for an in-place left turn (left reverse, right
        forward)."""
        left = max(-duty, min(duty, round(linear - turn)))
        right = max(-duty, min(duty, round(linear + turn)))
        return left, right

    # ── policy ───────────────────────────────────────────────────────────────

    def decide(self, latch, duty: int, reading: Reading | None) -> Decision:
        now = time.monotonic()
        dt = 0.0 if self._last_tick is None else min(0.2, now - self._last_tick)
        self._last_tick = now

        if not self.enabled:
            return Decision(*_signs(latch, duty), "off", "avoidance disabled")

        # Only forward cruise is planned. Reverse and manual turns pass through
        # untouched so a cornered robot can always be recovered by hand.
        if latch is None or latch != (1, 1):
            self.reset()
            return Decision(*_signs(latch, duty), "manual", "manual command")

        if reading is None:
            return Decision(0, 0, "halted", "no stereo reading yet")
        age = now - reading.timestamp
        if age > self.stale_s:
            return Decision(0, 0, "halted", f"stereo reading stale ({age:.1f}s)")
        if reading.error is not None:
            return Decision(0, 0, "halted", reading.error)

        ahead = self._ahead(reading)

        # Whether forward motion is allowed is a Schmitt trigger, not a bare
        # comparison: `ahead` sits within a few centimetres of --stop-m for as
        # long as the robot is working around an obstacle, and depth noise
        # alone then flips the branch every other tick. That chatter is not
        # cosmetic — it reversed the wheels several times a second and the
        # robot dithered in place instead of turning. Blocking latches at
        # --stop-m and only releases --release-m beyond it.
        if ahead is None:
            blocked = True  # nothing KNOWN in the corridor: blind, not clear
        elif self._blocked:
            blocked = ahead <= self.stop_m + self.release_m
        else:
            blocked = ahead <= self.stop_m
        self._blocked = blocked

        # "Stuck" is measured as time spent pinned near the floor, not as
        # consecutive pivot ticks. The two differ exactly when it matters: a
        # robot alternating pivot/steer on the boundary is making no progress
        # at all, yet every steer tick would reset a consecutive-tick counter
        # and the escape would never fire.
        if ahead is not None and ahead < self.stop_m + self.release_m:
            self._stuck_s += dt
        else:
            self._stuck_s = max(0.0, self._stuck_s - 2.0 * dt)

        # After backing off, turn to face clear road BEFORE driving again.
        # Without this the robot spends the space it just bought driving
        # straight back into the same obstacle: back off, steer, re-approach,
        # back off, forever. In simulation that limit cycle held it at the same
        # 2 m mark for the entire run. Forward motion stays suspended until the
        # way ahead is genuinely open, not merely past the stop floor.
        if self._must_clear and ahead is not None and ahead >= self.clear_m:
            self._must_clear = False

        if not blocked and not self._must_clear:
            # 1. Road clear: go straight, and unwind any turn in progress.
            if ahead is not None and ahead >= self.clear_m:
                # Commitment survives a moment of clear road. Clearing it here
                # unconditionally is wrong for the same reason the raw
                # comparison was: cruise and steer alternate while the robot is
                # still working past an obstacle, so every clear tick wiped the
                # side and the next steer re-picked freely. It flip-flopped
                # between the left and right gap and passed neither.
                self._clear_s += dt
                if self._clear_s > self.commit_clear_s:
                    self._committed_side = 0
                self._stuck_s = 0.0
                self._turn *= 1.0 - self.smooth
                left, right = self._mix(duty, self._turn, duty)
                return Decision(left, right, "cruise", f"{ahead:.2f} m clear ahead")

            # 2. Something is in the way but a sector is open: steer for it.
            target = self._pick(reading)
            if target is not None:
                angle, distance = target
                self._clear_s = 0.0
                # Only a genuinely off-axis heading changes the committed side.
                # Letting a 0 deg target clear it lets the side flip freely on
                # the next pivot, which is half of the dithering above.
                if angle:
                    self._committed_side = 1 if angle > 0 else -1
                desired = -self.turn_gain * (angle / 45.0) * duty
                self._turn += self.smooth * (desired - self._turn)
                near = min(ahead if ahead is not None else distance, distance)
                span = max(self.clear_m - self.stop_m, 1e-6)
                by_range = self.min_scale + (1.0 - self.min_scale) * min(
                    1.0, max(0.0, (near - self.stop_m) / span)
                )
                by_turn = 1.0 - 0.5 * min(1.0, abs(self._turn) / max(duty, 1))
                linear = round(duty * by_range * by_turn)
                left, right = self._mix(linear, self._turn, duty)
                return Decision(
                    left, right, "steer",
                    f"gap at {angle:+.0f} deg, {distance:.2f} m", angle,
                )

        # 3. Backing off, because turning alone cannot always clear the corridor.
        #
        #    Close in, the angle an obstacle must reach to leave the corridor
        #    exceeds what the cameras can resolve. A sector's depth is charged
        #    to its CENTRE angle, and the outermost centre is 31.8 deg (74.1
        #    deg rectified hfov over 7 sectors) — an obstacle further out than
        #    that is simply not seen. So turning can only clear the corridor
        #    beyond half_width / sin(31.8 deg), and nearer than that the
        #    obstacle stays in it however far the robot turns: it pivots on the
        #    spot indefinitely, which it did in simulation for 927 of 1200
        #    ticks. Reversing is the only move that changes the geometry, since
        #    the angle required shrinks as distance grows.
        #
        #    That radius scales with the corridor, so it is not a number to
        #    carry over from memory: 0.48 m when the corridor was sized for a
        #    0.25 m half-width, but 0.76 m at the 0.40 m this robot needs. Keep
        #    --stop-m at or above it, or every close encounter ends up here
        #    instead of being turned away from.
        #
        #    NOTE there is no rear sensor. This reverses BLIND, which is why it
        #    is slow, brief, and entered only once genuinely stuck.
        if now < self._backoff_until or self._stuck_s > self.pivot_timeout_s:
            if now >= self._backoff_until:
                self._backoff_until = now + self.backoff_s
                self._stuck_s = 0.0
                self._committed_side = 0
                self._must_clear = True
            speed = round(duty * self.min_scale)
            self._turn = 0.0
            return Decision(-speed, -speed, "backoff", "stuck — backing off to turn")

        # 4. Boxed in: pivot in place toward whichever side has more room. No
        #    forward component at all — this is the floor the stop-only guard
        #    used to enforce, and it still holds.
        # Latch the turn direction for the whole pivot episode. Re-deciding it
        # every tick produces a perfect limit cycle: turning left moves the
        # obstacle rightward through the frame, which makes the right side look
        # more open, which reverses the turn, which moves it back. Measured in
        # simulation as a +4.6/-9.2 degree oscillation that ran forever, and no
        # per-tick margin fixes it — the evidence really does swing from 0.5 m
        # to 4 m as the obstacle crosses the centre line.
        #
        # Latching is only safe because the episode is bounded: the stuck timer
        # above forces a back-off, and that clears the commitment so the next
        # attempt is free to choose the other way. An unbounded latch is what
        # made an earlier version pivot into a wall indefinitely.
        side = self._committed_side or self._roomier_side(reading)
        if side is not None:
            self._committed_side = side
            self._turn = -side * self.turn_gain * duty
            left, right = self._mix(0, self._turn, duty)
            where = "left" if side < 0 else "right"
            why = "turning to clear" if self._must_clear else "boxed in"
            return Decision(left, right, "pivot", f"{why}, pivoting {where}")

        # 5. Nothing known anywhere. Blind is not a heading.
        self.reset()
        return Decision(0, 0, "halted", "no known clearance in any sector")

    def _roomier_side(self, reading: Reading) -> int | None:
        """-1 (left) or +1 (right), by the best KNOWN distance on each side.
        None when neither side can be seen at all.

        Commitment biases this choice by `hysteresis_m` but must never lock it.
        Returning the committed side unconditionally — which this did at first —
        means a robot that commits toward a wall pivots into that wall forever,
        because the evidence that the other side is now wide open can no longer
        change its mind. In simulation it spent over half its time pivoting on
        the spot against a corridor wall.
        """
        left = [d for a, d in reading.sectors if a < 0 and d is not None]
        right = [d for a, d in reading.sectors if a > 0 and d is not None]
        if not left and not right:
            return None
        best_left = max(left, default=0.0)
        best_right = max(right, default=0.0)
        if self._committed_side < 0:
            best_left += self.hysteresis_m
        elif self._committed_side > 0:
            best_right += self.hysteresis_m
        return -1 if best_left > best_right else 1


def _signs(latch, duty: int) -> tuple[int, int]:
    """The raw wheel pair a latched direction asks for, ungated."""
    if latch is None:
        return (0, 0)
    return (latch[0] * duty, latch[1] * duty)


def avoider_from_config() -> Avoider:
    """The policy as configured for the robot, rather than for a bench sweep."""
    return Avoider(
        stop_m=config.AVOID_STOP_M,
        clear_m=config.AVOID_CLEAR_M,
        min_scale=config.AVOID_MIN_SCALE,
        stale_s=config.AVOID_STALE_S,
        turn_penalty=config.AVOID_TURN_PENALTY,
        turn_gain=config.AVOID_TURN_GAIN,
        smooth=config.AVOID_SMOOTH,
        hysteresis_m=config.AVOID_HYSTERESIS_M,
        half_width=config.AVOID_HALF_WIDTH_M,
        release_m=config.AVOID_RELEASE_M,
        commit_clear_s=config.AVOID_COMMIT_CLEAR_S,
        pivot_timeout_s=config.AVOID_PIVOT_TIMEOUT_S,
        backoff_s=config.AVOID_BACKOFF_S,
    )
