"""Yoga poses, and a coach that watches one and says what to fix.

Deterministic, like the rest of Orio's real-time side: the LLM chooses the pose
and talks around the session, but the corrections come from fixed rules over
measured body geometry, never from a model guessing. Each pose is a list of
checks in priority order — legs before torso before arms, because a correction
to the foundation often fixes what sits on it — and the coach speaks the first
one that fails, one cue at a time.

## Measured in 2-D, guarded by depth

Every angle here is taken in the picture (pose.py's `deg_2d`), which is honest
only when the limbs lie flat to the camera. On the robot the 3-D angles were
worse, not better: a straight standing knee read 136 deg, from depth samples
that slid off a thin limb onto the floor beside it. So each pose names the view
that keeps its joints in the image plane (`view`), and depth does the job it is
good at: checking the person is square to that view. Shoulder depths come from
the torso, where the depth is solid. A body turned 20 deg away gets "turn
toward me" before any angle is judged, because its angles would be wrong.

## Sided poses

Warrior II and Tree are done on either side. Joints in a check are written with
`{lead}` and `{other}` (e.g. "{lead}_knee"), and the pose's `lead()` decides
from the body which side that is — the bent knee, the standing leg. The choice
is made while the person is getting into the pose and then held until they come
out of it, so a wobble cannot swap which knee is being coached.

## Not a physio

The bands here are generous on purpose, and the cues only ever ask for small
adjustments. Pain is not something the camera can see: whatever drives the
session must stop on "it hurts", and say that coming out of a pose is always
fine. Tree has one check that is about safety rather than form — a foot pressed
on the side of the knee — and it outranks the rest.

Pure logic: no camera, no speech, no threads. `Coach.update()` takes a Person per
frame and returns at most one Cue; the caller speaks it.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from . import config
from .pose import JOINTS, KP, Person, angle_at, framing

OTHER = {"left": "right", "right": "left"}


def _pt(person: Person, name: str) -> np.ndarray | None:
    i = KP[name]
    return person.xy[i] if person.seen(i) else None


def _mid(person: Person, a: str, b: str) -> np.ndarray | None:
    pa, pb = _pt(person, a), _pt(person, b)
    return None if pa is None or pb is None else (pa + pb) / 2.0


def _angle(person: Person, joint: str) -> float | None:
    """2-D angle at a joint named like pose.JOINTS ("left_knee")."""
    pts = [_pt(person, n) for n in JOINTS[joint]]
    return None if any(p is None for p in pts) else angle_at(*pts)


def _leg_len(person: Person, side: str) -> float | None:
    """Hip-to-ankle length in pixels: the yardstick for vertical distances,
    so they mean the same at 1.5 m and at 3 m."""
    hip, knee, ankle = (_pt(person, f"{side}_{j}") for j in ("hip", "knee", "ankle"))
    if hip is None or knee is None or ankle is None:
        return None
    return float(np.linalg.norm(knee - hip) + np.linalg.norm(ankle - knee))


# ── checks ────────────────────────────────────────────────────────────────────
# A check measures one thing and says how far outside its band it is: `error()`
# is 0 inside, negative below, positive above, in the measurement's own units.
# `slack` is the hysteresis — how far past the band a measurement must go before
# the check FAILS; it then passes again only once back inside the band. That gap
# is what keeps a joint resting on a band edge from flickering right/wrong.


@dataclass(frozen=True)
class Angle:
    """A joint angle (2-D, degrees, 180 = straight) within [low, high]."""

    joint: str  # pose.JOINTS name; may use {lead}/{other}
    low: float
    high: float
    below: str | None = None  # cue when the angle is under `low`
    above: str | None = None  # cue when over `high`
    slack: float = 4.0

    @property
    def key(self) -> str:
        return self.joint

    def measure(self, person: Person, sides: dict) -> float | None:
        return _angle(person, self.joint.format(**sides))

    def error(self, v: float) -> float:
        return v - self.low if v < self.low else (v - self.high if v > self.high else 0.0)

    def cue(self, err: float) -> str | None:
        return self.below if err < 0 else self.above


@dataclass(frozen=True)
class Level:
    """The line between two keypoints within `max_deg` of horizontal."""

    a: str
    b: str
    max_deg: float
    say: str
    slack: float = 2.0

    @property
    def key(self) -> str:
        return f"level:{self.a}-{self.b}"

    def measure(self, person: Person, sides: dict) -> float | None:
        pa, pb = _pt(person, self.a.format(**sides)), _pt(person, self.b.format(**sides))
        if pa is None or pb is None:
            return None
        dx, dy = abs(pb[0] - pa[0]), abs(pb[1] - pa[1])
        return math.degrees(math.atan2(dy, dx)) if dx or dy else None

    def error(self, v: float) -> float:
        return max(0.0, v - self.max_deg)

    def cue(self, err: float) -> str | None:
        return self.say


@dataclass(frozen=True)
class Upright:
    """Hip midpoint to shoulder midpoint within `max_deg` of vertical."""

    max_deg: float
    say: str
    slack: float = 2.0

    @property
    def key(self) -> str:
        return "upright"

    def measure(self, person: Person, sides: dict) -> float | None:
        hips = _mid(person, "left_hip", "right_hip")
        shoulders = _mid(person, "left_shoulder", "right_shoulder")
        if hips is None or shoulders is None:
            return None
        dx, dy = shoulders[0] - hips[0], hips[1] - shoulders[1]  # image y points down
        return math.degrees(math.atan2(abs(dx), dy)) if dy > 0 else 90.0

    def error(self, v: float) -> float:
        return max(0.0, v - self.max_deg)

    def cue(self, err: float) -> str | None:
        return self.say


@dataclass(frozen=True)
class FootOffKnee:
    """Tree's safety check: the lifted foot must not press on the standing knee.

    Measures the vertical gap between the lifted ankle and the standing knee, in
    leg lengths. Too small means the foot is on the joint, where sideways
    pressure can hurt it — above or below is fine.
    """

    min_gap: float = 0.1
    say: str = "Move your foot off your knee. Rest it above or below the knee, never on it."
    slack: float = 0.02

    @property
    def key(self) -> str:
        return "foot-off-knee"

    def measure(self, person: Person, sides: dict) -> float | None:
        ankle = _pt(person, f"{sides['other']}_ankle")
        knee = _pt(person, f"{sides['lead']}_knee")
        leg = _leg_len(person, sides["lead"])
        if ankle is None or knee is None or not leg:
            return None
        return abs(float(ankle[1] - knee[1])) / leg

    def error(self, v: float) -> float:
        return min(0.0, v - self.min_gap)  # negative = too close

    def cue(self, err: float) -> str | None:
        return self.say


# ── poses ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class YogaPose:
    name: str
    view: str  # "front": the person faces the camera
    how_to: str  # spoken to get them into it
    checks: tuple
    entered: Callable[[Person, dict], bool]  # roughly in the pose at all?
    lead: Callable[[Person], str | None] | None = None  # which side leads; None = symmetric
    hold_s: float = config.YOGA_HOLD_S


def _mountain_entered(p: Person, sides: dict) -> bool:
    knees = [_angle(p, f"{s}_knee") for s in ("left", "right")]
    return all(k is not None and k > 150 for k in knees)


def _warrior_lead(p: Person) -> str | None:
    """The bent knee leads."""
    left, right = _angle(p, "left_knee"), _angle(p, "right_knee")
    if left is None or right is None:
        return None
    return "left" if left < right else "right"


def _warrior_entered(p: Person, sides: dict) -> bool:
    lead = _angle(p, f"{sides['lead']}_knee")
    back = _angle(p, f"{sides['other']}_knee")
    la, ra = _pt(p, "left_ankle"), _pt(p, "right_ankle")
    ls, rs = _pt(p, "left_shoulder"), _pt(p, "right_shoulder")
    if None in (lead, back) or any(x is None for x in (la, ra, ls, rs)):
        return False
    wide = abs(la[0] - ra[0]) > 1.5 * abs(ls[0] - rs[0])  # feet well past shoulder width
    return lead < 145 and back > 140 and wide


def _tree_lead(p: Person) -> str | None:
    """The standing leg leads: its ankle is the lower one (image y grows down)."""
    la, ra = _pt(p, "left_ankle"), _pt(p, "right_ankle")
    if la is None or ra is None:
        return None
    return "left" if la[1] > ra[1] else "right"


def _tree_entered(p: Person, sides: dict) -> bool:
    stand = _pt(p, f"{sides['lead']}_ankle")
    lifted = _pt(p, f"{sides['other']}_ankle")
    leg = _leg_len(p, sides["lead"])
    if stand is None or lifted is None or not leg:
        return False
    return (stand[1] - lifted[1]) > 0.15 * leg


MOUNTAIN = YogaPose(
    name="Mountain",
    view="front",
    how_to="Stand facing me with your feet together and your arms by your sides.",
    checks=(
        Angle("left_knee", 165, 180, below="Straighten your legs and stand tall."),
        Angle("right_knee", 165, 180, below="Straighten your legs and stand tall."),
        Upright(8, "Stand up straight, with your shoulders over your hips."),
        Level("left_shoulder", "right_shoulder", 6, "Relax your shoulders so they sit level."),
        Angle("left_shoulder", 0, 30, above="Let your arms rest down by your sides."),
        Angle("right_shoulder", 0, 30, above="Let your arms rest down by your sides."),
    ),
    entered=_mountain_entered,
)

WARRIOR_II = YogaPose(
    name="Warrior Two",
    view="front",
    how_to=("Face me and step your feet wide apart. Turn one foot out, bend that knee, "
            "and stretch your arms out to the sides at shoulder height."),
    checks=(
        Angle("{lead}_knee", 80, 110,
              below="Ease off your front knee a little, so it stays over your ankle.",
              above="Bend your front knee deeper, toward a right angle."),
        Angle("{other}_knee", 160, 180, below="Straighten your back leg."),
        Upright(10, "Keep your body upright, don't lean over your front leg."),
        Angle("left_shoulder", 75, 110, below="Lift your arms up to shoulder height.",
              above="Lower your arms to shoulder height."),
        Angle("right_shoulder", 75, 110, below="Lift your arms up to shoulder height.",
              above="Lower your arms to shoulder height."),
        Angle("left_elbow", 155, 180, below="Reach out through your fingertips and straighten your arms."),
        Angle("right_elbow", 155, 180, below="Reach out through your fingertips and straighten your arms."),
    ),
    entered=_warrior_entered,
    lead=_warrior_lead,
)

TREE = YogaPose(
    name="Tree",
    view="front",
    how_to=("Face me and stand on one leg. Bend the other knee out to the side, and rest "
            "that foot on your standing leg, above or below the knee."),
    checks=(
        FootOffKnee(),
        Angle("{lead}_knee", 165, 180, below="Keep your standing leg straight and strong."),
        Angle("{other}_knee", 0, 110, above="Bend your lifted knee more, and draw your foot higher."),
        Level("left_hip", "right_hip", 8, "Keep your hips level."),
        Upright(8, "Stand tall through your spine."),
    ),
    entered=_tree_entered,
    lead=_tree_lead,
)

POSES = {"mountain": MOUNTAIN, "warrior_ii": WARRIOR_II, "tree": TREE}


# ── the coach ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Cue:
    text: str
    kind: str  # "framing", "square", "prompt", "correction", "praise", "release"


# Framing problem (pose.framing) -> what to say about it.
_FRAMING = {
    "feet not visible": "Step back a little so I can see your feet.",
    "cut off at the bottom": "Step back a little so I can see your feet.",
    "head not visible": "Step back a little so I can see all of you.",
    "cut off at the top": "Step back a little so I can see all of you.",
    "cut off at the side": "Move toward the middle, where I can see you.",
}

# How long the pose must be held (s) before it counts as entered, and lost before
# it counts as left. Both stop a single odd frame from changing the state.
_ENTER_S = 0.5
_EXIT_S = 1.5
_LOST_S = 3.0  # nobody in view this long before saying so
_PROMPT_S = 10.0  # waiting this long for them to get into the pose -> repeat how_to


@dataclass
class _Track:
    """One check's smoothed measurement and whether it is currently failing."""

    value: float | None = None
    failing: bool = False


@dataclass
class Coach:
    """Watches one pose, frame by frame. `update()` returns what to say, if anything.

    States: "waiting" (not in the pose yet), "adjusting" (in it, something to
    fix), "holding" (right, the hold timer is running), "done" (held long enough).
    The hold timer pauses while adjusting rather than resetting — being told to
    lift your arms should not cost you the ten seconds already held.
    """

    pose: YogaPose
    cue_interval_s: float = config.YOGA_CUE_INTERVAL_S
    smoothing_s: float = config.YOGA_SMOOTHING_S
    square_tol_m: float = config.YOGA_SQUARE_TOL_M
    clock: Callable[[], float] = time.monotonic

    state: str = field(default="waiting", init=False)
    held_s: float = field(default=0.0, init=False)
    lead: str | None = field(default=None, init=False)
    failing: str | None = field(default=None, init=False)  # key of the check being coached
    last_cue: Cue | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self._tracks: dict[str, _Track] = {}
        self._last_t: float | None = None
        self._cue_t = -math.inf
        self._seen_t: float | None = None
        self._entered_since: float | None = None
        self._out_since: float | None = None
        self._waiting_since: float | None = None

    # ── speaking ─────────────────────────────────────────────────────────────

    def _say(self, t: float, text: str, kind: str, urgent: bool = False) -> Cue | None:
        """A cue, if the interval since the last one allows it.

        The same words again wait twice as long: they are still working on it.
        `urgent` cues (praise, release) only wait a short beat, so "that's it"
        lands while they are still in the moment that earned it.
        """
        gap = t - self._cue_t
        if urgent:
            if gap < 1.0:
                return None
        else:
            need = self.cue_interval_s
            if self.last_cue is not None and self.last_cue.text == text:
                need *= 2
            if gap < need:
                return None
        self._cue_t = t
        self.last_cue = Cue(text, kind)
        return self.last_cue

    # ── measuring ────────────────────────────────────────────────────────────

    def _smooth(self, key: str, raw: float | None, dt: float) -> _Track:
        tr = self._tracks.setdefault(key, _Track())
        if raw is None:
            # Not seen: no judgement either way. Forget the value so a joint that
            # reappears is judged on what it is now, not what it was.
            tr.value, tr.failing = None, False
        elif tr.value is None or dt <= 0:
            tr.value = raw
        else:
            a = 1.0 - math.exp(-dt / self.smoothing_s) if self.smoothing_s > 0 else 1.0
            tr.value += a * (raw - tr.value)
        return tr

    def _judge(self, tr: _Track, err: float, slack: float) -> None:
        if tr.failing:
            tr.failing = err != 0.0
        else:
            tr.failing = abs(err) > slack

    def _square(self, person: Person, dt: float) -> str | None:
        """A turn cue if the person is not facing the camera squarely, else None."""
        if self.pose.view != "front" or person.xyz is None:
            return None
        zl = person.xyz[KP["left_shoulder"], 2]
        zr = person.xyz[KP["right_shoulder"], 2]
        raw = float(zl - zr) if np.isfinite(zl) and np.isfinite(zr) else None
        tr = self._smooth("square", raw, dt)
        if tr.value is None:
            return None
        err = max(0.0, abs(tr.value) - self.square_tol_m)
        self._judge(tr, err, 0.03)
        if not tr.failing:
            return None
        far = "left" if tr.value > 0 else "right"  # larger z = farther from the camera
        return f"Turn to face me. Bring your {far} shoulder toward me a little."

    # ── the frame ────────────────────────────────────────────────────────────

    def update(self, person: Person | None, width: int, height: int,
               t: float | None = None) -> Cue | None:
        t = self.clock() if t is None else t
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if self._waiting_since is None:
            self._waiting_since = t
        if self.state == "done":
            return None

        if person is None:
            self.failing = None
            if self._seen_t is not None and t - self._seen_t < _LOST_S:
                return None
            if self._seen_t is None and t - self._waiting_since < _LOST_S:
                return None
            return self._say(t, "I can't see you. Step in front of me.", "framing")
        self._seen_t = t

        problems = framing(person, width, height)
        if problems:
            self.failing = "framing"
            return self._say(t, _FRAMING.get(problems[0], "Step back a little."), "framing")

        # Which side leads: re-decided only while out of the pose.
        if self.state == "waiting" or self.lead is None:
            self.lead = self.pose.lead(person) if self.pose.lead else "left"
            if self.lead is None:
                return None
        sides = {"lead": self.lead, "other": OTHER[self.lead]}

        if not self._in_pose(person, sides, t):
            self.failing = None
            if t - self._waiting_since >= _PROMPT_S and t - self._cue_t >= _PROMPT_S:
                return self._say(t, self.pose.how_to, "prompt")
            return None

        turn = self._square(person, dt)
        if turn is not None:
            self.state, self.failing = "adjusting", "square"
            return self._say(t, turn, "square")

        for check in self.pose.checks:
            tr = self._smooth(check.key, check.measure(person, sides), dt)
            if tr.value is None:
                continue
            err = check.error(tr.value)
            self._judge(tr, err, check.slack)
            if tr.failing:
                self.state, self.failing = "adjusting", check.key.format(**sides)
                text = check.cue(err)
                return self._say(t, text, "correction") if text else None

        self.failing = None
        if self.state != "holding":
            self.state = "holding"
            return self._say(t, "That's it. Hold it there, and breathe.", "praise", urgent=True)
        self.held_s += dt
        if self.held_s >= self.pose.hold_s:
            self.state = "done"
            return self._say(t, "And release. Well done.", "release", urgent=True)
        return None

    def _in_pose(self, person: Person, sides: dict, t: float) -> bool:
        """Debounced `pose.entered`: in after _ENTER_S of it, out after _EXIT_S without."""
        now_in = self.pose.entered(person, sides)
        if self.state == "waiting":
            if not now_in:
                self._entered_since = None
                return False
            if self._entered_since is None:
                self._entered_since = t
            if t - self._entered_since < _ENTER_S:
                return False
            self.state, self._out_since = "adjusting", None
            return True
        if now_in:
            self._out_since = None
            return True
        if self._out_since is None:
            self._out_since = t
        if t - self._out_since < _EXIT_S:
            return True  # a wobble, not a step out
        # Out of the pose: start over, side and smoothing included.
        self.state, self._entered_since, self._waiting_since = "waiting", None, t
        self._tracks.clear()
        return False
