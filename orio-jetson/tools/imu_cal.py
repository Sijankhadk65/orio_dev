"""Shared parts of the two IMU calibration tools.

`imu_calibrate.py` does every step by hand; `imu_calibrate_drive.py` drives the
steps it can and asks for the two it cannot. Both need the same windows, the
same shim instructions, and — the part that matters — the same rules for
deciding what a measurement proved. Written once here so the two cannot drift
into disagreeing about which sign means nose-down.

Not a package: the tools run as scripts, so `tools/` is already on the path.
"""

from __future__ import annotations

import statistics as st
import sys
import threading
import time

SETTLE_S = 1.2  # the body rocks after being moved; let it die down
REST_S = 2.0
SAMPLE_S = 3.0

STILL_SIGMA_DEG = 0.05  # a hand resting on the body shows up well above this
MIN_TILT_DEG = 0.8  # below this a shim step proves nothing
MIN_YAW_DEG = 10.0  # below this a turn proves nothing
RATE_HZ = 100.0

DIAGRAM = """
    Which end is which, and which side is LEFT:

            FRONT  =  the two drive wheels (hub motors), the way the robot drives
             ___________
            |  o     o  |   <- drive wheels, left one on YOUR left when you
     LEFT   |           |      stand BEHIND the robot looking the way it faces
      side  |    []     |   <- the IMU, on the plank
            |     o     |   <- castor (the small swivel wheel)
             -----------
            BACK  =  the castor end. This is the light end. Shim here first.

    "left" always means the ROBOT's left, i.e. your left when you stand
    behind it and look forward with it.
"""


class Window:
    """Samples gathered over one span of time, and the means callers want."""

    def __init__(self, samples: list) -> None:
        self.samples = samples

    def mean(self, name: str) -> float:
        return st.mean(getattr(r, name) for r in self.samples) if self.samples else 0.0

    @property
    def yaw_span(self) -> float:
        """How far yaw moved across the window, wrap-safe."""
        if len(self.samples) < 2:
            return 0.0
        first, last = self.samples[0].heading_deg, self.samples[-1].heading_deg
        return (last - first + 180.0) % 360.0 - 180.0

    @property
    def sigma(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return max(
            st.pstdev([r.pitch_deg for r in self.samples]),
            st.pstdev([r.roll_deg for r in self.samples]),
        )


class Collector:
    """Records every packet inside a measuring window, and none outside one.

    Hand this to `RvcReader(on_reading=...)`. Polling the reader instead loses
    most of the packets: a serial read hands over several at once and only the
    last of each burst survives.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: list | None = None
        self.all: list = []

    def __call__(self, reading) -> None:
        with self._lock:
            self.all.append(reading)
            if self._samples is not None:
                self._samples.append(reading)

    def start(self) -> None:
        with self._lock:
            self._samples = []

    def stop(self) -> Window:
        with self._lock:
            samples, self._samples = self._samples or [], None
        return Window(samples)


def live_preview(reader, note: str) -> None:
    """Show the numbers moving while the robot is being put in place.

    Without this the tool is a black box with a blinking cursor: a wrongly
    placed shim and a dead link look identical until the summary at the end.
    """
    print(f"    live: {note}")
    print("    (press Enter when the robot is in place and your hands are off)")
    stop = threading.Event()

    def show() -> None:
        while not stop.is_set():
            r = reader.fresh(max_age_s=0.5)
            if r is not None:
                sys.stdout.write(
                    f"\r      front-back {r.pitch_deg:+7.2f} deg   "
                    f"side-lean {r.roll_deg:+7.2f} deg   heading {r.heading_deg:+7.1f} deg  "
                )
            else:
                sys.stdout.write("\r      no data from the IMU ...                      ")
            sys.stdout.flush()
            time.sleep(0.1)

    thread = threading.Thread(target=show, daemon=True)
    thread.start()
    try:
        input()
    finally:
        stop.set()
        thread.join(timeout=0.5)
        sys.stdout.write("\r" + " " * 78 + "\n")


def measure(collector: Collector, seconds: float, settle_s: float = SETTLE_S) -> Window:
    """One still window: settle, then record."""
    print(f"    holding still for {settle_s:g}s, then measuring for {seconds:g}s ...")
    time.sleep(settle_s)
    collector.start()
    time.sleep(seconds)
    window = collector.stop()

    if not window.samples:
        print("    NOTHING MEASURED — the IMU stopped sending. Check the wire.")
        return window

    print(f"    measured: front-back {window.mean('pitch_deg'):+.2f} deg, "
          f"side-lean {window.mean('roll_deg'):+.2f} deg  ({len(window.samples)} readings)")
    expected = seconds * RATE_HZ
    if len(window.samples) < expected * 0.8:
        print(f"    PROBLEM: only {len(window.samples)} readings, expected about {expected:.0f}.")
        print("             The link is dropping data — check the TX wire and the 3.3 V.")
    if window.sigma > STILL_SIGMA_DEG:
        print(f"    PROBLEM: the robot was still moving (wobble {window.sigma:.2f} deg).")
        print("             Take your hands off, let it settle, and run this step again.")
    return window


def hand_step(reader, collector: Collector, title: str, instructions: list[str],
              finds: str, expect: str, seconds: float) -> Window:
    """Ask for a pose, wait for Enter with the live numbers up, then measure."""
    print("\n" + "-" * 78)
    print(title)
    print("-" * 78)
    for line in instructions:
        print(f"    {line}" if line.startswith(" ") else f"  - {line}")
    print(f"\n    This step finds: {finds}")
    live_preview(reader, expect)
    return measure(collector, seconds)


# The two poses a robot cannot put itself into: on a flat floor there is no
# body tilt to measure, only the few tenths of a degree it rocks by.
NOSE_DOWN_STEP = dict(
    title="Tilt the FRONT down, by raising the BACK",
    instructions=[
        "Chock both drive wheels (front) so the robot cannot roll.",
        "Put the shim (a book, 20-40 mm) on the floor just behind the castor.",
        "Ease the CASTOR up onto the shim, so the BACK is higher and the FRONT dips.",
        "  Do NOT lift the robot — roll or slide it on.",
        "Hands off once it is sitting on the shim.",
    ],
    finds="which number is the front-to-back tilt, and which way is down",
    expect="one number should move by a few degrees, the other should barely move",
)

# Raising ONE front wheel does not roll this robot, it tilts it diagonally.
# The chassis stands on three points — two drive wheels at the front, the castor
# at the back — so a shim under one wheel tilts the body about the line from the
# castor to the OTHER wheel. Measured 2026-09-22: a 30 mm shim under the left
# wheel moved the front-back number +4.1 deg and the side-lean only +1.7 deg,
# and the tool rightly refused to call that a roll measurement.
#
# The fix is to do it on both sides. Raising the left and raising the right
# produce the SAME front-back component and OPPOSITE side-lean components, so
# the difference of the two cancels the pitch and leaves pure roll. The castor
# sits on the centreline, so the nose-down step needs no such treatment.
LEFT_UP_STEP = dict(
    title="Tilt the LEFT side up",
    instructions=[
        "Take the shim out from under the castor; put the robot flat again.",
        "Now put the shim under the LEFT drive wheel only.",
        "  (left = your left when standing BEHIND the robot, looking forward)",
        "Roll that wheel onto it so the LEFT side sits higher than the right.",
        "Hands off.",
    ],
    finds="the side-to-side lean, half of it (the other half is the next step)",
    expect="both numbers will move — that is expected, the next step cancels the tilt",
)

RIGHT_UP_STEP = dict(
    title="Now the same on the RIGHT side",
    instructions=[
        "Move the shim to the RIGHT drive wheel, same thickness, same position.",
        "Roll that wheel onto it so the RIGHT side sits higher than the left.",
        "Hands off.",
    ],
    finds="the other half: left minus right is pure side-lean, with the nose tilt cancelled",
    expect="the side-lean should swing the OTHER way from the last step",
)


def tilt_result(rest: Window, tilted: Window, label: str, nose: bool):
    """Which angle a shim step moved, which way, and whether to believe it.

    `nose` says which pose this was: the castor shim puts the robot NOSE DOWN,
    which must report negative (pitch + is nose up); the wheel shim puts the
    LEFT SIDE UP, i.e. leaning right, which must report positive.

    Returns `(angle_name, sign, ok)`. `ok` is False when the step proved
    nothing — too little tilt, or a shim so far off centre that both angles
    moved. A sign guessed from noise is worse than an honest UNKNOWN.
    """
    d_pitch = tilted.mean("pitch_deg") - rest.mean("pitch_deg")
    d_roll = tilted.mean("roll_deg") - rest.mean("roll_deg")
    angle, delta, other = (
        ("pitch", d_pitch, d_roll) if abs(d_pitch) >= abs(d_roll) else ("roll", d_roll, d_pitch)
    )
    moved = "front-back" if angle == "pitch" else "side-lean"

    print(f"  {label}")
    print(f"    front-back changed {d_pitch:+7.2f} deg      side-lean changed {d_roll:+7.2f} deg")

    ok = True
    if abs(delta) < MIN_TILT_DEG:
        ok = False
        print(f"    COULD NOT TELL: the robot barely tilted ({abs(delta):.2f} deg).")
        print("                    Use a thicker shim (30-40 mm), make sure the wheel or")
        print("                    castor is properly up on it, and run this step again.")
    elif abs(other) > abs(delta) * 0.5:
        ok = False
        print(f"    COULD NOT TELL: the other number moved {other:+.2f} deg as well, which")
        print("                    means the shim was off to one side. Centre it under the")
        print("                    wheel (or the castor) and run this step again.")
    else:
        print(f"    -> this tilt is the {moved.upper()} number ({abs(delta):.1f} deg of tilt)")

    sign = -1 if (delta > 0) == nose else 1
    return angle, sign, ok


def roll_result(left_up: Window, right_up: Window):
    """Which angle is side-lean, its sign, and whether to believe it.

    Takes the DIFFERENCE of the two wheel-shim poses. Each one tilts the robot
    diagonally (see the note above `LEFT_UP_STEP`); halving the difference
    cancels the shared nose-up component and leaves the roll.

    Left side up means leaning right, which must report positive.
    """
    d_pitch = (left_up.mean("pitch_deg") - right_up.mean("pitch_deg")) / 2
    d_roll = (left_up.mean("roll_deg") - right_up.mean("roll_deg")) / 2
    angle, delta, other = (
        ("pitch", d_pitch, d_roll) if abs(d_pitch) >= abs(d_roll) else ("roll", d_roll, d_pitch)
    )
    moved = "front-back" if angle == "pitch" else "side-lean"

    print("  Left side up vs right side up (the difference is pure lean):")
    print(f"    front-back {d_pitch:+7.2f} deg      side-lean {d_roll:+7.2f} deg")

    ok = True
    if abs(delta) < MIN_TILT_DEG:
        ok = False
        print(f"    COULD NOT TELL: only {abs(delta):.2f} deg of lean either side.")
        print("                    Use a thicker shim (30-40 mm) and do both wheels again.")
    elif abs(other) > abs(delta) * 0.5:
        ok = False
        print(f"    COULD NOT TELL: the other number still moved {other:+.2f} deg after the")
        print("                    two sides cancelled, so the shims were not in matching")
        print("                    positions. Same thickness, same spot on each wheel.")
    else:
        print(f"    -> the lean is the {moved.upper()} number ({abs(delta):.1f} deg each way)")

    return angle, (1 if delta > 0 else -1), ok


def yaw_result(left_turn_deg: float, right_turn_deg: float | None = None):
    """The sign that makes a LEFT turn read positive, and whether to believe it.

    With both directions given, they must disagree in sign as well as being big
    enough — two turns reported the same way means yaw is not tracking at all.
    """
    ok = abs(left_turn_deg) >= MIN_YAW_DEG
    if right_turn_deg is not None:
        ok = ok and abs(right_turn_deg) >= MIN_YAW_DEG and (left_turn_deg > 0) != (right_turn_deg > 0)
    return (1 if left_turn_deg > 0 else -1), ok


def print_settings(pitch_offset: float, roll_offset: float,
                   pitch_sign: int, pitch_ok: bool,
                   roll_sign: int, roll_ok: bool,
                   yaw_sign: int, yaw_ok: bool) -> None:
    unknown = "UNKNOWN — see above"
    print("\n" + "=" * 78)
    print("SAVE THESE SETTINGS")
    print("=" * 78)
    print("Add these to settings.json (or tell Claude to put them in config.py):\n")
    print(f"  ORIO_IMU_PITCH_OFFSET_DEG = {pitch_offset:.2f}")
    print(f"  ORIO_IMU_ROLL_OFFSET_DEG  = {roll_offset:.2f}")
    print(f"  ORIO_IMU_PITCH_SIGN       = {f'{pitch_sign:+d}' if pitch_ok else unknown}")
    print(f"  ORIO_IMU_ROLL_SIGN        = {f'{roll_sign:+d}' if roll_ok else unknown}")
    print(f"  ORIO_IMU_YAW_SIGN         = {f'{yaw_sign:+d}' if yaw_ok else unknown}")
    if not (pitch_ok and roll_ok and yaw_ok):
        print("\nThe two offsets are good and can be saved now. The UNKNOWN lines need")
        print("their step doing again — this tool will not guess them.")
    print("\nCheck it worked: save them, run the tool again, and at the resting step")
    print("both numbers should read close to 0.00.")
