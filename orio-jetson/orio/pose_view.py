"""Drawing the pose model's view: skeleton, joint angles, and the yoga session.

Shared by the bench tool (tools/pose_debug.py) and the robot's own debug window
(`YogaDebugWindow`, opened with ORIO_YOGA_DEBUG=1), so both show the same
picture. A dev aid only: nothing here feeds back into what Orio does.
"""

from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np

from .pose import JOINTS, KEYPOINTS, SKELETON

log = logging.getLogger(__name__)

PANEL_W = 300
LEFT_BGR = (230, 160, 40)    # person's left
RIGHT_BGR = (40, 140, 245)   # person's right
CENTRE_BGR = (220, 220, 220)
ANGLE_3D_BGR = (80, 230, 80)
ANGLE_2D_BGR = (60, 220, 230)
WARN_BGR = (60, 60, 240)
TEXT_BGR = (230, 230, 230)
DIM_BGR = (140, 140, 140)
GOOD_BGR = (80, 230, 80)


def side_colour(i: int, j: int | None = None):
    names = [KEYPOINTS[i]] + ([KEYPOINTS[j]] if j is not None else [])
    if all(n.startswith("left_") for n in names):
        return LEFT_BGR
    if all(n.startswith("right_") for n in names):
        return RIGHT_BGR
    return CENTRE_BGR


def text(img, s, org, colour=TEXT_BGR, scale=0.45, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv2.LINE_AA)


def fmt(deg) -> str:
    return "  --" if deg is None else f"{deg:4.0f}"


def wrap(s: str, width: int = 34) -> list[str]:
    lines, line = [], ""
    for word in s.split():
        if line and len(line) + 1 + len(word) > width:
            lines.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    return lines + ([line] if line else [])


def draw_person(img, person, angles, subject: bool) -> None:
    thick = 2 if subject else 1
    for i, j in SKELETON:
        if person.seen(i) and person.seen(j):
            p, q = person.xy[i], person.xy[j]
            cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])),
                     side_colour(i, j), thick, cv2.LINE_AA)
    for i, (u, v) in enumerate(person.xy):
        if person.seen(i):
            cv2.circle(img, (int(u), int(v)), 3 if subject else 2, side_colour(i), -1, cv2.LINE_AA)
    if not subject:
        return
    for a in angles:
        if a.deg_2d is not None:
            text(img, f"{a.deg_2d:.0f}", (int(a.vertex[0]) + 6, int(a.vertex[1]) - 6),
                 ANGLE_2D_BGR, 0.5, 1)


def draw_panel(height, subject, angles, problems, n_people, infer_ms, fps, has_depth,
               footer: str = ""):
    panel = np.full((max(height, 360), PANEL_W, 3), 30, np.uint8)
    y = 22
    rate = f"{fps:4.1f} fps   " if fps else ""
    text(panel, f"{rate}infer {infer_ms:4.0f} ms", (10, y))
    y += 20
    text(panel, f"people: {n_people}", (10, y), DIM_BGR)
    if subject is not None and has_depth:
        d = subject.distance_m()
        text(panel, f"distance: {'--' if d is None else f'{d:.2f} m'}", (120, y), DIM_BGR)
    y += 24
    if subject is None:
        text(panel, "no person in view", (10, y), WARN_BGR)
        return panel
    for p in problems:
        text(panel, p, (10, y), WARN_BGR)
        y += 18
    y += 6
    text(panel, "joint            2D    3D", (10, y), DIM_BGR)
    y += 20
    by_name = {a.name: a for a in angles}
    for name in JOINTS:
        a = by_name.get(name)
        label = name.replace("_", " ")
        if a is None:
            text(panel, f"{label:<15}   not seen", (10, y), DIM_BGR)
        else:
            colour = ANGLE_3D_BGR if a.deg_3d is not None else ANGLE_2D_BGR
            text(panel, f"{label:<15} {fmt(a.deg_2d)}  {fmt(a.deg_3d)}", (10, y), colour)
        y += 20
    if footer:
        y += 8
        text(panel, footer, (10, y), DIM_BGR)
    return panel


def _strip(lines) -> np.ndarray:
    """[(text, colour)] -> a panel-wide strip, wrapped, sized to fit."""
    rows = []
    for s, colour in lines:
        rows += [(w, colour) for w in (wrap(s) or [""])]
    strip = np.full((16 + 18 * len(rows), PANEL_W, 3), 45, np.uint8)
    for k, (s, colour) in enumerate(rows):
        text(strip, s, (10, 20 + 18 * k), colour)
    return strip


def draw_coach(coach) -> np.ndarray:
    """A bench coach's state (pose_debug --coach), as a strip under the panel."""
    p = coach.pose
    lead = f"  lead {coach.lead}" if p.lead and coach.state != "waiting" else ""
    colour = {"holding": GOOD_BGR, "done": GOOD_BGR, "adjusting": ANGLE_2D_BGR}.get(
        coach.state, DIM_BGR)
    lines = [(f"coach: {p.name}", TEXT_BGR),
             (f"{coach.state}{lead}   hold {coach.held_s:4.1f}/{p.hold_s:.0f} s", colour)]
    if coach.failing:
        lines.append((f"fixing: {coach.failing}", WARN_BGR))
    if coach.last_cue is not None:
        lines.append((f'"{coach.last_cue.text}"', TEXT_BGR))
    return _strip(lines)


def draw_session(view) -> np.ndarray:
    """A live session's state (coaching.View), as a strip under the panel."""
    framer, coach = view.framer, view.coach
    lines = [(f"yoga: {view.pose_name}   phase: {view.phase}", TEXT_BGR)]
    if view.phase == "framing":
        if framer.issues:
            lines += [(f"- {w}", WARN_BGR) for _, w in framer.issues]
        else:
            lines.append(("in shot - settling", GOOD_BGR))
    else:
        lead = f"  lead {coach.lead}" if view.coach.pose.lead and coach.state != "waiting" else ""
        colour = {"holding": GOOD_BGR, "done": GOOD_BGR, "adjusting": ANGLE_2D_BGR}.get(
            coach.state, DIM_BGR)
        lines.append((f"{coach.state}{lead}   hold {coach.held_s:4.1f}/"
                      f"{coach.pose.hold_s:.0f} s", colour))
        if coach.failing:
            lines.append((f"fixing: {coach.failing}", WARN_BGR))
    if view.said:
        lines.append(("to the LLM:", DIM_BGR))
        lines.append((view.said, TEXT_BGR))
    return _strip(lines)


def render(frame, people, subject, angles, problems, infer_ms, fps, has_depth,
           strip=None, footer: str = ""):
    """Picture with skeletons, the angle panel beside it, `strip` under that."""
    view = frame.copy()  # the camera's buffer is shared; never draw on it
    for p in people:
        draw_person(view, p, angles if p is subject else [], p is subject)
    panel = draw_panel(view.shape[0], subject, angles, problems, len(people),
                       infer_ms, fps, has_depth, footer)
    if strip is not None:
        panel = np.vstack([panel, strip])
    if panel.shape[0] > view.shape[0]:
        pad = np.zeros((panel.shape[0] - view.shape[0], view.shape[1], 3), np.uint8)
        view = np.vstack([view, pad])
    return np.hstack([view, panel])


class YogaDebugWindow:
    """The yoga debug view on the robot, in its own window and thread.

    During a session it draws what the session itself saw — the same frame, the
    same people, the framing problems and the coach's state — so nothing is
    computed twice. With no session it runs the pose model on the camera itself
    at a lower rate, which is the way to check framing before starting one.
    """

    _IDLE_FPS = 5

    def __init__(self, fps: int = 15) -> None:
        self._period = 1.0 / max(1, fps)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._seq = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="yoga-debug", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _idle(self, fps: float):
        """No session: one frame through the pose model, drawn."""
        from . import coaching
        from .gemini import shared
        from .pose import framing, joint_angles, lift

        camera = shared()
        frames = camera.read(after=self._seq, timeout=1.0)
        self._seq = frames.seq
        est, lock = coaching.estimator()
        t0 = time.monotonic()
        with lock:
            people = est.estimate(frames.color)
        infer_ms = (time.monotonic() - t0) * 1000
        subject = people[0] if people else None
        angles, problems = [], []
        if subject is not None:
            lift(subject, frames.depth, camera.intrinsics)
            angles = joint_angles(subject)
            h, w = frames.color.shape[:2]
            problems = framing(subject, w, h)
        strip = _strip([("no yoga session", DIM_BGR)])
        return render(frames.color, people, subject, angles, problems, infer_ms, fps, True,
                      strip)

    def _run(self) -> None:
        from . import coaching

        window = "orio yoga"
        fps, last = 0.0, None
        while not self._stop.is_set():
            start = time.monotonic()
            try:
                session = coaching.current()
                view = session.view if session is not None and session.running else None
                if view is not None and start - view.t < 1.0:
                    img = render(view.frame, view.people, view.subject, view.angles, [],
                                 view.infer_ms, fps, True, draw_session(view))
                    period = self._period
                else:
                    img = self._idle(fps)
                    period = 1.0 / self._IDLE_FPS
            except Exception as exc:  # noqa: BLE001 - a debug view must never stop Orio
                log.warning("yoga debug view: %s", exc)
                if self._stop.wait(1.0):
                    break
                continue
            if last is not None:
                inst = 1.0 / max(start - last, 1e-6)
                fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
            last = start
            cv2.imshow(window, img)
            wait_ms = max(1, round((period - (time.monotonic() - start)) * 1000))
            cv2.waitKey(wait_ms)
        cv2.destroyWindow(window)
