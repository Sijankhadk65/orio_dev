#!/usr/bin/env python3
"""Live body keypoints and joint angles from the Gemini — the yoga assistant's eyes.

    uv run python tools/pose_debug.py                  # window: skeleton + angles
    uv run python tools/pose_debug.py --no-window      # terminal only, over ssh
    uv run python tools/pose_debug.py --image me.jpg   # a still photo, no camera
    uv run python tools/pose_debug.py --coach tree     # also run the yoga coach
    uv run python tools/pose_debug.py --coach tree --speak   # ...out loud

Before any coaching rule can say "bend your front knee more", three things have
to be true, and this is the tool for checking them:

1. **The whole body is in shot.** At the robot's camera height a standing adult
   needs to be roughly 2-3 m away. Anything cut off is listed in red at the top
   of the panel; a joint whose keypoints were not seen gets no angle at all.
2. **The keypoints sit on the joints.** Watch the skeleton through a few poses
   and in your real clothes and lighting. Loose sleeves and a dark room both
   move the wrists and ankles first.
3. **The angles mean what they say.** The panel lists each joint twice: `2D`,
   measured in the picture, and `3D`, measured on the depth-lifted points. They
   agree when the limb is flat to the camera and part ways when it points toward
   or away from it — that disagreement is foreshortening, and the 3-D figure is
   the one to believe. A `3D` of `--` means a keypoint had no usable depth
   (often the silhouette edge sampling the wall; see POSE_DEPTH_GATE_M).

On the picture, each joint is labelled with its 2-D angle — the one the yoga
coach judges by (see yoga.py for why). Blue limbs are the person's left, orange
their right — the right of the picture when they face the camera.

## --coach

Runs `yoga.Coach` on the live picture for one pose (mountain, warrior_ii, tree)
and shows what it would say: the state, which side leads, the hold timer, the
check it is working on, and the last cue. Cues are also printed to the terminal.
Add --speak to hear the cues in Orio's voice (ORIO_TTS, ElevenLabs by default),
so the coach can be tried from the mat rather than read off the screen. Every
sentence the pose can produce is synthesized at startup and cached under cues/,
so the first run for a voice takes a little while and later runs start at once.
Speech runs in the background: the picture never waits on it.

Keys: Q or Esc quits. S saves the raw frame and a JSON of keypoints and angles
to `captures/pose/` (gitignored) — the way to collect reference angles for the
pose library from a real person in a real pose.

Like the other debug tools it drives nothing. It opens the camera itself, so it
cannot run alongside main.py: the SDK will not open one device twice.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from orio import config
from orio.pose import JOINTS, KEYPOINTS, SKELETON, PoseEstimator, framing, joint_angles, lift
from orio.yoga import POSES, Coach

PANEL_W = 300
LEFT_BGR = (230, 160, 40)    # person's left
RIGHT_BGR = (40, 140, 245)   # person's right
CENTRE_BGR = (220, 220, 220)
ANGLE_3D_BGR = (80, 230, 80)
ANGLE_2D_BGR = (60, 220, 230)
WARN_BGR = (60, 60, 240)
TEXT_BGR = (230, 230, 230)
DIM_BGR = (140, 140, 140)

SNAPSHOT_DIR = config.ROOT / "captures" / "pose"


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


def draw_panel(height, subject, angles, problems, n_people, infer_ms, fps, has_depth):
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
    y += 8
    text(panel, "S snapshot   Q quit", (10, y), DIM_BGR)
    return panel


def wrap(s: str, width: int = 34) -> list[str]:
    lines, line = [], ""
    for word in s.split():
        if line and len(line) + 1 + len(word) > width:
            lines.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    return lines + ([line] if line else [])


def draw_coach(coach) -> np.ndarray:
    """The coach's state, as a strip to stack under the angle panel."""
    strip = np.full((150, PANEL_W, 3), 45, np.uint8)
    p = coach.pose
    text(strip, f"coach: {p.name}", (10, 20), TEXT_BGR)
    lead = f"  lead {coach.lead}" if p.lead and coach.state != "waiting" else ""
    colour = {"holding": ANGLE_3D_BGR, "done": ANGLE_3D_BGR, "adjusting": ANGLE_2D_BGR}.get(
        coach.state, DIM_BGR)
    text(strip, f"{coach.state}{lead}   hold {coach.held_s:4.1f}/{p.hold_s:.0f} s", (10, 42), colour)
    if coach.failing:
        text(strip, f"fixing: {coach.failing}", (10, 62), WARN_BGR)
    if coach.last_cue is not None:
        for k, line in enumerate(wrap(f'"{coach.last_cue.text}"')[:4]):
            text(strip, line, (10, 86 + 18 * k), TEXT_BGR)
    return strip


def snapshot(frame, subject, angles, problems) -> Path:
    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)
    stem = SNAPSHOT_DIR / time.strftime("%Y%m%d-%H%M%S")
    cv2.imwrite(str(stem.with_suffix(".png")), frame)
    def num(v):
        return None if v is None or not np.isfinite(v) else round(float(v), 4)

    data = {"framing": problems, "person": None}
    if subject is not None:
        data["person"] = {
            "distance_m": num(subject.distance_m()),
            "keypoints": {
                name: {
                    "xy": [num(v) for v in subject.xy[i]],
                    "conf": num(subject.conf[i]),
                    "xyz": None if subject.xyz is None else [num(v) for v in subject.xyz[i]],
                }
                for i, name in enumerate(KEYPOINTS)
            },
            "angles": {a.name: {"2d": num(a.deg_2d), "3d": num(a.deg_3d)} for a in angles},
        }
    stem.with_suffix(".json").write_text(json.dumps(data, indent=2))
    return stem.with_suffix(".png")


def analyse(estimator, frame, depth, intrinsics):
    """(people, subject, angles, problems, infer_ms) for one frame."""
    t0 = time.monotonic()
    people = estimator.estimate(frame)
    infer_ms = (time.monotonic() - t0) * 1000
    subject = people[0] if people else None
    angles, problems = [], []
    if subject is not None:
        if depth is not None:
            lift(subject, depth, intrinsics)
        angles = joint_angles(subject)
        h, w = frame.shape[:2]
        problems = framing(subject, w, h)
    return people, subject, angles, problems, infer_ms


def render(frame, people, subject, angles, problems, infer_ms, fps, has_depth, coach=None):
    view = frame.copy()  # the camera's buffer is shared; never draw on it
    for p in people:
        draw_person(view, p, angles if p is subject else [], p is subject)
    panel = draw_panel(view.shape[0], subject, angles, problems, len(people),
                       infer_ms, fps, has_depth)
    if coach is not None:
        panel = np.vstack([panel, draw_coach(coach)])
    if panel.shape[0] > view.shape[0]:
        pad = np.zeros((panel.shape[0] - view.shape[0], view.shape[1], 3), np.uint8)
        view = np.vstack([view, pad])
    return np.hstack([view, panel])


def report(subject, angles, problems, n_people, infer_ms, fps) -> str:
    rate = [f"{fps:4.1f} fps"] if fps else []
    if subject is None:
        return "  ".join(rate + ["no person in view"])
    d = subject.distance_m()
    parts = rate + [f"{infer_ms:3.0f} ms", f"people {n_people}",
                    f"at {'--' if d is None else f'{d:.2f} m'}"]
    parts += [f"{a.name}={fmt(a.deg_2d).strip()}/{fmt(a.deg_3d).strip()}" for a in angles]
    if problems:
        parts.append("[" + "; ".join(problems) + "]")
    return "  ".join(parts)


def run_image(estimator, path: Path, window: bool) -> int:
    frame = cv2.imread(str(path))
    if frame is None:
        print(f"could not read {path}")
        return 1
    people, subject, angles, problems, infer_ms = analyse(estimator, frame, None, None)
    print(report(subject, angles, problems, len(people), infer_ms, 0.0) + "   (2D/3D, no depth)")
    if window:
        cv2.imshow("pose", render(frame, people, subject, angles, problems, infer_ms, 0.0, False))
        while cv2.waitKey(50) not in (27, ord("q")):
            if cv2.getWindowProperty("pose", cv2.WND_PROP_VISIBLE) < 1:
                break
        cv2.destroyAllWindows()
    return 0


def run_camera(estimator, window: bool, coach=None, voice=None) -> int:
    from orio.gemini import GeminiCamera

    cam = GeminiCamera()
    try:
        cam.start()
        seq, fps, last_print, t_prev = 0, 0.0, 0.0, None
        while True:
            frames = cam.read(after=seq)
            seq = frames.seq
            now = time.monotonic()
            if t_prev is not None:
                inst = 1.0 / max(now - t_prev, 1e-6)
                fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
            t_prev = now

            people, subject, angles, problems, infer_ms = analyse(
                estimator, frames.color, frames.depth, cam.intrinsics)
            if coach is not None:
                h, w = frames.color.shape[:2]
                cue = coach.update(subject, w, h, now)
                if cue is not None:
                    print(f"coach [{cue.kind}] {cue.text}", flush=True)
                    if voice is not None:
                        voice.say(cue.text)

            if not window:
                if now - last_print >= 1.0:
                    print(report(subject, angles, problems, len(people), infer_ms, fps),
                          flush=True)
                    last_print = now
                continue

            cv2.imshow("pose", render(frames.color, people, subject, angles, problems,
                                      infer_ms, fps, True, coach))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                print(f"saved {snapshot(frames.color, subject, angles, problems)}")
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if voice is not None:
            voice.close()
        if window:
            cv2.destroyAllWindows()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--no-window", action="store_true", help="print once a second instead")
    parser.add_argument("--image", type=Path, help="run on a still image instead of the camera")
    parser.add_argument("--model", type=Path, default=config.POSE_MODEL_PATH,
                        help="pose checkpoint (.pt, or an exported .engine)")
    parser.add_argument("--coach", choices=sorted(POSES),
                        help="also run the yoga coach for this pose (camera only)")
    parser.add_argument("--speak", action="store_true",
                        help="speak the coach's cues (needs --coach)")
    parser.add_argument("--conf", type=float, default=config.POSE_CONFIDENCE,
                        help="minimum person-box confidence")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    estimator = PoseEstimator(model_path=args.model, confidence=args.conf)
    if args.image is not None:
        if args.coach:
            parser.error("--coach needs the live camera: it judges a pose held over time")
        return run_image(estimator, args.image, not args.no_window)
    if args.speak and not args.coach:
        parser.error("--speak speaks the coach's cues: pick a pose with --coach")
    coach = Coach(POSES[args.coach]) if args.coach else None
    voice = None
    if args.speak:
        from orio.cues import CoachVoice
        from orio.tts import get_tts
        from orio.yoga import phrases

        voice = CoachVoice(get_tts())
        intro = f"Let's do {coach.pose.name}. {coach.pose.how_to}"
        lines = [intro, *phrases(coach.pose)]
        print(f"preparing {len(lines)} spoken cues for {coach.pose.name} "
              "(first run for a voice synthesizes them; later runs load from cues/)...")
        voice.prewarm(lines)
        voice.say(intro)
    return run_camera(estimator, not args.no_window, coach, voice)


if __name__ == "__main__":
    sys.exit(main())
