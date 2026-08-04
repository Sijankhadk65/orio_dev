"""Generate placeholder eye-expression data, one per FSM state.

Deliberately simple stand-ins so the eyes pipeline is testable end-to-end
before designer art exists, in the same JSON schema `orio.eyes._Expression`
reads — dropping real art in later is just overwriting these files.

    uv run python tools/make_placeholder_eyes.py        # writes assets/eyes/*.json

Canvas is 1024x600 to match the Elecrow RC070N 7" panel. Keep clips
loop-clean (first keyframe's values == last keyframe's) per
docs/eyes_animation_plan.md.
"""

from __future__ import annotations

import json
from pathlib import Path

FPS = 30
CX, CY = 512, 300
EYE_DX = 150  # horizontal offset of each eye from centre
EYE_W, EYE_H = 170, 220  # base eye size (rounded rect)
RADIUS = 60

CYAN = [51, 224, 255]
AMBER = [255, 189, 51]
RED = [255, 69, 69]
# Sleeping eyes: dimmer than awake, but still clearly visible as closed-eye
# lines (a sleeping face, not a blank screen).
DIM = [77, 179, 209]


def _track(color: list[int], keyframes: list[tuple[int, float, float, float, float]]) -> dict:
    """`keyframes` are (t, w, h, x, y) tuples."""
    return {
        "color": color,
        "radius": RADIUS,
        "rotation": 0,
        "keyframes": [
            {"t": t, "w": w, "h": h, "x": x, "y": y} for t, w, h, x, y in keyframes
        ],
    }


def clip(state: str) -> dict:
    """Assemble the expression data for one FSM state."""
    left, right = CX - EYE_DX, CX + EYE_DX

    if state == "idle":
        # Open eyes, calm blink every ~3 s.
        color = CYAN
        h = [EYE_H, EYE_H, EYE_H * 0.08, EYE_H, EYE_H]
        ts = [0, 66, 70, 74, 90]
        left_kf = [(t, EYE_W, hh, left, CY) for t, hh in zip(ts, h)]
        right_kf = [(t, EYE_W, hh, right, CY) for t, hh in zip(ts, h)]

    elif state == "listening":
        # Wide, attentive; a faster, lighter pulse (taller eyes).
        y = CY - 10
        h = [EYE_H * 1.15, EYE_H * 1.25, EYE_H * 1.15]
        ts = [0, 20, 40]
        left_kf = [(t, EYE_W, hh, left, y) for t, hh in zip(ts, h)]
        right_kf = [(t, EYE_W, hh, right, y) for t, hh in zip(ts, h)]
        color = CYAN

    elif state == "thinking":
        # Eyes glance up-and-away, slow pondering drift; squinted the whole time.
        h = EYE_H * 0.8
        ts = [0, 30, 60, 90]
        dx = [0, -40, 10, 0]
        dy = [0, -50, -40, 0]
        left_kf = [(t, EYE_W, h, left + ddx, CY + ddy) for t, ddx, ddy in zip(ts, dx, dy)]
        right_kf = [(t, EYE_W, h, right + ddx, CY + ddy) for t, ddx, ddy in zip(ts, dx, dy)]
        color = CYAN

    elif state == "speaking":
        # Rhythmic bob/squint pulse while audio plays.
        h = [EYE_H, EYE_H * 0.7, EYE_H, EYE_H]
        ts = [0, 8, 16, 24]
        left_kf = [(t, EYE_W, hh, left, CY) for t, hh in zip(ts, h)]
        right_kf = [(t, EYE_W, hh, right, CY) for t, hh in zip(ts, h)]
        color = CYAN

    elif state == "asleep":
        # Closed-eye lines with a clear, slow breathing motion so the panel
        # plainly reads as "sleeping face", not a blank screen.
        y = CY + 25
        h = [EYE_H * 0.24, EYE_H * 0.34, EYE_H * 0.24]
        ts = [0, 60, 120]
        left_kf = [(t, EYE_W, hh, left, y) for t, hh in zip(ts, h)]
        right_kf = [(t, EYE_W, hh, right, y) for t, hh in zip(ts, h)]
        color = DIM

    elif state == "error":
        # Flat dim-red eyes; brief by nature (recoverable).
        h = EYE_H * 0.2
        left_kf = [(0, EYE_W, h, left, CY), (30, EYE_W, h, left, CY)]
        right_kf = [(0, EYE_W, h, right, CY), (30, EYE_W, h, right, CY)]
        color = RED

    else:
        raise ValueError(f"unknown state {state!r}")

    return {
        "fps": FPS,
        "eyes": {"left": _track(color, left_kf), "right": _track(color, right_kf)},
    }


STATES = ["idle", "listening", "thinking", "speaking", "asleep", "error"]


def main() -> None:
    out_dir = Path(__file__).resolve().parent.parent / "assets" / "eyes"
    out_dir.mkdir(parents=True, exist_ok=True)
    for state in STATES:
        path = out_dir / f"{state}.json"
        path.write_text(json.dumps(clip(state)))
        print(f"wrote {path.relative_to(out_dir.parent.parent)}")


if __name__ == "__main__":
    main()
