"""Generate placeholder eye-expression clips (Lottie JSON), one per FSM state.

These are deliberately simple stand-ins so the eyes pipeline is testable
end-to-end *now*, before designer art exists. Each clip is a pre-baked Lottie
file (vector shapes, CPU-rasterized by rlottie at runtime) keyed by the
`orio.fsm.State` it represents — exactly the format the final designer clips
will use, so dropping real art in later is just overwriting these files.

    uv run python tools/make_placeholder_eyes.py        # writes assets/eyes/*.json

Canvas is 1024x600 to match the Elecrow RC070N 7" panel. Keep clips vector and
loop-clean (first frame == last frame) per docs/eyes_animation_plan.md.
"""

from __future__ import annotations

import json
from pathlib import Path

W, H = 1024, 600
FPS = 30
CY = H / 2
EYE_DX = 150  # horizontal offset of each eye from centre
EYE_W, EYE_H = 170, 220  # base eye size (rounded rect)

CYAN = [0.20, 0.88, 1.0, 1.0]
AMBER = [1.0, 0.74, 0.20, 1.0]
RED = [1.0, 0.27, 0.27, 1.0]
# Sleeping eyes: dimmer than awake, but still clearly visible as closed-eye
# lines (a sleeping face, not a blank screen).
DIM = [0.30, 0.70, 0.82, 1.0]


def _bg() -> dict:
    """Full-frame black backdrop so the panel reads as a dark face."""
    return {
        "ty": 4, "nm": "bg", "ind": 99, "ip": 0, "op": 100000, "st": 0,
        "ks": {
            "o": {"a": 0, "k": 100}, "r": {"a": 0, "k": 0},
            "p": {"a": 0, "k": [W / 2, H / 2]}, "a": {"a": 0, "k": [0, 0]},
            "s": {"a": 0, "k": [100, 100]},
        },
        "shapes": [
            {"ty": "rc", "p": {"a": 0, "k": [0, 0]},
             "s": {"a": 0, "k": [W, H]}, "r": {"a": 0, "k": 0}},
            {"ty": "fl", "c": {"a": 0, "k": [0.04, 0.04, 0.05]},
             "o": {"a": 0, "k": 100}},
        ],
    }


def _eye(ind: int, cx: float, color: list, scale_y, pos_y, op: int) -> dict:
    """One eye as a rounded rect. `scale_y`/`pos_y` are static values or
    Lottie keyframe lists (a=1) for blink/look animation."""
    sy_anim = isinstance(scale_y, list)
    py_anim = isinstance(pos_y, list)
    return {
        "ty": 4, "nm": f"eye{ind}", "ind": ind, "ip": 0, "op": op, "st": 0,
        "ks": {
            "o": {"a": 0, "k": 100}, "r": {"a": 0, "k": 0},
            "p": ({"a": 1, "k": pos_y} if py_anim
                  else {"a": 0, "k": [cx, pos_y]}),
            "a": {"a": 0, "k": [0, 0]},
            "s": ({"a": 1, "k": scale_y} if sy_anim
                  else {"a": 0, "k": [100, scale_y]}),
        },
        "shapes": [
            {"ty": "rc", "p": {"a": 0, "k": [0, 0]},
             "s": {"a": 0, "k": [EYE_W, EYE_H]}, "r": {"a": 0, "k": 60}},
            {"ty": "fl", "c": {"a": 0, "k": color[:3]}, "o": {"a": 0, "k": 100}},
        ],
    }


def _kf(frames: list[tuple[int, list]]) -> list:
    """Build a keyframe list with smooth easing between the given (t, value)s."""
    out = []
    for i, (t, val) in enumerate(frames):
        kf = {"t": t, "s": val}
        if i < len(frames) - 1:
            kf["i"] = {"x": [0.6], "y": [1.0]}
            kf["o"] = {"x": [0.4], "y": [0.0]}
        out.append(kf)
    return out


def _scale_full(cy_height_pct: float) -> list:
    return [100, cy_height_pct]


def clip(state: str) -> dict:
    """Assemble a full Lottie document for one FSM state."""
    left, right = W / 2 - EYE_DX, W / 2 + EYE_DX

    if state == "idle":
        # Open eyes, calm blink every ~3 s.
        op = 90
        blink = _kf([(0, _scale_full(100)), (66, _scale_full(100)),
                     (70, _scale_full(8)), (74, _scale_full(100)),
                     (90, _scale_full(100))])
        color = CYAN
        eyes = [_eye(1, left, color, blink, CY, op),
                _eye(2, right, color, blink, CY, op)]

    elif state == "listening":
        # Wide, attentive; a faster, lighter pulse (taller eyes).
        op = 40
        pulse = _kf([(0, _scale_full(115)), (20, _scale_full(125)),
                     (40, _scale_full(115))])
        eyes = [_eye(1, left, CYAN, pulse, CY - 10, op),
                _eye(2, right, CYAN, pulse, CY - 10, op)]

    elif state == "thinking":
        # Eyes glance up-and-away, slow pondering drift.
        op = 90
        look = _kf([(0, [left, CY]), (30, [left - 40, CY - 50]),
                    (60, [left + 10, CY - 40]), (90, [left, CY])])
        look_r = _kf([(0, [right, CY]), (30, [right - 40, CY - 50]),
                      (60, [right + 10, CY - 40]), (90, [right, CY])])
        squint = _scale_full(80)
        eyes = [_eye(1, left, CYAN, squint, look, op),
                _eye(2, right, CYAN, squint, look_r, op)]

    elif state == "speaking":
        # Rhythmic bob/squint pulse while audio plays.
        op = 24
        bob = _kf([(0, _scale_full(100)), (8, _scale_full(70)),
                   (16, _scale_full(100)), (24, _scale_full(100))])
        eyes = [_eye(1, left, CYAN, bob, CY, op),
                _eye(2, right, CYAN, bob, CY, op)]

    elif state == "asleep":
        # Closed-eye lines with a clear, slow breathing motion so the panel
        # plainly reads as "sleeping face", not a blank screen.
        op = 120
        breathe = _kf([(0, _scale_full(24)), (60, _scale_full(34)),
                       (120, _scale_full(24))])
        eyes = [_eye(1, left, DIM, breathe, CY + 25, op),
                _eye(2, right, DIM, breathe, CY + 25, op)]

    elif state == "error":
        # Flat dim-red eyes; brief by nature (recoverable).
        op = 30
        flat = _scale_full(20)
        eyes = [_eye(1, left, RED, flat, CY, op),
                _eye(2, right, RED, flat, CY, op)]

    else:
        raise ValueError(f"unknown state {state!r}")

    return {
        "v": "5.7.0", "fr": FPS, "ip": 0, "op": op,
        "w": W, "h": H, "nm": f"orio_eyes_{state}", "ddd": 0,
        "assets": [], "layers": [*eyes, _bg()],
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
