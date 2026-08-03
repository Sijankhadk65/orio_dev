"""Cycle the eyes through every FSM state so you can eyeball the look.

    ORIO_EYES_FULLSCREEN=0 uv run python tools/eyes_demo.py   # windowed (NoMachine)
    uv run python tools/eyes_demo.py                          # fullscreen panel

Drives a real `StateMachine` through a representative sequence on a timer; the
`EyesController` reacts exactly as it would to the live conversation loop. Ctrl-C
to quit.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # repo root → import orio

from orio import config
from orio.eyes import EyesController
from orio.fsm import State, StateMachine

# A plausible conversation arc, plus ERROR at the end.
SEQUENCE = [
    (State.IDLE, 3.0),
    (State.LISTENING, 3.0),
    (State.THINKING, 3.0),
    (State.SPEAKING, 3.0),
    (State.ASLEEP, 3.0),
    (State.ERROR, 2.0),
]


def main() -> None:
    fsm = StateMachine(initial=State.IDLE)
    eyes = EyesController(
        fsm,
        clips_dir=config.EYES_CLIPS_DIR,
        size=config.EYES_SIZE,
        fullscreen=config.EYES_FULLSCREEN,
        fps=config.EYES_FPS,
        debug=True,  # this is the eyeball-the-look tool; always show the overlay
        transition_ms=config.EYES_TRANSITION_MS,
    )
    eyes.start()
    print("Eyes demo — Ctrl-C to quit. Cycling states:")
    try:
        while True:
            for state, hold in SEQUENCE:
                print(f"  → {state}")
                fsm.to(state, force=True)
                time.sleep(hold)
    except KeyboardInterrupt:
        pass
    finally:
        eyes.stop()
        print("\nbye")


if __name__ == "__main__":
    main()
