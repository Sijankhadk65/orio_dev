# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=17", "python-dotenv>=1.0", "pillow>=10.1"]
# ///
"""A fake Orio for developing the app on a laptop — no robot, no Jetson stack.

    uv run tools/mock_server.py                  # ws://0.0.0.0:8765/ws
    uv run tools/mock_server.py --token secret   # check the app's token
    uv run tools/mock_server.py --stall-every 20 # freeze status 3 s in every 20
    uv run tools/mock_server.py --no-video       # don't serve the camera view

It serves through the robot's own `orio/server.py` and `orio/telemetry.py`, so
the handshake, version check, token check and message encoding are the real
ones; only the numbers are made up. The inline script metadata above keeps
`uv run` from installing the robot's dependencies (torch, the Orbbec SDK, ...),
which a laptop neither has nor needs.

The camera view is a moving test pattern, JPEG-encoded at 640 px wide like the
robot's, so a frozen or laggy stream is easy to see.

The Android emulator reaches this at ws://10.0.2.2:8765/ws; a phone on the same
Wi-Fi at ws://<laptop IP>:8765/ws. Keep it honest: a change to what the real
server sends changes this in the same commit.
"""

from __future__ import annotations

import argparse
import io
import logging
import random
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import telemetry  # noqa: E402
from orio.server import AppServer  # noqa: E402

DIALOGUE = [
    ("wake", "Hey Orio"),
    ("said", "Hmm?"),
    ("heard", "what can you see"),
    ("tool", "what_do_you_see"),
    ("said", "I can see a person and a chair."),
    ("heard", "go to the chair"),
    ("tool", "go_to"),
    ("said", "I walked over to the chair. I'm about a metre from it now."),
]

STATE_AFTER = {"wake": "LISTENING", "heard": "THINKING", "tool": "THINKING", "said": "SPEAKING"}


class FakeOrio:
    """Wandering numbers in the real status shape, and a looping conversation."""

    def __init__(self, stall_every: float) -> None:
        self.state = "ASLEEP"
        self.heading = 0.0
        self.sectors: list[float | None] = [1.5] * 7
        self.behaviour = "idle"
        self.detail = ""
        self._stall_every = stall_every
        self._started = time.monotonic()

    def status(self) -> dict:
        if self._stall_every:
            phase = (time.monotonic() - self._started) % self._stall_every
            if phase < 0.25:
                time.sleep(3.0)  # blocks the server loop: the app should show "No status"
        self.heading = (self.heading + random.uniform(-0.3, 0.3)) % 360
        for i, d in enumerate(self.sectors):
            if random.random() < 0.03:
                self.sectors[i] = None  # a dropout now and then; unknown reads blocked
            else:
                self.sectors[i] = min(3.0, max(0.12, (d or 1.5) + random.uniform(-0.12, 0.12)))
        return {
            "state": self.state,
            "behaviour": self.behaviour,
            "behaviour_detail": self.detail,
            "wheel_owner": None,
            "heading_deg": self.heading,
            "speed_percent": 5,
            "sensors": {
                "camera": {"ok": True, "detail": "Gemini 336L · 33 ms old (mock)"},
                "imu": {"ok": True, "detail": "BNO085 (mock)"},
                "drivetrain": {"ok": True, "detail": "STM32 (mock)"},
                "neck": {"ok": True, "detail": "holding pose (mock)"},
            },
            "sectors": {"fov_deg": 94.0, "distance_m": list(self.sectors),
                        "stop_m": 0.20, "clear_m": 0.70},
        }

    def frame(self, after: int) -> tuple[int, float, bytes]:
        """A test pattern that visibly moves, so a frozen stream is obvious."""
        from PIL import Image, ImageDraw, ImageFont

        w, h = 640, 400  # the Gemini's colour aspect
        img = Image.new("RGB", (w, h), (16, 20, 24))
        draw = ImageDraw.Draw(img)
        bars = [(192, 192, 192), (192, 192, 0), (0, 192, 192), (0, 192, 0),
                (192, 0, 192), (192, 0, 0), (0, 0, 192)]
        for i, colour in enumerate(bars):  # one bar per sector, left to right
            draw.rectangle([i * w // 7, 0, (i + 1) * w // 7, h // 2], fill=colour)
        t = time.time()
        x = int((t * 120) % w)
        draw.ellipse([x - 24, h * 3 // 4 - 24, x + 24, h * 3 // 4 + 24], fill=(31, 138, 138))
        font = ImageFont.load_default(size=22)
        stamp = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"
        draw.text((16, h // 2 + 12), f"orio (mock) · frame {after + 1} · {stamp}",
                  fill=(230, 230, 230), font=font)
        out = io.BytesIO()
        img.save(out, "JPEG", quality=60)
        return after + 1, t, out.getvalue()

    def converse(self, period: float) -> None:
        step = 0
        while True:
            time.sleep(period)
            kind, text = DIALOGUE[step % len(DIALOGUE)]
            step += 1
            self.state = STATE_AFTER[kind]
            if kind == "tool":
                args = {"target": "chair"} if text == "go_to" else {}
                result = ("went to the chair — standing 0.7 m away, facing them"
                          if text == "go_to" else "person (0.91), chair (0.78)")
                if text == "go_to":
                    self.behaviour, self.detail = "cruise", "forward"
                telemetry.tool(text, args, result)
            else:
                telemetry.hub.transcript(kind, text)
                self.behaviour, self.detail = "idle", ""
                if kind == "said" and step % len(DIALOGUE) == 0:
                    telemetry.event("arrived", "stopped about a metre from the chair")
            if step % len(DIALOGUE) == 0:
                self.state = "ASLEEP"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--path", default="/ws")
    parser.add_argument("--token", default="", help="require this token (default: accept any)")
    parser.add_argument("--talk-every", type=float, default=4.0, help="seconds between lines")
    parser.add_argument("--stall-every", type=float, default=0.0,
                        help="freeze status for 3 s once in this many seconds (0 = never)")
    parser.add_argument("--no-video", action="store_true", help="don't serve the camera view")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    orio = FakeOrio(args.stall_every)
    server = AppServer(orio.status, robot="orio (mock)", host=args.host, port=args.port,
                       path=args.path, token=args.token,
                       video=None if args.no_video else orio.frame)
    print(server.start())
    if not server.running:
        return 1
    threading.Thread(target=orio.converse, args=(args.talk_every,), daemon=True).start()
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
