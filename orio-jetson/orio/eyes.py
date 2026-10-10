"""Animated eyes and mouth — Orio's face on the HDMI panel, driven by the operator FSM.

The eyes are a *subscriber* to `fsm.StateMachine`, not their own state source:
the conversation loop never calls into here. It just transitions the FSM, and
`_Face` (a direct port of the reference `orio-eyes-standalone.html` canvas
engine) eases toward that state's procedural target geometry every frame.

Why procedural, and why ported 1:1 from the HTML reference rather than reusing
the reference's Lottie export: the shapes are pure math (two rounded blobs
whose width/height/position/skew/color are closed-form functions of time and
state, plus a blink/wink/mic-reactive/jitter layer, and a single-stroke mouth
whose width/opening/curve/tilt/zigzag morph the same way) — the reference itself
never bakes them into a fixed clip. An earlier attempt did bake them (into
Lottie, played back with `rlottie`) and hit real interpolation bugs: keyframed
Scale-type properties rendered correctly only on their first frame, and a
layer's Position broke the moment that layer also carried an animated Path.
Porting the reference's own formulas sidesteps that class of bug entirely and
reproduces its motion exactly instead of approximating it via sampled
keyframes.

Smoothness: all geometry is expressed in a 128x72 *unit* space (so every magic
number — gap between eyes, blob sizes, jitter — matches the reference), but is
drawn at full panel resolution: the face is filled at `_SUPERSAMPLE`x the
panel's size with per-row/per-column rect fills, then `smoothscale`d down, which
anti-aliases the edges into clean curves instead of visible pixel steps.

Concurrency: a background thread owns the pygame window and renders continuously
(~`fps`), because every operator call (`stt.listen`, `convo.send`, `tts.speak`)
blocks the main thread — drawing there would freeze the face during exactly the
moments it should be liveliest. The FSM listener (run on the loop's thread) only
flips a flag; the render thread picks up the new state on its next frame.
"""

from __future__ import annotations

import logging
import math
import os
import random
import threading
import time

import pygame

from .fsm import State, StateMachine

log = logging.getLogger(__name__)

# Unit space the face geometry is authored in — matches the reference's
# coordinate system, so every magic number in `_Face` (gap between eyes, blob
# sizes, jitter magnitude, ...) carries over unchanged. It's scaled uniformly to
# fit the panel and centred.
_UNIT_W, _UNIT_H = 128, 72

# Render at this multiple of the panel's resolution, then smoothscale down —
# that's the anti-aliasing. 2 is plenty at 1024x600.
_SUPERSAMPLE = 2

# Face styles — variations on the same geometry and motion, picked with
# ORIO_EYES_STYLE. `eye_border` is the ring's thickness in units (0 = none),
# drawn in `eye_border_color` (scaled by the state's dim, so ASLEEP stays dim).
_STYLES: dict[str, dict] = {
    "plain": {"eye_border": 0.0, "eye_border_color": (255, 255, 255)},
    "outlined": {"eye_border": 1.5, "eye_border_color": (255, 255, 255)},
}

_HUE: dict[str, tuple[int, int, int]] = {
    "ASLEEP": (0x2F, 0x7F, 0xB5),
    "IDLE": (0x4D, 0xDB, 0xFF),
    "LISTENING": (0x5C, 0xFF, 0xC0),
    "THINKING": (0xB5, 0x7B, 0xFF),
    "SPEAKING": (0xFF, 0xD2, 0x4D),
    "ERROR": (0xFF, 0x4D, 0x4D),
}
_BG = (7, 7, 11)


def _lerp(a: float, b: float, k: float) -> float:
    return a + (b - a) * k


def _hash01(n: float) -> float:
    """Deterministic pseudo-random in [0, 1), matching the reference's GLSL-style
    hash `Math.abs((Math.sin(n * 12.9898) * 43758.5453) % 1)` bit for bit —
    `math.fmod` (not `%`) to keep Python's sign convention the same as JS's."""
    x = math.sin(n * 12.9898) * 43758.5453
    return abs(math.fmod(x, 1.0))


class _Face:
    """Direct port of `OrioEyes` from orio-eyes-standalone.html: procedural
    per-state target geometry, exponentially eased toward every frame, drawn
    as two rounded blobs and a mouth stroke, scaled from unit space to the
    surface's real resolution. Owns its own clock so pausing the render thread (e.g. under a
    debugger) doesn't jump the animation."""

    def __init__(self, initial_state: str, style: str = "plain",
                 color: tuple[int, int, int] | None = None) -> None:
        self._t0 = time.monotonic()
        if style not in _STYLES:
            log.warning("eyes: unknown style %r — using 'plain' (have: %s)", style, ", ".join(_STYLES))
            style = "plain"
        self.style = _STYLES[style]
        # One fixed color for every state, or None to color by state (`_HUE`).
        self.fixed_color = color
        self.state = initial_state if initial_state in _HUE else "IDLE"
        self.enter_t = 0.0
        self.mic = 0.0
        self.next_blink = 2.4
        self.blink_start = -1.0
        self.wink_start = -1.0
        self.wink_eye = 0
        self.g = {"w": 26.0, "h": 30.0, "r": 9.0, "dx": 0.0, "dy": 0.0, "skew": 0.0, "dim": 1.0,
                  "mw": 14.0, "mopen": 0.0, "mcurve": 3.0, "mtilt": 0.0, "mzig": 0.0, "mdx": 0.0}
        self.col = list(color or _HUE[self.state])

    def now(self) -> float:
        return time.monotonic() - self._t0

    def set_state(self, name: str) -> None:
        if name not in _HUE or name == self.state:
            return
        t = self.now()
        self.state = name
        self.enter_t = t
        self.next_blink = t + 2.2

    # ── procedural target geometry — the whole animation vocabulary lives here ──
    def _sway(self, t: float) -> tuple[float, float]:
        """A gently drifting look-around offset: tweens between two random
        points every ~2.6s, seeded by the time bucket so it's smooth and
        endless without ever storing state between calls."""
        seed = math.floor(t / 2.6)
        k = min(1.0, math.fmod(t, 2.6) / 0.5)
        e = k * k * (3 - 2 * k)  # smoothstep
        ax, ay = _hash01(seed) * 6 - 3, _hash01(seed + 5) * 4 - 2
        bx, by = _hash01(seed + 1) * 6 - 3, _hash01(seed + 6) * 4 - 2
        return _lerp(ax, bx, e), _lerp(ay, by, e)

    def _targets(self, t: float) -> dict[str, float]:
        """Eye keys: w h r dx dy skew dim. Mouth keys (m*): `mw` width, `mopen`
        how far the lips part at the middle, `mcurve` smile (+) / frown (-)
        depth, `mtilt` right-side-up (+) slant, `mzig` zigzag amplitude, `mdx`
        sideways offset."""
        age = t - self.enter_t
        pop = math.exp(-age * 4.5) * math.sin(age * 15)  # settle overshoot on state entry
        st = self.state
        if st == "ASLEEP":
            return {"w": 28, "h": 5 + math.sin(t * 1.15) * 1.6, "r": 2.5,
                     "dx": 0, "dy": 7 + math.sin(t * 1.15) * 1.2, "skew": 0.06, "dim": 0.55,
                     "mw": 6, "mopen": 1.5 + math.sin(t * 1.15) * 1.2, "mcurve": 0,
                     "mtilt": 0, "mzig": 0, "mdx": 0}
        if st == "IDLE":
            sx, sy = self._sway(t)
            return {"w": 26, "h": 30 + pop * 5, "r": 9,
                     "dx": sx, "dy": sy + math.sin(t * 1.5) * 1.1, "skew": 0, "dim": 1,
                     "mw": 14, "mopen": 0, "mcurve": 3 + pop, "mtilt": 0, "mzig": 0, "mdx": 0}
        if st == "LISTENING":
            return {"w": 27 + self.mic * 1.5, "h": 34 + self.mic * 4 + pop * 5, "r": 10,
                     "dx": math.sin(t * 0.8) * 1.6, "dy": -1 - self.mic, "skew": -0.05, "dim": 1,
                     "mw": 7, "mopen": 4 + pop, "mcurve": 0, "mtilt": 0, "mzig": 0, "mdx": 0}
        if st == "THINKING":
            return {"w": 22, "h": 19, "r": 8,
                     "dx": -3 + math.cos(t * 1.5) * 2.2, "dy": -5 + math.sin(t * 1.5) * 1.4,
                     "skew": 0.3, "dim": 1,
                     "mw": 10, "mopen": 0, "mcurve": -0.5, "mtilt": 2 + math.sin(t * 1.5) * 0.5,
                     "mzig": 0, "mdx": 6}
        if st == "SPEAKING":
            env = math.sin(t * 8.5) * 0.6 + math.sin(t * 4.1) * 0.4
            return {"w": 26 - env * 1.2, "h": 26 + env * 6, "r": 9,
                     "dx": 0, "dy": -env * 1.6, "skew": -0.05, "dim": 1,
                     "mw": 13 - env * 1.5, "mopen": 3.5 + env * 3, "mcurve": 1,
                     "mtilt": 0, "mzig": 0, "mdx": 0}
        if st == "ERROR":
            return {"w": 28, "h": 11 + pop * 4, "r": 3, "dx": 0, "dy": 1, "skew": 0.55, "dim": 1,
                     "mw": 18, "mopen": 0, "mcurve": -1, "mtilt": 0, "mzig": 1.5, "mdx": 0}
        return {"w": 26, "h": 30, "r": 9, "dx": 0, "dy": 0, "skew": 0, "dim": 1,
                 "mw": 14, "mopen": 0, "mcurve": 3, "mtilt": 0, "mzig": 0, "mdx": 0}

    # ── drawing ──────────────────────────────────────────────────────────────
    @staticmethod
    def _blob(surface: pygame.Surface, cx: float, cy: float, w: float, h: float,
              r: float, skew: float, color: tuple[int, int, int]) -> None:
        """A rounded rect with per-row horizontal slant, in surface pixels: fill
        one 1px-tall rect per row, insetting near the top/bottom for the corner
        radius and offsetting sideways for `skew` (a per-row ratio, so it's the
        same at any scale). Edges are hard here; the supersample + smoothscale
        in the render loop anti-aliases them."""
        w, h = max(2, round(w)), max(1, round(h))
        r = max(0.0, min(r, h / 2, w / 2))
        x0, y0 = round(cx - w / 2), round(cy - h / 2)
        for row in range(h):
            d = min(row, h - 1 - row)
            inset = 0
            if d < r:
                inset = round(r - math.sqrt(max(0.0, r * r - (r - d - 0.5) ** 2)))
            off = round(skew * (row - (h - 1) / 2))
            rw = w - inset * 2
            if rw > 0:
                surface.fill(color, (x0 + inset + off, y0 + row, rw, 1))

    @staticmethod
    def _mouth(surface: pygame.Surface, s: float, cx: float, cy: float, w: float, open_: float,
               curve: float, tilt: float, zig: float, color: tuple[int, int, int]) -> None:
        """The mouth: a 2-unit stroke bent by `curve` / `tilt` / `zig` and
        parted into a lens by `open_` (widest at the middle, closed at the
        corners). `cx`/`cy` are surface pixels, the shape args are units, `s` is
        pixels per unit. Filled one 1px column at a time; each column also
        spans back to the previous column's centre, so steep slopes stay one
        unbroken stroke. The stroke thins along a circle over the last unit at
        each corner, so the ends come out round."""
        w = max(2, round(w * s))
        x0 = round(cx - w / 2)
        prev = None
        for col in range(w):
            u = (col + 0.5) / w * 2 - 1  # -1 left corner … +1 right corner
            tri = abs(((col / s) % 6) / 6 * 4 - 2) - 1
            yc = cy + (curve * (0.5 - u * u) - tilt * u + zig * tri) * s
            half = max(0.0, open_) / 2 * math.sqrt(max(0.0, 1 - u * u)) * s
            d = min(col, w - 1 - col) + 0.5  # px from the nearest corner
            stroke = math.sqrt(max(0.0, s * s - (s - d) ** 2)) if d < s else s
            lo, hi = (yc, yc) if prev is None else (min(yc, prev), max(yc, prev))
            top, bot = round(lo - stroke - half), round(hi + stroke + half)
            surface.fill(color, (x0 + col, top, 1, max(1, bot - top)))
            prev = yc

    def draw(self, surface: pygame.Surface) -> None:
        t = self.now()
        w, h = surface.get_size()
        tg = self._targets(t)
        g = self.g
        for key in g:
            g[key] = _lerp(g[key], tg[key], 0.16)
        mic_target = abs(math.sin(t * 3.1) * 0.6 + math.sin(t * 7.9) * 0.4) if self.state == "LISTENING" else 0.0
        self.mic = _lerp(self.mic, mic_target, 0.18)

        target_color = self.fixed_color or _HUE[self.state]
        for i in range(3):
            self.col[i] = _lerp(self.col[i], target_color[i], 0.1)

        # Blinks (idle family only) + an occasional post-blink wink while idle.
        if self.state in ("IDLE", "LISTENING", "SPEAKING") and t > self.next_blink:
            self.blink_start = t
            self.next_blink = t + 2.6 + random.random() * 3.6
        blink = 1.0
        if self.blink_start > 0:
            b = (t - self.blink_start) / 0.13
            if b < 2:
                blink = max(0.04, abs(b - 1))
            else:
                self.blink_start = -1.0
                if self.state == "IDLE" and random.random() < 0.28:
                    self.wink_start = t
                    self.wink_eye = 0 if random.random() < 0.5 else 1
        wink, wink_eye = 1.0, -1
        if self.wink_start > 0:
            b = (t - self.wink_start) / 0.2
            if b < 2:
                wink, wink_eye = max(0.05, abs(b - 1)), self.wink_eye
            else:
                self.wink_start = -1.0

        surface.fill(_BG)
        # Unit space → surface pixels: uniform scale to fit, centred.
        s = min(w / _UNIT_W, h / _UNIT_H)

        def px(ux: float, uy: float) -> tuple[float, float]:
            return w / 2 + ux * s, h / 2 + uy * s

        # Eyes sit a little above centre to leave room for the mouth below.
        eye_y, gap_half = -5, 20
        jitter = round(random.random() * 3 - 1.5) if self.state == "ERROR" and random.random() < 0.22 else 0
        color = tuple(round(c * g["dim"]) for c in self.col)
        border = self.style["eye_border"]
        border_color = tuple(round(c * g["dim"]) for c in self.style["eye_border_color"])

        for i in range(2):
            lid = blink * (wink if i == wink_eye else 1)
            eye_h = max(2.0, g["h"] * lid)
            cx, cy = px((-gap_half if i == 0 else gap_half) + g["dx"] + jitter, eye_y + g["dy"])
            skew = g["skew"] * (1 if i == 0 else -1)
            if border > 0:
                # A ring: the same blob grown by `border` on every side, then the
                # eye drawn on top of it.
                self._blob(surface, cx, cy, (g["w"] + 2 * border) * s, (eye_h + 2 * border) * s,
                           (g["r"] + border) * s, skew, border_color)
            self._blob(surface, cx, cy, g["w"] * s, eye_h * s, g["r"] * s, skew, color)

        # Mouth follows the eyes at reduced parallax, so the face moves as one
        # but reads as having depth.
        mx, my = px(g["dx"] * 0.6 + g["mdx"] + jitter, 21 + g["dy"] * 0.4)
        self._mouth(surface, s, mx, my, g["mw"], g["mopen"], g["mcurve"], g["mtilt"], g["mzig"], color)


class EyesController:
    """Owns the face window + render thread; reacts to FSM transitions."""

    def __init__(
        self,
        fsm: StateMachine,
        *,
        size: tuple[int, int] = (1024, 600),
        fullscreen: bool = True,
        fps: int = 30,
        debug: bool = False,
        style: str = "plain",
        color: tuple[int, int, int] | None = None,
    ) -> None:
        self._fsm = fsm
        self._size = size
        self._fullscreen = fullscreen
        self._fps = fps
        self._debug = debug
        self._style = style
        self._color = color

        self._lock = threading.Lock()
        self._pending = fsm.state  # latest state the render thread should show
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._unsubscribe = None

    # ── lifecycle ────────────────────────────────────────────────────────────
    def start(self) -> None:
        """Subscribe to the FSM and spin up the render thread."""
        self._unsubscribe = self._fsm.subscribe(self._on_transition)
        self._thread = threading.Thread(
            target=self._run, name="eyes", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Tear down the subscription, render thread, and window."""
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ── FSM hook (runs on the loop thread — keep it cheap) ───────────────────
    def _on_transition(self, _old: State, new: State) -> None:
        with self._lock:
            self._pending = new

    def _take_pending(self) -> State:
        with self._lock:
            return self._pending

    def _draw_overlay(
        self, screen: pygame.Surface, font: pygame.font.Font, state: State, fps: float
    ) -> None:
        """Draw the current FSM state + live render FPS (debug only)."""
        text = f"{state.name}  {fps:4.1f} fps"
        label = font.render(text, True, (0, 255, 0))
        # Drop shadow for legibility over the animated face.
        shadow = font.render(text, True, (0, 0, 0))
        screen.blit(shadow, (9, 9))
        screen.blit(label, (8, 8))

    def _run(self) -> None:
        try:
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            pygame.init()
            pygame.mouse.set_visible(False)
            flags = pygame.FULLSCREEN | pygame.SCALED if self._fullscreen else 0
            screen = pygame.display.set_mode(self._size, flags)
            pygame.display.set_caption("Orio")
            overlay_font = pygame.font.Font(None, 28) if self._debug else None
        except Exception:
            log.exception("eyes: could not open display — face disabled")
            return

        hires = pygame.Surface((self._size[0] * _SUPERSAMPLE, self._size[1] * _SUPERSAMPLE))
        face = _Face(self._take_pending().name, self._style, self._color)
        clock = pygame.time.Clock()

        while not self._stop.is_set():
            # Draining the event queue keeps the window responsive (and lets a
            # window-manager close request stop us cleanly).
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop.set()

            face.set_state(self._take_pending().name)
            face.draw(hires)

            # Averaging the supersampled frame down is the anti-aliasing.
            pygame.transform.smoothscale(hires, self._size, screen)

            if overlay_font is not None:
                self._draw_overlay(screen, overlay_font, self._take_pending(), clock.get_fps())

            pygame.display.flip()
            clock.tick(self._fps)

        pygame.quit()
