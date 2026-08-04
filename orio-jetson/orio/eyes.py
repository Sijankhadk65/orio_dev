"""Animated eyes — Orio's face on the HDMI panel, driven by the operator FSM.

The eyes are a *subscriber* to `fsm.StateMachine`, not their own state source:
the conversation loop never calls into here. It just transitions the FSM, and
`_Face` (a direct port of the reference `orio-eyes-standalone.html` canvas
engine) eases toward that state's procedural target geometry every frame.

Why procedural, and why ported 1:1 from the HTML reference rather than reusing
the reference's Lottie export: the shapes are pure math (two rounded blobs
whose width/height/position/skew/color are closed-form functions of time and
state, plus a blink/wink/mic-reactive/glitch layer) — the reference itself
never bakes them into a fixed clip. An earlier attempt did bake them (into
Lottie, played back with `rlottie`) and hit real interpolation bugs: keyframed
Scale-type properties rendered correctly only on their first frame, and a
layer's Position broke the moment that layer also carried an animated Path.
Porting the reference's own formulas sidesteps that class of bug entirely and
reproduces its motion exactly instead of approximating it via sampled
keyframes.

Sharpness: the reference draws on a tiny native `<canvas>` (128x72) with hard
(non-anti-aliased) per-row rect fills, then blows it up with CSS
`image-rendering: pixelated`. We do the same — draw at (128, 72) with plain
`Surface.fill()` per row, then `pygame.transform.scale` (nearest-neighbor, no
smoothing) up to the panel's real resolution. Drawing anti-aliased at full res
and *then* downsampling (an earlier version of this file did that) softens the
edges into a blur instead of the reference's crisp blocky look.

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

# Native draw resolution — matches orio-eyes-standalone.html's <canvas> exactly,
# so every magic number in `_Face` (gap between eyes, blob sizes, specular pixel
# offsets, jitter magnitude, ...) carries over unchanged.
_NATIVE_W, _NATIVE_H = 128, 72

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


# CRT scanline overlay: matches the reference's `repeating-linear-gradient(to
# bottom, rgba(0,0,0,.30) 0px, rgba(0,0,0,.30) 2px, transparent 2px, transparent
# 6px)` drifting overlay div (`scanDrift`, 1.6s linear infinite). Drawn over the
# upscaled face at display resolution — not the tiny native canvas — so the
# bands stay crisp 2px lines regardless of `_size`.
_SCANLINE_PERIOD = 6
_SCANLINE_BAND = 2
_SCANLINE_ALPHA = round(0.30 * 255)
_SCANLINE_DRIFT_S = 1.6  # seconds per full period of downward drift


def _build_scanlines(size: tuple[int, int]) -> pygame.Surface:
    """Precompute one tileable drift-cycle's worth of scanline texture, tall
    enough that shifting it down by up to one period never exposes a gap at
    the top (the second blit in the render loop covers that by re-wrapping)."""
    w, h = size
    tile_h = h + _SCANLINE_PERIOD
    surf = pygame.Surface((w, tile_h), pygame.SRCALPHA)
    dark = (0, 0, 0, _SCANLINE_ALPHA)
    for y in range(0, tile_h, _SCANLINE_PERIOD):
        surf.fill(dark, (0, y, w, _SCANLINE_BAND))
    return surf


def _draw_scanlines(screen: pygame.Surface, scanlines: pygame.Surface, t: float) -> None:
    drift = round((t % _SCANLINE_DRIFT_S) / _SCANLINE_DRIFT_S * _SCANLINE_PERIOD)
    screen.blit(scanlines, (0, drift))
    screen.blit(scanlines, (0, drift - _SCANLINE_PERIOD))


# Vertical RGB fringe overlay: matches the reference's other repeating-linear-
# gradient div (`to right`, three 2px bands per 6px period, static — no
# animation on this one in the reference either).
_FRINGE_ALPHA = round(0.03 * 255)
_FRINGE_COLORS = (
    (255, 0, 80, _FRINGE_ALPHA),
    (0, 255, 160, _FRINGE_ALPHA),
    (60, 120, 255, _FRINGE_ALPHA),
)


def _build_fringe(size: tuple[int, int]) -> pygame.Surface:
    w, h = size
    surf = pygame.Surface(size, pygame.SRCALPHA)
    for x in range(0, w, _SCANLINE_PERIOD):
        for i, color in enumerate(_FRINGE_COLORS):
            surf.fill(color, (x + i * _SCANLINE_BAND, 0, _SCANLINE_BAND, h))
    return surf


class _Face:
    """Direct port of `OrioEyes` from orio-eyes-standalone.html: procedural
    per-state target geometry, exponentially eased toward every frame, drawn
    as two hard-edged rounded blobs (no anti-aliasing) on a tiny native
    surface. Owns its own clock so pausing the render thread (e.g. under a
    debugger) doesn't jump the animation."""

    def __init__(self, initial_state: str) -> None:
        self._t0 = time.monotonic()
        self.state = initial_state if initial_state in _HUE else "IDLE"
        self.enter_t = 0.0
        self.mic = 0.0
        self.next_blink = 2.4
        self.blink_start = -1.0
        self.wink_start = -1.0
        self.wink_eye = 0
        self.g = {"w": 26.0, "h": 30.0, "r": 9.0, "dx": 0.0, "dy": 0.0, "skew": 0.0, "dim": 1.0}
        self.col = list(_HUE[self.state])

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
        age = t - self.enter_t
        pop = math.exp(-age * 4.5) * math.sin(age * 15)  # settle overshoot on state entry
        st = self.state
        if st == "ASLEEP":
            return {"w": 28, "h": 5 + math.sin(t * 1.15) * 1.6, "r": 2.5,
                     "dx": 0, "dy": 7 + math.sin(t * 1.15) * 1.2, "skew": 0.06, "dim": 0.55}
        if st == "IDLE":
            sx, sy = self._sway(t)
            return {"w": 26, "h": 30 + pop * 5, "r": 9,
                     "dx": sx, "dy": sy + math.sin(t * 1.5) * 1.1, "skew": 0, "dim": 1}
        if st == "LISTENING":
            return {"w": 27 + self.mic * 1.5, "h": 34 + self.mic * 4 + pop * 5, "r": 10,
                     "dx": math.sin(t * 0.8) * 1.6, "dy": -1 - self.mic, "skew": -0.05, "dim": 1}
        if st == "THINKING":
            return {"w": 22, "h": 19, "r": 8,
                     "dx": -3 + math.cos(t * 1.5) * 2.2, "dy": -5 + math.sin(t * 1.5) * 1.4,
                     "skew": 0.3, "dim": 1}
        if st == "SPEAKING":
            env = math.sin(t * 8.5) * 0.6 + math.sin(t * 4.1) * 0.4
            return {"w": 26 - env * 1.2, "h": 26 + env * 6, "r": 9,
                     "dx": 0, "dy": -env * 1.6, "skew": -0.05, "dim": 1}
        if st == "ERROR":
            return {"w": 28, "h": 11 + pop * 4, "r": 3, "dx": 0, "dy": 1, "skew": 0.55, "dim": 1}
        return {"w": 26, "h": 30, "r": 9, "dx": 0, "dy": 0, "skew": 0, "dim": 1}

    # ── drawing ──────────────────────────────────────────────────────────────
    @staticmethod
    def _blob(surface: pygame.Surface, cx: float, cy: float, w: float, h: float,
              r: float, skew: float, color: tuple[int, int, int]) -> None:
        """A pixel rounded rect with per-row horizontal slant: fill one 1px-tall
        rect per scanline, insetting near the top/bottom to approximate the
        corner radius and offsetting sideways for `skew`. No anti-aliasing —
        that hard edge is what makes the upscale read as crisp pixels instead
        of a blur."""
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

    def draw(self, surface: pygame.Surface) -> None:
        t = self.now()
        w, h = surface.get_size()
        tg = self._targets(t)
        g = self.g
        for key in g:
            g[key] = _lerp(g[key], tg[key], 0.16)
        mic_target = abs(math.sin(t * 3.1) * 0.6 + math.sin(t * 7.9) * 0.4) if self.state == "LISTENING" else 0.0
        self.mic = _lerp(self.mic, mic_target, 0.18)

        target_color = _HUE[self.state]
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
        cy, gap_half = h / 2, 20
        jitter = round(random.random() * 3 - 1.5) if self.state == "ERROR" and random.random() < 0.22 else 0

        for i in range(2):
            lid = blink * (wink if i == wink_eye else 1)
            eye_h = max(2.0, g["h"] * lid)
            cx = (-gap_half if i == 0 else gap_half) + w / 2 + g["dx"] + jitter
            skew = g["skew"] * (1 if i == 0 else -1)
            color = tuple(round(c * g["dim"]) for c in self.col)
            self._blob(surface, cx, cy + g["dy"], g["w"], eye_h, g["r"], skew, color)
            # Top-left specular pixel — retro shine, keeps blobs from reading flat.
            if eye_h > 8 and g["dim"] > 0.7:
                sx = round(cx - g["w"] / 2 + g["r"] * 0.8 + skew * eye_h / 3)
                sy = round(cy + g["dy"] - eye_h / 2 + 3)
                spec = tuple(round((c + 255) / 2) for c in color)  # 50% white blend
                surface.fill(spec, (sx, sy, 2, 2))

        # Occasional horizontal glitch slice — displace a strip sideways.
        if self.state == "ERROR" and random.random() < 0.3:
            y = random.randrange(0, max(1, h - 10))
            hh = min(4 + random.randrange(0, 6), h - y)
            strip = surface.subsurface((0, y, w, hh)).copy()
            surface.fill(_BG, (0, y, w, hh))
            surface.blit(strip, (round(random.random() * 10 - 5), y))


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
    ) -> None:
        self._fsm = fsm
        self._size = size
        self._fullscreen = fullscreen
        self._fps = fps
        self._debug = debug

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

        native = pygame.Surface((_NATIVE_W, _NATIVE_H))
        face = _Face(self._take_pending().name)
        scanlines = _build_scanlines(self._size)
        fringe = _build_fringe(self._size)
        clock = pygame.time.Clock()

        while not self._stop.is_set():
            # Draining the event queue keeps the window responsive (and lets a
            # window-manager close request stop us cleanly).
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop.set()

            face.set_state(self._take_pending().name)
            face.draw(native)

            # Nearest-neighbor upscale from the tiny native canvas — this is
            # what makes the blocky pixel edges instead of a blur.
            pygame.transform.scale(native, self._size, screen)
            _draw_scanlines(screen, scanlines, face.now())
            screen.blit(fringe, (0, 0))

            if overlay_font is not None:
                self._draw_overlay(screen, overlay_font, self._take_pending(), clock.get_fps())

            pygame.display.flip()
            clock.tick(self._fps)

        pygame.quit()
