"""Animated eyes — Orio's face on the HDMI panel, driven by the operator FSM.

The eyes are a *subscriber* to `fsm.StateMachine`, not their own state source:
the conversation loop never calls into here. It just transitions the FSM, and
this controller maps the new `State` to a pre-baked **Lottie clip** and plays it.

Why pre-baked clips (not procedural): the expressions are a small fixed set, and
`rlottie` rasterizes them on the **CPU**, so the always-on face never contends
with the GPU running the LLM / object detector. rlottie itself has no live
blending between clips, so a state change is masked with a **blink** instead —
eyelids sweep shut, the clip swaps while hidden, then they sweep back open —
the same trick most robot faces use to hide a hard cut behind natural eye
behavior. Drawn as a pygame overlay over whatever the clip renders, so it
works unmodified once designer art replaces the placeholders. See
docs/eyes_animation_plan.md.

Concurrency: a background thread owns the pygame window and renders continuously
(~`fps`), because every operator call (`stt.listen`, `convo.send`, `tts.speak`)
blocks the main thread — drawing there would freeze the face during exactly the
moments it should be liveliest. The FSM listener (run on the loop's thread) only
flips a flag; the render thread picks up the new state on its next frame.
"""

from __future__ import annotations

import logging
import os
import threading
from pathlib import Path

import pygame
import rlottie_python as rl

from .fsm import State, StateMachine

log = logging.getLogger(__name__)

# Which clip file backs each FSM state. Files live in the clips dir as
# "<name>.json" (Lottie). Reaction one-shots (happy/confused) come later with
# the command layer — they aren't FSM states, so they're not mapped here yet.
_STATE_CLIP: dict[State, str] = {
    State.ASLEEP: "asleep",
    State.IDLE: "idle",
    State.LISTENING: "listening",
    State.THINKING: "thinking",
    State.SPEAKING: "speaking",
    State.ERROR: "error",
}


class _Clip:
    """A loaded Lottie animation plus its frame metadata."""

    def __init__(self, path: Path) -> None:
        self.name = path.stem
        self._anim = rl.LottieAnimation.from_file(str(path))
        self.total = max(1, self._anim.lottie_animation_get_totalframe())
        self.fps = self._anim.lottie_animation_get_framerate() or 30.0
        self.w, self.h = self._anim.lottie_animation_get_size()

    def surface(self, frame: int) -> pygame.Surface:
        """Rasterize `frame` (CPU) into a pygame surface."""
        buf = self._anim.lottie_animation_render(
            frame_num=frame % self.total, width=self.w, height=self.h
        )
        return pygame.image.frombuffer(buf, (self.w, self.h), "BGRA")

    def destroy(self) -> None:
        self._anim.lottie_animation_destroy()


class EyesController:
    """Owns the face window + render thread; reacts to FSM transitions."""

    def __init__(
        self,
        fsm: StateMachine,
        *,
        clips_dir: Path,
        size: tuple[int, int] = (1024, 600),
        fullscreen: bool = True,
        fps: int = 30,
        debug: bool = False,
        transition_ms: int = 250,
    ) -> None:
        self._fsm = fsm
        self._clips_dir = clips_dir
        self._size = size
        self._fullscreen = fullscreen
        self._fps = fps
        self._debug = debug
        self._transition_ms = transition_ms

        self._lock = threading.Lock()
        self._pending = fsm.state  # latest state the render thread should show
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._unsubscribe = None
        self._clips: dict[str, _Clip] = {}

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

    # ── render thread ────────────────────────────────────────────────────────
    def _load_clips(self) -> None:
        for name in set(_STATE_CLIP.values()):
            path = self._clips_dir / f"{name}.json"
            if not path.exists():
                log.warning("eyes: missing clip %s", path)
                continue
            try:
                self._clips[name] = _Clip(path)
            except Exception:
                log.exception("eyes: failed to load clip %s", path)

    def _clip_for(self, state: State) -> _Clip | None:
        clip = self._clips.get(_STATE_CLIP.get(state, "idle"))
        return clip or self._clips.get("idle")

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

    # Near-black, matching the placeholder clips' own backdrop (see
    # tools/make_placeholder_eyes.py's `_bg()`) so the sweep reads as the
    # face's own eyelids closing, not a foreign color bar. Fine as a default
    # for designer art too, since almost every robot-face design is dark-bg.
    _LID_COLOR = (10, 10, 13)

    def _draw_lids(self, screen: pygame.Surface, closed: float) -> None:
        """Draw eyelids converging from the top/bottom edges.

        `closed` in [0, 1]: 0 is fully open (no-op), 1 is fully shut (the
        whole screen covered, top and bottom bars meeting at the centre).
        """
        if closed <= 0:
            return
        h = round(closed * self._size[1] / 2)
        if h <= 0:
            return
        w = self._size[0]
        pygame.draw.rect(screen, self._LID_COLOR, (0, 0, w, h))
        pygame.draw.rect(screen, self._LID_COLOR, (0, self._size[1] - h, w, h))

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

        self._load_clips()
        if not self._clips:
            log.error("eyes: no clips loaded — face disabled")
            pygame.quit()
            return

        # Split the configured duration into a close half and an open half —
        # each at least 1 frame so a very short transition_ms still blinks.
        half_frames = max(1, round(self._transition_ms / 1000 * self._fps / 2))

        clock = pygame.time.Clock()
        shown = self._take_pending()
        clip = self._clip_for(shown)
        frame = 0
        # While blinking: "phase" is "close" (sweeping shut over the outgoing
        # clip) or "open" (sweeping back over the already-swapped-in clip);
        # "step" counts frames within the current phase.
        blink: dict | None = None

        while not self._stop.is_set():
            # Draining the event queue keeps the window responsive (and lets a
            # window-manager close request stop us cleanly).
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop.set()

            target = self._take_pending()
            if target is not shown:
                new_clip = self._clip_for(target)
                shown = target
                if half_frames > 0 and new_clip is not None:
                    if blink is not None and blink["phase"] == "close":
                        # Still sweeping shut — just open into the newer
                        # target instead of restarting the close.
                        blink["to_clip"] = new_clip
                    else:
                        # Idle, or already opening into a now-stale target:
                        # (re)start a fresh blink.
                        blink = {"to_clip": new_clip, "phase": "close", "step": 0}
                else:
                    clip = new_clip
                    frame = 0
                    blink = None

            if blink is not None:
                if clip is not None:
                    screen.blit(clip.surface(frame), (0, 0))
                    frame += 1
                closing = blink["phase"] == "close"
                p = (blink["step"] + 1) / half_frames
                eased = p * p * (3 - 2 * p)  # smoothstep: eases in/out instead of a constant-speed sweep
                self._draw_lids(screen, eased if closing else 1.0 - eased)
                blink["step"] += 1
                if blink["step"] >= half_frames:
                    if closing:
                        clip, frame = blink["to_clip"], 0
                        blink["phase"], blink["step"] = "open", 0
                    else:
                        blink = None
            elif clip is not None:
                screen.blit(clip.surface(frame), (0, 0))
                frame = (frame + 1) % clip.total

            if overlay_font is not None:
                self._draw_overlay(screen, overlay_font, shown, clock.get_fps())

            pygame.display.flip()
            clock.tick(self._fps)

        for clip in self._clips.values():
            clip.destroy()
        pygame.quit()
