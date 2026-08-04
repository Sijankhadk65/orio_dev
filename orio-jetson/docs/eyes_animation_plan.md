# Eyes animation — Orio's face

Animated robot eyes on the Jetson's HDMI panel (Elecrow RC070N 7", 1024x600)
that react to what Orio is doing. The eyes are a **subscriber** to the operator
FSM (`orio/fsm.py`), not their own state source: the conversation loop only
transitions the FSM, and `_Face` in `orio/eyes.py` eases toward that state's
target geometry every frame.

> Status: built on `feature/eyes`. Renderer + FSM subscriber are in and
> verified end-to-end, ported 1:1 from the reference `orio-eyes-standalone.html`
> canvas prototype.

## Approach — procedural pygame, ported from the reference canvas prototype

Originally these were pre-baked Lottie clips played back with `rlottie`, then
briefly a set of hand-authored keyframe JSON files interpolated in Python.
Both were dropped: `rlottie`'s keyframed Scale-type properties render
correctly only on their first frame (and Position breaks the moment a layer
also carries an animated Path — both confirmed by isolated repro, not a
config issue), and the hand-authored keyframes were only a sampled
approximation of the design's actual motion, not the real thing.

The real thing is `orio-eyes-standalone.html`: a self-contained canvas
prototype where every expression is a closed-form function of time and state
— width/height/position/skew/color computed each frame from sines, a seeded
pseudo-random sway, an exponential-decay "pop" on state entry, a blink/wink
state machine, a simulated mic envelope for `LISTENING`, and a glitch/jitter
layer for `ERROR`. `_Face` in `orio/eyes.py` is a direct line-by-line port of
that engine (see `_Face._targets`, `_Face._blob`, `_Face.draw`) — same
formulas, same constants, so the motion matches the reference exactly instead
of approximating it.

**Sharpness**: the reference draws on a tiny native `<canvas width=128
height=72>` with hard (non-anti-aliased) per-row rect fills, then scales it up
with CSS `image-rendering: pixelated`. We do the same: `_Face.draw` fills a
128×72 `pygame.Surface` one scanline at a time (`Surface.fill`, no
anti-aliasing), and `EyesController._run` blows that up to the panel's real
resolution with `pygame.transform.scale` (nearest-neighbor). Drawing
anti-aliased at full res and downsampling — an earlier version of this file
did that — softens the edges into a blur instead of the reference's crisp
blocky look.

Trade-off: no separate "clip swap" transition — a state change just retargets
the same continuous easing (`g[key] = lerp(g[key], target[key], 0.16)` each
frame, exactly like the reference), so the eyes visibly morph into the new
shape/color over a few hundred ms rather than cutting or blinking through it.

## State → look

| FSM state   | Look                                             |
|-------------|---------------------------------------------------|
| `ASLEEP`    | flat bars, slow breathing rise/fall               |
| `IDLE`      | tall blobs, gentle sway + periodic blink/wink     |
| `LISTENING` | taller, mic-reactive level pulse                  |
| `THINKING`  | small, slanted, drifting up-left                  |
| `SPEAKING`  | syllable bounce (squash/stretch)                  |
| `ERROR`     | angry inward slant + jitter + occasional glitch   |

Reaction one-shots (`happy`, `confused`) are **not** FSM states — they'll arrive
with the command/tool layer and get layered over the current sustained look.

## Code

- **`orio/eyes.py`**:
  - `_Face` — the ported engine: per-state target geometry (`_targets`), the
    hard-edged scanline blob renderer (`_blob`), and per-frame state
    (blink/wink timers, mic envelope, eased color/geometry) in `draw`.
  - `EyesController(fsm, …)`:
    - `subscribe()`s to the FSM; the listener just flips a flag (it runs on
      the loop thread and must stay cheap).
    - A **background render thread** owns the pygame window and draws
      continuously at `fps`. Drawing off the main thread matters because every
      operator call (`stt.listen`, `convo.send`, `tts.speak`) **blocks** — the
      face would otherwise freeze during the liveliest moments.
    - A display failure only **disables the face**, never breaks the loop.
- **`orio/conversation.py`** — `_start_eyes(fsm)` constructs + `start()`s the
  controller when `config.EYES_ENABLED`, and `stop()`s it in the shutdown path.
  Import is **local** so text/headless runs never pull in pygame.
- **`tools/eyes_demo.py`** — cycles the states on screen to eyeball the look.

## Config (`orio/config.py`, all env-overridable)

| Env var                 | Default            | Meaning                              |
|-------------------------|--------------------|--------------------------------------|
| `ORIO_EYES`             | `0` (off)          | Enable the face (needs a display)    |
| `ORIO_EYES_FULLSCREEN`  | `1`                | Fullscreen kiosk; `0` = windowed dev |
| `ORIO_EYES_SIZE`        | `1024x600`         | Panel resolution                     |
| `ORIO_EYES_FPS`         | `30`               | Render rate (24–30 is plenty)        |

There's no clips directory or transition-duration setting anymore — there are
no files to point at (the expressions are code, not data) and no separate
transition to tune (state changes just retarget the same easing).

## Tuning / regenerating

There's no asset file to swap in anymore — the look *is* `_Face` in
`orio/eyes.py`. To change a state's motion, edit its branch in `_targets` (or
the shared constants like `_NATIVE_W/_NATIVE_H`, `_HUE`, the blink timing in
`draw`) directly, the same way you'd edit `orio-eyes-standalone.html`'s
`targets()` method — they're meant to stay in lockstep. The standalone HTML
file remains the fastest way to iterate on new motion in a browser before
porting a change into Python.

## Run it

```bash
# Watch the states cycle on screen (windowed, good over NoMachine):
ORIO_EYES_FULLSCREEN=0 uv run python tools/eyes_demo.py

# With the live operator loop (face follows real FSM state):
ORIO_EYES=1 uv run main.py
```

## Future — extra GUI

A richer GUI (status panels, etc.) may sit alongside the eyes later. The
renderer is deliberately isolated behind `EyesController` + the FSM seam, so a
GUI can be added without touching the conversation loop. **Not in scope now** —
keep the face the only thing on the panel for the first cut.

## Deferred

Phoneme/RMS-driven mouth-sync for `SPEAKING`, camera look-at gaze tracking (eye
position is already a plain per-frame (x, y), so continuous parametric gaze is
a straightforward extension), a real round-panel driver (GC9A01, etc.), and
the rest of the reference's CRT chrome — the drifting scanlines
(`_build_scanlines`/`_draw_scanlines`) and the vertical RGB fringe
(`_build_fringe`) are ported, but the vignette and flicker overlays are still
only in `orio-eyes-standalone.html`. The
`EyesController` interface is the swap point for all of these.
