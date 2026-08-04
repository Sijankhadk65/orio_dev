# Eyes animation — Orio's face

Animated robot eyes on the Jetson's HDMI panel (Elecrow RC070N 7", 1024x600)
that react to what Orio is doing. The eyes are a **subscriber** to the operator
FSM (`orio/fsm.py`), not their own state source: the conversation loop only
transitions the FSM, and the face maps each `State` to a clip.

> Status: built on `feature/eyes`. Renderer + FSM subscriber + per-state
> expression data are in and verified end-to-end.

## Approach — procedural pygame drawing (not Lottie/rlottie)

Originally these were pre-baked Lottie clips played back with `rlottie`,
specifically to keep the always-on face off the **GPU** (rlottie rasterizes on
the CPU) since that's running the LLM / object detector. That approach was
dropped after hitting real `rlottie` bugs: the build available here renders
keyframed Scale-type properties (a layer's own scale, a shape group's
transform, or a shape's own declared size) correctly only on their first
frame, and separately breaks a layer's Position the moment that same layer
also carries an animated Path — both confirmed by isolated repro, not a
config issue.

Since the actual art is just two parametric rounded rects per expression (no
designer-authored vector paths), each state is now a small keyframe table
(`assets/eyes/<state>.json`: per-eye color, corner radius, static rotation,
and a `(t, w, h, x, y)` list) that `orio/eyes.py` linearly interpolates and
draws directly with `pygame.draw.rect` every frame. This is *cheaper* per
frame than rasterizing a full Lottie shape tree, so the CPU-only goal still
holds, and it sidesteps the rlottie bugs entirely since nothing is rendered
through Lottie anymore.

Trade-off: no live blending between expressions, so a state change is a
**hard cut** — masked with the blink described below, same as before.

## State → clip map

| FSM state   | Clip         | Look                                            |
|-------------|--------------|-------------------------------------------------|
| `ASLEEP`    | `asleep`     | dim near-closed slits, slow breathing           |
| `IDLE`      | `idle`       | open calm eyes, periodic blink                  |
| `LISTENING` | `listening`  | wide/attentive, light pulse                     |
| `THINKING`  | `thinking`   | squint, glance up-and-away, slow drift          |
| `SPEAKING`  | `speaking`   | rhythmic bob/squint pulse                       |
| `ERROR`     | `error`      | flat red eyes (brief — recoverable)             |

Reaction one-shots (`happy`, `confused`) are **not** FSM states — they'll arrive
with the command/tool layer and get layered over the current sustained clip.

## Code

- **`orio/eyes.py`** — `EyesController(fsm, …)`:
  - `subscribe()`s to the FSM; the listener just flips a flag (it runs on the
    loop thread and must stay cheap).
  - A **background render thread** owns the pygame window and draws continuously
    at `fps`. Drawing off the main thread matters because every operator call
    (`stt.listen`, `convo.send`, `tts.speak`) **blocks** — the face would
    otherwise freeze during the liveliest moments.
  - Preloads every expression once; on a state change, hard-cuts to the new one.
  - A missing expression / no display only **disables the face**, never breaks
    the loop.
- **`orio/conversation.py`** — `_start_eyes(fsm)` constructs + `start()`s the
  controller when `config.EYES_ENABLED`, and `stop()`s it in the shutdown path.
  Import is **local** so text/headless runs never pull in pygame.
- **`tools/make_placeholder_eyes.py`** — generates placeholder expression data.
- **`tools/eyes_demo.py`** — cycles the states on screen to eyeball the look.

## Config (`orio/config.py`, all env-overridable)

| Env var                 | Default            | Meaning                              |
|-------------------------|--------------------|--------------------------------------|
| `ORIO_EYES`             | `0` (off)          | Enable the face (needs a display)    |
| `ORIO_EYES_FULLSCREEN`  | `1`                | Fullscreen kiosk; `0` = windowed dev |
| `ORIO_EYES_SIZE`        | `1024x600`         | Panel resolution                     |
| `ORIO_EYES_FPS`         | `30`               | Render rate (24–30 is plenty)        |
| `ORIO_EYES_CLIPS_DIR`   | `assets/eyes/`     | Where `<state>.json` clips live      |

## Expression data

`assets/eyes/<state>.json` holds the two eyes' motion as a small custom schema
(not Lottie):

```json
{
  "fps": 30,
  "eyes": {
    "left":  {"color": [r, g, b], "radius": 52, "rotation": 0,
              "keyframes": [{"t": 0, "w": 176, "h": 214, "x": 372, "y": 300}, ...]},
    "right": {...}
  }
}
```

`(w, h, x, y)` are linearly interpolated between keyframes each frame (see
`_EyeTrack` in `orio/eyes.py`); `color`/`radius`/`rotation` are static per eye.
Keep clips loop-clean (first keyframe's values == last keyframe's) since the
loop period is the last keyframe's `t`. See `assets/eyes/README.md` for the
per-state parameters, and `tools/make_placeholder_eyes.py` for a generator you
can copy to author new keyframe curves.

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

Phoneme/RMS-driven mouth-sync for `SPEAKING`, camera look-at gaze tracking (now
that eye position is a plain interpolated (x, y) each frame, continuous
parametric gaze is a straightforward extension rather than needing directional
look clips), and a real round-panel driver (GC9A01, etc.). The
`EyesController` interface is the swap point for all three.
