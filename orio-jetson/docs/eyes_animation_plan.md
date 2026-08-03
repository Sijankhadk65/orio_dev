# Eyes animation — Orio's face

Animated robot eyes on the Jetson's HDMI panel (Elecrow RC070N 7", 1024x600)
that react to what Orio is doing. The eyes are a **subscriber** to the operator
FSM (`orio/fsm.py`), not their own state source: the conversation loop only
transitions the FSM, and the face maps each `State` to a clip.

> Status: built on `feature/eyes`. Renderer + FSM subscriber + per-state
> placeholder clips are in and verified end-to-end.

## Approach — pre-baked Lottie clips (not procedural)

The expressions are a small fixed set, so they're authored **ahead of time** as
Lottie clips and played back at runtime with **`rlottie`**, which rasterizes on
the **CPU**. That's the whole point: the always-on face never contends with the
**GPU** running the LLM / object detector. (An earlier draft of this plan drew
the eyes procedurally in pygame each frame — superseded; we don't animate at
runtime.)

Trade-off: `rlottie` has **no live blending** between clips, so a state change is
a **hard cut** to the new clip. If a cut looks too abrupt, author a short
transition clip rather than trying to blend at runtime.

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
  - Preloads every clip once; on a state change, hard-cuts to the new clip.
  - A missing clip / no display only **disables the face**, never breaks the loop.
- **`orio/conversation.py`** — `_start_eyes(fsm)` constructs + `start()`s the
  controller when `config.EYES_ENABLED`, and `stop()`s it in the shutdown path.
  Import is **local** so text/headless runs never pull in pygame/rlottie.
- **`tools/make_placeholder_eyes.py`** — generates the placeholder clips.
- **`tools/eyes_demo.py`** — cycles the states on screen to eyeball the look.

## Config (`orio/config.py`, all env-overridable)

| Env var                 | Default            | Meaning                              |
|-------------------------|--------------------|--------------------------------------|
| `ORIO_EYES`             | `0` (off)          | Enable the face (needs a display)    |
| `ORIO_EYES_FULLSCREEN`  | `1`                | Fullscreen kiosk; `0` = windowed dev |
| `ORIO_EYES_SIZE`        | `1024x600`         | Panel resolution                     |
| `ORIO_EYES_FPS`         | `30`               | Render rate (24–30 is plenty)        |
| `ORIO_EYES_CLIPS_DIR`   | `assets/eyes/`     | Where `<state>.json` clips live      |

## Clips — placeholders now, designer art later

`assets/eyes/<state>.json` currently holds **placeholder** clips (simple cyan
rounded-rect eyes) so the pipeline runs today. They're the **same format** the
final art will use, so swapping in designer clips is just overwriting the files
(keep the names = FSM state). Authoring recipe: simple **vector** shapes, canvas
**1024x600**, 24–30 fps, loop-clean (first frame == last). Tools: After Effects +
Bodymovin, or LottieFiles / Lottielab → export Lottie JSON.

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

Phoneme/RMS-driven mouth-sync for `SPEAKING`, camera look-at gaze tracking
(pre-baked clips can't do continuous parametric gaze — would need directional
look clips or a small procedural overlay), and a real round-panel driver
(GC9A01, etc.). The `EyesController` interface is the swap point for all three.
