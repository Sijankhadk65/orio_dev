# Eyes animation — deferred plan

> Shelved until the robot has a real state machine to drive it. The FSM
> (`orio/fsm.py`, `feature/FSM`) is the prerequisite; the eyes are a *subscriber*
> to FSM state changes, not their own source of truth. Pick this back up once
> the FSM has landed on `develop`.

## Goal

Animated robot eyes on the Jetson's HDMI display that react to what Orio is
doing. Four visual states, one per `orio.fsm.State`:

| FSM state   | Eyes look                                              |
|-------------|--------------------------------------------------------|
| `IDLE`      | calm; occasional blink; slow subtle drift/look-around  |
| `LISTENING` | widen/attentive; pupils dilate; slightly faster blink  |
| `THINKING`  | pupils glance up-and-away; gentle "pondering" drift     |
| `SPEAKING`  | rhythmic bob/squint pulse while audio plays             |

(`ERROR` can get a look later — e.g. flat/X eyes.)

## Display target

**HDMI / Jetson screen** — fullscreen **pygame/SDL** window. Chosen because it's
the fastest to iterate on (visible over the remote/NoMachine session, `DISPLAY=:1`)
and needs no extra wiring. `pygame` has aarch64 wheels (verified `pygame==2.6.1`).
Keep the renderer behind an interface so a real panel (round SPI GC9A01, etc.)
can swap in later without touching the animation logic.

## Architecture

- **`orio/eyes.py`**
  - `EyesController` owns the pygame window + a **background render thread**
    (~60 fps). Public API: `start()`, `stop()`. It does **not** expose
    `set_state` to the loop — instead it **subscribes to the FSM** and reacts to
    `(old, new)` transitions. This keeps the conversation loop ignorant of the
    eyes entirely.
  - Per state it holds *target* parameters (eye openness, pupil offset, color,
    blink cadence) and **interpolates** current → target each frame, so
    transitions are smooth instead of snapping.
  - **Blink** is a timer-driven behavior layered over IDLE/LISTENING,
    independent of state.
- **Why concurrent:** every operator call (`stt.listen`, `convo.send`,
  `tts.speak`) blocks. Drawing from the same thread would freeze the eyes during
  exactly the moments they should be liveliest. The render thread animates
  continuously; the FSM transition is just a cheap signal.

## Integration

- `orio/conversation.py`: construct `EyesController(fsm)` when
  `config.EYES_ENABLED` and a display is present; `start()` before the loop,
  `stop()` in the shutdown path. No per-state calls in the loop — the FSM
  already emits the transitions (see `feature/FSM`).
- Config: add `EYES_ENABLED` (env `ORIO_EYES`, default off so headless/text
  runs are unaffected), plus optional `EYES_FULLSCREEN` / window size.
- Dep: re-add `pygame` (`uv add pygame`) when work resumes — it was removed when
  this was shelved.

## First-cut scope

Get the four states visibly distinct with smooth interpolation + blink. Defer:
phoneme/RMS-driven mouth-sync for SPEAKING, look-at-target tracking from the
camera, and the real panel driver.
