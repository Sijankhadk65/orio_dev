# Orio — LLM operator layer

The conversational brain for Orio, the wheeled mobile robot. It speaks *as* the
robot: you talk to it through the **mic**, a local LLM (via **Ollama**) replies
within the robot's scope, and the reply is spoken aloud with **ElevenLabs**
cloud TTS. (A keyboard mode is available too.) Runs on Windows, Linux (Jetson),
and macOS — mic/speaker I/O goes through `sounddevice` (PortAudio), not any
OS-specific CLI tool.

The LLM can also call **tools** ([LangChain](https://python.langchain.com/) +
Ollama's native tool-calling) to answer questions it can't just make up — right
now, a vision tool: ask "what do you see" and it captures a camera frame, runs
it through a YOLO object detector, and answers from the real result. Tools here
are query-only. Motion (`drive_to`, `move_arm_to`, `stop`, `get_status`) will
be a separate, deterministic-mediator-routed layer later. **The LLM never talks
to hardware directly and is never in the control loop.**

```
mic → ASR (Whisper) → LLM (Ollama) ⇄ tools (vision: camera → YOLO) → TTS (ElevenLabs) → speaker
                          │  later: motion tool calls
                     deterministic mediator → STM32 → motors/servos
```

## Layout

| File | Role |
|---|---|
| `main.py` | Entry point (`uv run main.py`) |
| `orio/config.py` | All tunables (model, mic, voice, scope prompt) — env-overridable |
| `orio/audio_input.py` | Cross-platform mic capture (`sounddevice`) shared by ASR + wake word |
| `orio/asr.py` | RMS voice-activity gate + Whisper transcription |
| `orio/llm.py` | `ChatOllama` (LangChain) chat wrapper: history, streaming, tool-call loop |
| `orio/tools.py` | LLM-callable tools (currently: vision) — degrades to none on missing deps |
| `orio/vision.py` | Camera capture (OpenCV) + YOLO (`ultralytics`) object detection |
| `orio/tts.py` | Pluggable TTS (`elevenlabs` engine, `console` fallback) |
| `orio/conversation.py` | The interactive talk loop (voice or keyboard) |
| `orio/eyes.py` | Animated face (`EyesController`), subscribes to FSM state changes |
| `tools/eyes_demo.py` | Cycles the eyes through every state, no mic/LLM needed |
| `settings.example.json` | Template for local `settings.json` (gitignored) — non-secret config, no env vars needed |

## One-time setup

Python deps are already managed by `uv` (`ollama`, `elevenlabs`, `faster-whisper`,
`sounddevice`). A few things live outside Python:

**1. Install + start the Ollama daemon** (not a pip package):

```bash
curl -fsSL https://ollama.com/install.sh | sh   # installs + starts the service
ollama pull llama3.2:3b                          # ~2 GB, fits the Orin Nano budget
```

If the service isn't running, start it with `ollama serve` (or
`systemctl start ollama` / the Ollama app on Windows).

**2. ElevenLabs API key** — copy `.env.example` to `.env` and fill in
`ELEVENLABS_API_KEY` (get one at https://elevenlabs.io). `config.py` loads
`.env` automatically; real environment variables still take precedence over it.

**3. PortAudio (Linux/Jetson only)** — `sounddevice`'s wheel bundles PortAudio
for Windows/macOS, but on Linux it dynamically loads the system library,
which isn't installed by default:

```bash
sudo apt install -y libportaudio2
```

Without it, voice mode fails at startup with `OSError: PortAudio library not
found` (text mode is unaffected — it never imports `sounddevice`).

**4. Mic + speech recognizer** — list input/output devices with
`uv run python -m sounddevice`; if the wrong one is picked by default, pin it
with `ORIO_MIC_DEVICE`/`ORIO_SPEAKER_DEVICE` (index or name substring — see
Configuration below). The Whisper model (`base`, ~140 MB) downloads from
Hugging Face on first run and is cached under `~/.cache/huggingface`.

On Linux, leaving the device unset doesn't just take PortAudio's raw
"default" at face value: if a device literally named `pipewire` exists,
that's preferred instead (`orio/audio_input.py`'s `resolve_device()`). This
matters because PortAudio's "default" ALSA device can silently resolve to a
dead stream when PipeWire is holding the real mic open elsewhere — observed
on a Jetson, where "default" captured pure silence but "pipewire" correctly
reached the USB mic. An explicit `ORIO_MIC_DEVICE`/`ORIO_SPEAKER_DEVICE`
always overrides this.

**5. Camera (for the vision tool)** — needs a webcam at `ORIO_CAMERA_INDEX`
(default `0`, the first/only camera). The YOLO nano checkpoint (~6 MB)
auto-downloads into `models/yolo/` (gitignored) the first time the vision tool
actually runs. No camera, or want the LLM to run with no tools at all? Set
`ORIO_TOOLS=0` — Orio still runs fine, it just can't answer "what do you see".

**6. Local settings (optional)** — for anything you want to keep set across
runs (device pins, `ORIO_EYES`, etc.), copy `settings.example.json` to
`settings.json` and edit it, instead of setting env vars every time:

```bash
cp settings.example.json settings.json
```

```powershell
Copy-Item settings.example.json settings.json
```

`settings.json` is gitignored (like `.env`) — it's your local machine's
config, not something to commit. `config.py` reads it as a fallback layer
below real env vars, so a one-off `ORIO_EYES=0 uv run main.py` still overrides
whatever's in the file. Only non-secret tunables belong here — the ElevenLabs
key always comes from `.env`, never from `settings.json`, even if you were to
track it.

## Run

From `orio-jetson/`, with the Ollama daemon running and `.env` filled in:

```bash
uv run main.py
```

```powershell
uv run main.py
```

Say "Hey Orio" to wake it, then speak when you see `🎤 listening…`; Orio
transcribes you, replies in text, and speaks it. Say "power down" / "goodbye
orio" or press Ctrl-C to stop.

### Common recipes

> **Windows/PowerShell warning:** bash's `VAR=x command` inline form does
> **not** work in PowerShell — `uv run main.py ORIO_EYES=1` silently passes
> `ORIO_EYES=1` as an ignored argument to `main.py` instead of setting an env
> var, so the setting has no effect and nothing looks wrong (no error, it just
> runs with defaults). Always use the PowerShell column below on Windows — or,
> for anything you don't want to retype every run, put it in `settings.json`
> instead (see One-time setup above) and just run `uv run main.py`.

| What | bash | PowerShell |
|---|---|---|
| Defaults (voice + wake word + ElevenLabs) | `uv run main.py` | `uv run main.py` |
| Keyboard only, no mic | `ORIO_INPUT=text uv run main.py` | `$env:ORIO_INPUT="text"; uv run main.py` |
| No audio output (text replies only) | `ORIO_TTS=console uv run main.py` | `$env:ORIO_TTS="console"; uv run main.py` |
| Fully headless (no mic, no speaker) | `ORIO_INPUT=text ORIO_TTS=console uv run main.py` | `$env:ORIO_INPUT="text"; $env:ORIO_TTS="console"; uv run main.py` |
| Animated eyes, windowed (dev box) | `ORIO_EYES=1 ORIO_EYES_FULLSCREEN=0 uv run main.py` | `$env:ORIO_EYES="1"; $env:ORIO_EYES_FULLSCREEN="0"; uv run main.py` |
| Animated eyes, fullscreen (robot panel) | `ORIO_EYES=1 uv run main.py` | `$env:ORIO_EYES="1"; uv run main.py` |
| Pin a specific mic / speaker | `ORIO_MIC_DEVICE=1 ORIO_SPEAKER_DEVICE=4 uv run main.py` | `$env:ORIO_MIC_DEVICE="1"; $env:ORIO_SPEAKER_DEVICE="4"; uv run main.py` |
| Dedicated wake-word model (openWakeWord) | `ORIO_WAKE_ENGINE=oww uv run main.py` | `$env:ORIO_WAKE_ENGINE="oww"; uv run main.py` |
| Always-on listening, no wake word | `ORIO_WAKE=0 uv run main.py` | `$env:ORIO_WAKE="0"; uv run main.py` |
| No tools (no camera, or don't want vision) | `ORIO_TOOLS=0 uv run main.py` | `$env:ORIO_TOOLS="0"; uv run main.py` |
| Pin a specific camera | `ORIO_CAMERA_INDEX=1 uv run main.py` | `$env:ORIO_CAMERA_INDEX="1"; uv run main.py` |

PowerShell env vars set with `$env:` persist for the rest of that terminal
session (until you close it or explicitly clear them), so once set they apply
to every `uv run` after — you don't need to repeat them each command.

## Configuration

Every var below can be set as an env var (see recipes above) or as a key in
`settings.json` (see One-time setup) — same names, same string values, either
way.

| Var | Default | Notes |
|---|---|---|
| `ORIO_LLM_MODEL` | `llama3.2:3b` | Any pulled Ollama model |
| `OLLAMA_HOST` | _(localhost)_ | Point at a remote daemon |
| `ORIO_LLM_TEMPERATURE` | `0.3` | Low — Orio has a narrow job |
| `ORIO_MAX_HISTORY_TURNS` | `12` | Conversation turns kept in the rolling history |
| `ORIO_INPUT` | `voice` | `voice` (mic) or `text` (keyboard) |
| `ORIO_MIC_DEVICE` | _(system default)_ | `sounddevice` input device: index or name substring |
| `ORIO_ASR_MODEL` | `base` | Whisper size: `tiny`/`base`/`small`… |
| `ORIO_ASR_COMPUTE_TYPE` | `int8` | Whisper CPU quantization |
| `ORIO_ASR_LANGUAGE` | `en` | Whisper transcription language |
| `ORIO_VAD_MIN_RMS` | `300` | Speech-gate floor (raise in a noisy room) |
| `ORIO_VAD_SILENCE_MS` | `800` | Trailing silence that ends a phrase |
| `ORIO_VAD_MAX_PHRASE_S` | `15` | Hard cap on a single phrase's length |
| `ORIO_VAD_THRESHOLD_FACTOR` | `3.0` | Speech threshold as a multiple of the ambient noise floor |
| `ORIO_TOOLS` | `1` | `0` to disable all LLM tool-calling (e.g. no camera) |
| `ORIO_CAMERA_INDEX` | `0` | `cv2.VideoCapture` device index |
| `ORIO_YOLO_MODEL` | `models/yolo/yolo11n.pt` | Ultralytics checkpoint name or path; auto-downloads if missing |
| `ORIO_YOLO_CONFIDENCE` | `0.5` | Minimum detection confidence [0,1] to report an object |
| `ORIO_VISION_DEBUG` | `0` | `1` for a live camera + detection-box preview window |
| `ORIO_VISION_DEBUG_FPS` | `15` | Preview window's target refresh rate |
| `ORIO_TTS` | `elevenlabs` | `elevenlabs` or `console` (no audio) |
| `ELEVENLABS_API_KEY` | _(required)_ | Set in `.env` — see `.env.example` |
| `ORIO_ELEVENLABS_VOICE_ID` | Rachel | Any ElevenLabs voice ID |
| `ORIO_ELEVENLABS_MODEL` | `eleven_turbo_v2_5` | ElevenLabs model ID |
| `ORIO_SPEAKER_DEVICE` | _(system default)_ | `sounddevice` output device: index or name substring |
| `ORIO_EYES` | `0` | `1` to enable the animated face |
| `ORIO_EYES_FULLSCREEN` | `1` | `0` to run windowed (e.g. dev box, NoMachine) |
| `ORIO_EYES_SIZE` | `1024x600` | Render size — match your panel |
| `ORIO_EYES_FPS` | `30` | Render frame rate |
| `ORIO_EYES_DEBUG` | `0` | `1` to overlay the FSM state name + FPS |
| `ORIO_EYES_TRANSITION_MS` | `400` | Blink duration masking a state's clip swap; `0` for an instant cut |
| `ORIO_EYES_CLIPS_DIR` | `assets/eyes` | Per-state Lottie clip directory |
| `ORIO_WAKE` | `1` | `0` to disable wake-word gating (always listening) |
| `ORIO_WAKE_ENGINE` | `whisper` | `whisper` (reuse ASR) or `oww` (dedicated openWakeWord model) |
| `ORIO_WAKE_PHRASES` | `hey orio,okay orio,…` | Comma-separated accepted phrases |
| `ORIO_WAKE_FUZZY` | `0.82` | Fuzzy-match tolerance (0–1); `>1.0` disables |
| `ORIO_FOLLOWUP_WINDOW_S` | `8.0` | Seconds to stay awake for a follow-up after replying |
| `ORIO_WAKE_OWW_MODEL` | `hey_jarvis` | openWakeWord model path or built-in name |
| `ORIO_WAKE_OWW_THRESHOLD` | `0.5` | Detection score above which a frame counts as the wake word |
| `ORIO_WAKE_OWW_FRAMEWORK` | `onnx` | `onnx` or `tflite` inference backend |

## Vision (object detection)

Orio's first real tool: ask "what do you see" (or similar) and the LLM calls
`what_do_you_see`, which grabs one frame from the camera, runs it through a
YOLO nano model, and gets back the actual objects, their rough position
(left/center/right, close/far), and confidence — the model answers from that,
not a guess. It's read-only: the tool only reports what's visible, it never
drives or moves anything.

Missing camera, opencv, or ultralytics? `get_tools()` in `orio/tools.py`
catches it and Orio just runs with no tools, same as any other optional piece
in this app — nothing else breaks. Force that off explicitly with
`ORIO_TOOLS=0`.

**Debug preview** — set `ORIO_VISION_DEBUG=1` (handy in `settings.json`, see
One-time setup) for a second window showing the live camera feed with
detection boxes and an FPS counter drawn on it, so you can see exactly what
the vision tool sees, continuously, not just at the moment it's asked. It
shares the same camera/model as the LLM's tool calls (via a lock — the two
never fight over the camera), refreshes at `ORIO_VISION_DEBUG_FPS`, and closes
with Esc or on shutdown. Off by default — it's a dev aid the LLM never sees,
not something to leave on for the robot's normal operation.

To sanity-check the camera + model directly, without the LLM in the loop:

```bash
uv run python -c "from orio.vision import ObjectDetector; print(ObjectDetector().detect_once())"
```

The first run downloads the YOLO nano checkpoint (~6 MB) into `models/yolo/`
(gitignored, like `voices/`).

## Eyes / face display

Animated eyes react to Orio's FSM state (idle / listening / thinking /
speaking / asleep / error) — off by default (`ORIO_EYES=0`), since headless/CI
runs shouldn't try to open a display. On the robot it renders fullscreen on the
Elecrow 7" panel (1024×600); on a dev box, run it windowed instead.

To just eyeball the look without the mic/LLM running, use the demo tool — it
drives a `StateMachine` through every state on a timer:

```bash
ORIO_EYES_FULLSCREEN=0 uv run python tools/eyes_demo.py
```

PowerShell:

```powershell
$env:ORIO_EYES_FULLSCREEN = "0"; uv run python tools/eyes_demo.py
```

Ctrl-C to quit. To enable the face in the real conversation loop, set
`ORIO_EYES=1` (and `ORIO_EYES_FULLSCREEN=0` if you're not on the panel) before
`uv run main.py`. Note the checked-in clips under `assets/eyes/` are
placeholder art (`tools/make_placeholder_eyes.py`), not the final designs.

## Scope

The system prompt in `config.py` keeps the small local model on-task: it talks
only about what Orio can do (driving, looking, moving arms/head, status) and
declines off-topic requests. Vision is real (see above) — it uses the tool and
reports the actual result instead of guessing. Motion still isn't wired up: it
won't pretend to physically drive or move its arms, and says it'll be able to
once its controls are connected.

Small local models (llama3.2:3b) are occasionally over-eager about invoking
the tool, or invoke one that doesn't exist, on questions that have nothing to
do with vision. The system prompt explicitly guards against this ("only call
it when actually asked about what you can see... never invent a tool... never
write JSON in your reply"), which fixed the worst cases in testing, but it's
worth knowing this is a small-model quirk, not a bug in the tool-calling code,
if you see it recur — tune the prompt further or try a larger model.

## Next

Motion tools: define `drive_to`/`move_arm_to`/`stop`/`get_status` the same way
vision was added (a LangChain `@tool` in `orio/tools.py`), but route their
execution through the deterministic mediator over the STM32 serial link
instead of running locally like vision does — the LLM must stay out of the
control loop. See the `orio_kb` notes (`orio_llm_command_layer.md`).
