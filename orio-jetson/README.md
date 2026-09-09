# Orio — LLM operator layer

The conversational brain for Orio, the wheeled mobile robot. It speaks *as* the
robot: you talk to it through the **mic**, an LLM replies within the robot's
scope, and the reply is spoken aloud with **ElevenLabs** cloud TTS. (A keyboard
mode is available too.) Runs on Windows, Linux (Jetson), and macOS — mic/speaker
I/O goes through `sounddevice` (PortAudio), not any OS-specific CLI tool.

The chat-model backend is pluggable (`ORIO_LLM_PROVIDER`): **Anthropic Claude**
(cloud, the default — a local 3B model wasn't reliable enough at staying in
scope, see "Scope" below) or a local model via **Ollama**.

The LLM can also call **tools** ([LangChain](https://python.langchain.com/) +
native tool-calling) to answer questions it can't just make up: a vision
tool (ask "what do you see" and it captures a camera frame, runs it through a
YOLO object detector, and answers from the real result), and a knowledge-base
tool (ask what Orio is, what it can do, or anything loaded into its
deployment-specific knowledge, and it retrieves an answer from a local
sqlite-vec store instead of improvising), and **drive tools** — ask Orio to move
forward, back up, turn, stop, or go faster and it actually rolls (see "Driving"
below).

**The LLM never talks to hardware directly and is never in the control loop.**
A drive tool call lands in `orio/body.py`, which owns the STM32 links, and every
move it makes runs under the obstacle-avoidance policy in `orio/avoid.py` — the
model asks to go forward, the policy decides the heading thirty times a second.
There is no unguarded path to the wheels and no flag that makes one; if the
cameras will not open, the drive tools are not offered at all. The board's own
watchdog e-stops it 500 ms after the last heartbeat regardless. The model
chooses *what*; nothing it can say chooses *how*.

```
mic → ASR (ElevenLabs Scribe) → LLM (Claude or Ollama) ⇄ tools → TTS (ElevenLabs) → speaker
                                                          ├ vision: camera → YOLO
                                                          ├ knowledge base: sqlite-vec RAG
                                                          └ drive: body.py (bounded hop)
                                                                └ avoid.py ⇄ stereo (30 Hz)
                                                              → STM32 drivetrain → FSESCs → motors
                                                              → STM32 motion → neck servos
```

## Layout

| File | Role |
|---|---|
| `main.py` | Entry point (`uv run main.py`) |
| `orio/config.py` | All tunables (model, mic, voice, scope prompt) — env-overridable |
| `orio/audio_input.py` | Cross-platform mic capture (`sounddevice`) shared by ASR + wake word |
| `orio/asr.py` | RMS voice-activity gate + ElevenLabs Scribe (cloud) transcription |
| `orio/llm.py` | Chat wrapper (LangChain, `ChatAnthropic` or `ChatOllama`): history, streaming, tool-call loop |
| `orio/tools.py` | LLM-callable tools (drive, vision, knowledge base) — each degrades away if its hardware/deps are missing |
| `orio/body.py` | Owns the boards and cameras for the session: holds the neck pose, runs each drive tool's guarded hop |
| `orio/avoid.py` | The avoidance policy (`Avoider`) + the stereo thread feeding it (`Sensor`) — shared by the app and the teleop tool |
| `orio/seek.py` | The "go to that" behaviour: scan with the head, face the target, close the distance, all closed-loop |
| `orio/drivetrain.py` | Framed serial link to the drivetrain board (WHOAMI handshake, heartbeat, `set_drive`) |
| `orio/motion.py` | Framed serial link to the motion board (neck + arm servos) |
| `orio/stereo.py` | Stereo depth from the IMX219-83 pair, reduced to per-sector obstacle distances |
| `orio/vision.py` | Camera capture (OpenCV) + YOLO (`ultralytics`) object detection |
| `orio/knowledge.py` | Per-profile RAG knowledge base (sqlite-vec + local ONNX embeddings) |
| `orio/kb_ingest.py` | CLI to ingest `.md`/`.txt` documents into a knowledge-base profile |
| `orio/tts.py` | Pluggable TTS (`elevenlabs` engine, `console` fallback) |
| `orio/conversation.py` | The interactive talk loop (voice or keyboard) |
| `orio/eyes.py` | Animated face (`EyesController`), subscribes to FSM state changes |
| `tools/eyes_demo.py` | Cycles the eyes through every state, no mic/LLM needed |
| `tools/teleop_guarded.py` | WASD teleop on the same `orio/avoid.py` policy — keeps the CLI flags for sweeping its tunables |
| `settings.example.json` | Template for local `settings.json` (gitignored) — non-secret config, no env vars needed |

## One-time setup

Python deps are already managed by `uv` (`langchain-anthropic`, `elevenlabs`,
`sounddevice`). A few things live outside Python:

**1. Anthropic API key** (default LLM backend) — copy `.env.example` to `.env`
and fill in `ANTHROPIC_API_KEY` (get one at https://console.anthropic.com).
`config.py` loads `.env` automatically; real environment variables still take
precedence over it.

**1b. (Optional) Ollama, for a local/offline model instead** — set
`ORIO_LLM_PROVIDER=ollama` (env var or `settings.json`), then install + start
the Ollama daemon (not a pip package):

```bash
curl -fsSL https://ollama.com/install.sh | sh   # installs + starts the service
ollama pull llama3.2:3b                          # ~2 GB, fits the Orin Nano budget
```

If the service isn't running, start it with `ollama serve` (or
`systemctl start ollama` / the Ollama app on Windows).

**2. ElevenLabs API key** — same `.env`, fill in `ELEVENLABS_API_KEY` (get one
at https://elevenlabs.io). Used for both TTS and speech-to-text (Scribe).

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
Configuration below). Transcription is a cloud call to ElevenLabs Scribe — no
local model to download, but voice mode needs internet.

On Linux, leaving the device unset doesn't just take PortAudio's raw
"default" at face value: if a device literally named `pipewire` exists,
that's preferred instead (`orio/audio_input.py`'s `resolve_device()`). This
matters because PortAudio's "default" ALSA device can silently resolve to a
dead stream when PipeWire is holding the real mic open elsewhere — observed
on a Jetson, where "default" captured pure silence but "pipewire" correctly
reached the USB mic. An explicit `ORIO_MIC_DEVICE`/`ORIO_SPEAKER_DEVICE`
always overrides this.

**5. Camera (for the vision tool)** — on the Jetson this is the IMX219 CSI
pair, captured through the ISP via Argus. That needs an OpenCV built with
GStreamer, which PyPI's `opencv-python` is not, so the project uses JetPack's
system build instead. Link it into the venv (idempotent, safe to re-run):

```bash
sudo apt install python3-opencv      # once, if not already present
uv run python tools/link_system_cv2.py
```

**Re-run that after recreating the venv** — `uv sync` alone will not restore
the link, and without it CSI capture cannot work. Two related pins exist for
the same reason and should not be "cleaned up": `requires-python = ">=3.12,<3.13"`
(the system cv2 is built for 3.12 only) and `numpy<2` (it is compiled against
numpy 1.x). `[tool.uv] override-dependencies` also keeps `opencv-python` out of
the environment so it cannot shadow the link.

For a plain USB webcam instead, set `ORIO_CAMERA_USE_ARGUS=0` and pick the
device with `ORIO_CAMERA_INDEX` — that path is ordinary V4L2 and needs none of
the above.

The YOLO nano checkpoint (~6 MB) auto-downloads into `models/yolo/`
(gitignored) the first time the vision tool actually runs. No camera, or want
the LLM to run with no tools at all? Set `ORIO_TOOLS=0` — Orio still runs fine,
it just can't answer "what do you see".

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

From `orio-jetson/`, with `.env` filled in (and the Ollama daemon running, if
`ORIO_LLM_PROVIDER=ollama`):

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
| Talk only — don't open the boards or cameras | `ORIO_DRIVE=0 ORIO_NECK=0 uv run main.py` | `$env:ORIO_DRIVE="0"; $env:ORIO_NECK="0"; uv run main.py` |
| Drive gently (first run on a new floor) | `ORIO_DRIVE_SPEED_PERCENT=15 ORIO_DRIVE_MAX_STEP_S=1 uv run main.py` | `$env:ORIO_DRIVE_SPEED_PERCENT="15"; $env:ORIO_DRIVE_MAX_STEP_S="1"; uv run main.py` |
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
| `ORIO_LLM_PROVIDER` | `anthropic` | `anthropic` (Claude, cloud) or `ollama` (local model) |
| `ORIO_LLM_MODEL` | `claude-haiku-4-5-20251001` (anthropic) / `llama3.2:3b` (ollama) | Claude model ID, or any pulled Ollama model |
| `ANTHROPIC_API_KEY` | _(required for anthropic)_ | Set in `.env` — see `.env.example` |
| `OLLAMA_HOST` | _(localhost)_ | Point at a remote daemon (only used when `ORIO_LLM_PROVIDER=ollama`) |
| `ORIO_LLM_TEMPERATURE` | `0.3` | Low — Orio has a narrow job |
| `ORIO_MAX_HISTORY_TURNS` | `12` | Conversation turns kept in the rolling history |
| `ORIO_INPUT` | `voice` | `voice` (mic) or `text` (keyboard) |
| `ORIO_MIC_DEVICE` | _(system default)_ | `sounddevice` input device: index or name substring |
| `ORIO_ASR_MODEL` | `scribe_v1` | ElevenLabs Scribe model id |
| `ORIO_ASR_LANGUAGE` | `en` | ISO-639-1 language hint; `""` to auto-detect |
| `ORIO_VAD_MIN_RMS` | `300` | Speech-gate floor (raise in a noisy room) |
| `ORIO_VAD_SILENCE_MS` | `800` | Trailing silence that ends a phrase |
| `ORIO_VAD_MAX_PHRASE_S` | `15` | Hard cap on a single phrase's length |
| `ORIO_VAD_THRESHOLD_FACTOR` | `3.0` | Speech threshold as a multiple of the ambient noise floor |
| `ORIO_TOOLS` | `1` | `0` to disable all LLM tool-calling (e.g. no camera) |
| `ORIO_CAMERA_INDEX` | `0` | `cv2.VideoCapture` device index (USB webcams only, i.e. when Argus is off) |
| `ORIO_CAMERA_USE_ARGUS` | `1` | `1` to capture CSI cameras via `nvarguscamerasrc`; `0` for plain V4L2 |
| `ORIO_CAMERA_SENSOR_ID` | `0` | Argus sensor id — *not* the `/dev/video*` number, the two are inverted |
| `ORIO_CAMERA_WIDTH` | `1280` | Frame width handed to YOLO (ISP downscales in hardware) |
| `ORIO_CAMERA_HEIGHT` | `720` | Frame height handed to YOLO |
| `ORIO_CAMERA_FPS` | `30` | Sensor capture rate |
| `ORIO_YOLO_MODEL` | `models/yolo/yolo11n.pt` | Ultralytics checkpoint name or path; auto-downloads if missing |
| `ORIO_YOLO_CONFIDENCE` | `0.5` | Minimum detection confidence [0,1] to report an object |
| `ORIO_VISION_DEBUG` | `0` | `1` for a live camera + detection-box preview window |
| `ORIO_VISION_DEBUG_FPS` | `15` | Preview window's target refresh rate |
| `ORIO_STEREO` | `0` | `1` to enable stereo depth / obstacle detection |
| `ORIO_STEREO_LEFT_SENSOR_ID` | `0` | Argus sensor that is the physically *left* camera |
| `ORIO_STEREO_RIGHT_SENSOR_ID` | `1` | Argus sensor that is the physically *right* camera |
| `ORIO_STEREO_CAPTURE_WIDTH` / `_HEIGHT` | `1640` / `1232` | Sensor capture mode — the *binned* full-FOV one; see below |
| `ORIO_STEREO_WIDTH` / `_HEIGHT` | `320` / `240` | Matching resolution (small is faster *and* denser) |
| `ORIO_STEREO_PHOTOMETRIC_MATCH` | `1` | Relevel the right eye onto the left before matching |
| `ORIO_STEREO_EXPOSURE_NS` / `_GAIN` | *(unset)* | Pin both sensors to one fixed exposure/gain; unset = auto |
| `ORIO_STEREO_SECTORS` | `7` | Sectors the depth map is reduced to |
| `ORIO_STEREO_MIN_RANGE_M` / `_MAX_RANGE_M` | `0.25` / `4.0` | Usable range; outside reads unknown |
| `ORIO_STEREO_MIN_VALID_FRAC` | `0.10` | Valid-pixel floor before a sector reports a distance |
| `ORIO_STEREO_BAND_TOP` / `_BOTTOM` | `0.35` / `0.71` | Image band that can hold a collidable obstacle |
| `ORIO_STEREO_CALIBRATION` | `models/stereo/calibration.npz` | Calibration from `tools/calibrate_stereo.py` |
| `ORIO_DRIVETRAIN_PORT` | `/dev/orio_drive` | Drivetrain board's serial port — a udev symlink, never a raw `ttyACM*` |
| `ORIO_MOTION_PORT` | `/dev/orio_motion` | Motion board's serial port (neck + arm servos) |
| `ORIO_DRIVE` | `1` | `0` to leave the drivetrain closed — Orio then says it can't move |
| `ORIO_DRIVE_SPEED_PERCENT` | `5` | Starting duty; `set_speed` moves it within the min/max below |
| `ORIO_DRIVE_SPEED_MIN_PERCENT` / `_MAX_PERCENT` | `5` / `60` | Speed window the LLM cannot drive outside of |
| `ORIO_DRIVE_STEP_S` | `1.5` | How long one forward/backward hop lasts when the model doesn't say |
| `ORIO_TURN_STEP_S` | `0.7` | Same, for a turn in place |
| `ORIO_DRIVE_MAX_STEP_S` | `4.0` | Hard ceiling on a single hop — the bound on one wrong command |
| `ORIO_NECK` | `1` | `0` to leave the head alone and not open the motion board |
| `ORIO_NECK_PAN_DEG` / `_TILT_DEG` | `175` / `40` | Pose the neck is held at (vendor scale). **Tilt is the avoidance policy's aim — usable window is ~35–45, see `config.py`** |
| `ORIO_AVOID_STOP_M` | `0.50` | Never drive forward with anything known nearer than this |
| `ORIO_AVOID_CLEAR_M` | `1.20` | Beyond this the way ahead counts as open and Orio goes straight |
| `ORIO_AVOID_MIN_SCALE` | `0.35` | Duty scale at `STOP_M`, ramping to full at `CLEAR_M` |
| `ORIO_AVOID_STALE_S` | `0.50` | A reading older than this stops the robot — in every direction |
| `ORIO_AVOID_HALF_WIDTH_M` | `0.40` | Half the chassis plus margin — how wide the corridor that must stay clear is |
| `ORIO_AVOID_TURN_PENALTY` | `1.0` | Metres a 45° detour must be worth before it's taken |
| `ORIO_AVOID_TURN_GAIN` | `0.9` | Steering strength |
| `ORIO_AVOID_SMOOTH` | `0.35` | Steering low-pass (0–1]; lower is smoother, slower to react |
| `ORIO_AVOID_HYSTERESIS_M` | `0.30` | Bonus for staying on the side already turning toward |
| `ORIO_AVOID_RELEASE_M` | `0.12` | How far past `STOP_M` the way must clear before driving resumes |
| `ORIO_AVOID_COMMIT_CLEAR_S` | `0.8` | Clear road before Orio stops favouring the side it was turning to |
| `ORIO_AVOID_PIVOT_TIMEOUT_S` | `2.0` | Pivoting longer than this without clearing triggers a back-off |
| `ORIO_AVOID_BACKOFF_S` | `1.0` | How long the back-off reverses for — **reverses blind, no rear sensor** |
| `ORIO_AVOID_TICK_S` | `0.03` | How often a move asks the policy for a fresh decision |
| `ORIO_SEEK_SCAN_OFFSETS_DEG` | `0,-30,30,-55,55` | Head pan offsets swept looking for a target (+ is left) |
| `ORIO_SEEK_SCAN_TILTS_DEG` | `40,26,13` | Tilts swept at each pan; driving tilt first |
| `ORIO_LOOK_TILT_SWEEP_DEG` | `13,26,40,48` | Tilts a plain look samples (0 = up, ~50 = floor/stop) |
| `ORIO_LOOK_TILT_UP_DEG` | `0,13,26` | Tilts for "look up" |
| `ORIO_LOOK_TILT_DOWN_DEG` | `40,48` | Tilts for "look down" |
| `ORIO_LOOK_PAN_DEG` | `35` | How far the head turns for "look left" / "look right" |
| `ORIO_SEEK_SETTLE_S` | `0.7` | Wait after a head move before trusting the frame (servo + auto-exposure) |
| `ORIO_SEEK_TIMEOUT_S` | `60` | Hard bound on one `go_to` |
| `ORIO_SEEK_ARRIVE_M` | `= AVOID_CLEAR_M` | How close counts as arrived — set by the guard, not a free choice |
| `ORIO_SEEK_CENTRE_DEG` | `9` | Target within this of straight ahead counts as lined up |
| `ORIO_SEEK_TURN_BURST_S` | `0.3` | One pivot burst while lining up (timed, corrected by looking again) |
| `ORIO_SEEK_HOP_S` | `0.8` | One forward step of an approach |
| `ORIO_SEEK_MAX_LOST` | `4` | Frames the target may be missing before Orio gives up |
| `ORIO_KB_PROFILE` | `orio` | Knowledge-base profile to query (see Knowledge base below) |
| `ORIO_KB_TOP_K` | `3` | Max chunks retrieved per knowledge-base query |
| `ORIO_KB_DIR` | `kb/` | Where sqlite-vec profile databases live |
| `ORIO_TTS` | `elevenlabs` | `elevenlabs` or `console` (no audio) |
| `ELEVENLABS_API_KEY` | _(required)_ | Set in `.env` — see `.env.example`. Used for both TTS and STT (Scribe) |
| `ORIO_ELEVENLABS_VOICE_ID` | Rachel | Any ElevenLabs voice ID |
| `ORIO_ELEVENLABS_MODEL` | `eleven_turbo_v2_5` | ElevenLabs model ID |
| `ORIO_SPEAKER_DEVICE` | _(system default)_ | `sounddevice` output device: index or name substring |
| `ORIO_EYES` | `0` | `1` to enable the animated face |
| `ORIO_EYES_FULLSCREEN` | `1` | `0` to run windowed (e.g. dev box, NoMachine) |
| `ORIO_EYES_SIZE` | `1024x600` | Render size — match your panel |
| `ORIO_EYES_FPS` | `30` | Render frame rate |
| `ORIO_EYES_DEBUG` | `0` | `1` to overlay the FSM state name + FPS |
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

**Why capture goes through Argus** — the IMX219 exposes exactly one V4L2
format, `RG10` (10-bit packed Bayer), and only the Jetson ISP debayers it. A
plain `cv2.VideoCapture(index)` therefore returns a **solid green frame while
reporting success**: `read()` gives `True`, the array has the right shape, and
every pixel is identical. Nothing raises — it just looks like the camera works
and YOLO never detects anything. `ObjectDetector` logs an explicit error if it
ever sees a single-colour frame, so that failure can't go quiet again.

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

## Driving

Ask Orio to move and it moves. Eight tools: `move_forward`, `move_backward`,
`turn_left`, `turn_right`, `stop_moving`, `set_speed`, `look_around`, and
`go_to`. The first four take an optional `seconds`; the model normally leaves it
out and gets the default hop.

**`go_to` is the one that matters for "come here".** Reaching a person is not a
move, and asking an LLM to do it by emitting moves makes it guess a distance it
cannot measure and a heading it cannot see, one blind hop at a time. So the
model picks the target and nothing else; `orio/seek.py` does the rest, closed
loop: sweep the head across `ORIO_SEEK_SCAN_OFFSETS_DEG` running the detector at
each stop, pivot the chassis in short bursts until the target is within
`ORIO_SEEK_CENTRE_DEG` of straight ahead, then guarded forward hops with a
re-detect between each one. Nothing integrates — every decision comes from the
picture in front of it, because the robot drifts off heading (no odometry) and
the target walks about.

The bearing is exact rather than estimated: `stereo.obstacles()` splits the
frame into equal-width columns, so the sector a bounding box's centre falls into
is `int(cx / width * n)` and its range comes from the same depth map the policy
is steering on. That only holds if the box and the depth came from the same
frame, which is what `Sensor.snapshot()` is for.

**Approaching and avoiding are the same manoeuvre.** A person is an obstacle;
the policy steers around anything nearer than `AVOID_CLEAR_M` and won't drive
forward at all inside `AVOID_STOP_M`. Past that range "go to them" and "don't
hit them" are opposite instructions, and the guard wins — `go_to` gets no
exemption. So `ORIO_SEEK_ARRIVE_M` defaults to `AVOID_CLEAR_M`: Orio stops about
a metre short, which is where you'd stop in front of someone anyway.

**Looking is two-dimensional.** One tilt is one horizontal slice of the room,
and the tilt that finds a *person* is much higher than intuition suggests
because the cameras sit low — someone standing a metre away is mostly above the
driving eyeline. Measured on the robot: a person at 0.77 m was invisible at the
driving tilt across every pan, and found immediately at 26°. So both
`what_do_you_see` and `look_around` pan to where they were told and then tilt
through `ORIO_LOOK_TILT_SWEEP_DEG`, merging what they find by label and noting
whether it was above or below eyeline. `go_to`'s scan walks the same grid,
tilt-major, stopping the moment it finds the target — one stop in the common
case, fifteen worst case.

The tilt scale runs 0 = up, measured by sweeping the joint and looking at the
frames: 0 is the ceiling, 13–26 is a standing person's head and shoulders, 40 is
the ground ahead (the driving pose), and ~50 is the mechanical stop, past which
the joint ignores further commands while STATUS keeps reporting the angle it was
asked for.

`look_around` moves only the head (`left`/`right`/`ahead`/`up`/`down`) and
reports what it then sees. **Positive pan turns the head left** — measured on
the robot, not assumed. It leaves the head where you pointed it; driving re-aims
to the driving pose first (`Body._ensure_driving_pose`), so a move can never
inherit a look's aim.

**A move is a bounded hop, not a latch.** The teleop tool holds a direction down
at 30 Hz while a key is pressed; the LLM issues one command and then goes back
to talking, so `body.move()` commands the wheels, waits, and stops them in a
`finally` — the wheels cannot outlive the tool call that started them. Two
numbers bound how wrong one command can go: `ORIO_DRIVE_MAX_STEP_S` (how long)
and `ORIO_DRIVE_SPEED_MAX_PERCENT` (how fast). `set_speed` clamps into that
window and reports the clamped value, so the model cannot talk its way past it.

**Avoidance is not optional.** Every hop runs the policy in `orio/avoid.py`,
which reads the stereo sector map at ~30 Hz on its own thread and picks the
heading each tick: cruise straight, steer around, pivot to find a way through,
back off when boxed in, or halt. There is no toggle, no direction that bypasses
it, and no path from a tool call to `set_drive()` that skips it. It's
non-optional in a second sense too — if the cameras don't open, or the neck
won't hold the pose those distances are measured through, `can_drive` is False,
the drive tools are never bound, and Orio says it can't move. Blind and moving
isn't a reachable state.

Be exact about what a forward-facing sensor can cover, though:

| Direction | Covered by |
|---|---|
| Forward | The full policy — heading chosen every tick |
| Turns | Nothing; a pivot translates nowhere, and the sides aren't sensed |
| **Reverse** | **Nothing. There is no rear sensor — reversing is blind** |

The one guard that applies to *all four* is sight itself: a move refuses to
start, and stops mid-hop, whenever the reading is missing, failed, or staler
than `ORIO_AVOID_STALE_S`. A wedged camera stops the robot in every direction
rather than leaving it driving on a frozen picture of an empty corridor.

Because the policy often does something other than what was asked, each tool
returns what *actually* happened — "steering around something in the way",
"couldn't go forward", "stopped: no known clearance in any sector" — and the
system prompt tells Orio to report that rather than claim a clean success.

**The cameras are shared, not duplicated.** Stereo needs both sensors, and Argus
won't open a third handle on one it already owns. So with avoidance armed the
`what_do_you_see` tool is handed the stereo pair's rectified left frame
(`ObjectDetector.detect_in`) instead of opening its own capture. That frame is
`ORIO_STEREO_WIDTH`×`_HEIGHT` (320×240) rather than 1280×720, so expect the
model to miss small or distant objects it would otherwise catch. For the same
reason `ORIO_VISION_DEBUG=1` is refused while the body holds the cameras — use
`tools/stereo_debug.py` instead.

**The head is aimed at startup and held there.** `body.start()` puts the neck at
`ORIO_NECK_PAN_DEG` / `_TILT_DEG` before anything else: pan 175 makes the
cameras' "straight ahead" the chassis's, and tilt sets what ground the policy
measures. That tilt window is narrow and was measured on the robot — 30° looks
at the upper wall and cannot see the floor at all, 45° sees only floor closer
than `AVOID_CLEAR_M` so the robot never cruises, and past ~50° the joint is
against its stop. 40° is the middle. `config.py` carries the sweep. It has to be *held*,
not merely placed: the motion board's e-stop cuts the servo PWM rather than
freezing it, and it e-stops 500 ms after the last heartbeat, so a released neck
sags. That's why the motion link stays open for the whole session and the head
goes slack, deliberately, on the way out.

Each board is independent and neither is fatal. A drivetrain that won't open
costs the drive tools and swaps the movement half of the system prompt for one
that says Orio can't move (`config.NO_DRIVE_PROMPT`) — it never claims a
capability it doesn't have. A neck that won't pose is a warning today, but
becomes fatal once driving depends on seeing.

Both boards are found by their udev symlinks (`/dev/orio_drive`,
`/dev/orio_motion`), never a raw `/dev/ttyACM*` — the number is USB enumeration
order and swaps between boots, and both boards share the same framing, so a
drive frame sent to the motion board decodes cleanly as a joint angle. Each link
also confirms the board's identity on the wire before arming it.

## Stereo depth & obstacle detection

`orio/stereo.py` turns the IMX219-83's two sensors into depth, and reduces that
to the nearest obstacle in each of a few sectors across the view:

```python
from orio.stereo import ObstacleDetector
det = ObstacleDetector()
omap = det.sense()
omap.describe()          # "nearest 0.53 m left (uncalibrated, approximate)"
omap.clearance_ahead()   # metres straight on, or None if unknown
```

See it live — the stereo counterpart to `ORIO_VISION_DEBUG`:

```bash
uv run python tools/stereo_debug.py
```

Left pane is the camera with per-sector distance and valid-pixel percentage,
right pane is the depth map (warm near, cool far, black unknown).

**Perception only, still.** `ObstacleMap` carries distances, never velocities.
Nothing in `stereo.py` decides how fast to go, when to stop, or which way to
turn, and nothing here talks to the STM32. Those decisions live one layer up in
`orio/avoid.py`, which is what the drive tools and the teleop tool both steer
with — this file only ever describes the world.

### Calibrate before trusting the numbers

Uncalibrated, depth falls back to published optics plus a measured row offset.
Obstacles *rank* correctly — nearer things read nearer — but the absolute
metres carry real error. `ObstacleMap.calibrated` reports which mode produced a
reading, and `describe()` says "approximate" out loud rather than hiding it.

```bash
uv run python tools/calibrate_stereo.py --square-mm 25
```

Print a checkerboard, tape it flat to something rigid, and capture 20+ pairs at
varied distances and angles. Measure a square with calipers — that number sets
the scale of the entire calibration.

### Four hardware quirks, all measured

Both are silent failures, so they are pinned in config rather than discovered
again later:

- **The eye/sensor mapping depends on the cabling.** Argus sensor 0 is the
  physically *left* camera (`ORIO_STEREO_LEFT_SENSOR_ID` defaults to `0`), but
  the ribbons were crossed until 2026-09-05 and sensor 1 was left. Get it
  backwards and every disparity comes out negative; `depth()` drops negative
  disparities, so the map goes blank rather than raising. Re-measure after any
  CSI recabling.
- **The sensors are not row-aligned.** There is a consistent vertical offset —
  9 px at 240 px tall on the binned capture mode. SGBM assumes row-aligned
  input, so this is removed before matching — by the calibration when present,
  by the measured constant otherwise.
- **The capture mode is load-bearing.** `1920x1080` looks like the obvious
  choice and is a trap: it is a 1.71x centre *crop* of the array with no
  binning. Measured against the binned `1640x1232` mode it carries **4.7x the
  sensor noise** (sigma 8.47 vs 1.82) — each pixel gets a quarter of the light,
  the ISP answers with analog gain, and SGBM matches the noise. It also narrows
  the FOV to ~47° while `obstacles()` assumes 73°, and changes the focal length,
  so distances read ~28% low. Valid depth: **29.7% on the crop, 50.6% binned.**
- **The eyes auto-expose independently.** Argus has no cross-sensor sync, and
  they drift far apart — 56% brightness and 63% contrast mismatch on one indoor
  scene. SGBM compares raw intensities and is not illumination-invariant, so
  the right eye is releveled onto the left's mean/std before matching. Worth
  **29.7% → 41.9%** on its own, and **50.6% → 72.1%** combined with the binned
  mode. Set `ORIO_STEREO_EXPOSURE_NS`/`_GAIN` to fix it at the source instead.

  CLAHE is the tempting alternative here and is *worse* (24.5%): it amplifies
  each eye's noise independently, and the two eyes' noise differs.

### Why 320x240

Counter-intuitively, matching small is better here: it runs SGBM fast *and*
produces more valid pixels than 640-wide matching, because coarser matching
copes better with the blank walls this robot faces. Obstacle avoidance needs
range, not fine detail. The 4:3 aspect matches the binned capture mode — 320x180
against a 4:3 sensor mode would squash the frame and shear the epipolar
geometry. End to end, `sense_with_frames()` measured **29.7 fps at 67.7% valid
depth** on an indoor scene.

A sector below `ORIO_STEREO_MIN_VALID_FRAC` valid pixels reports `None`, not a
distance. Untextured surfaces genuinely cannot be measured by a passive stereo
pair, and unknown must never be acted on as clear.

## Knowledge base (RAG)

Orio's second tool: ask what it is, what it can do, or anything covered by
its deployment's knowledge, and the LLM calls `search_knowledge_base`, which
embeds the question locally and does a nearest-neighbor search over a
[sqlite-vec](https://github.com/asg017/sqlite-vec) collection —
`orio/knowledge.py`. Embeddings come from a small local ONNX model
(`BAAI/bge-small-en-v1.5` via
[fastembed](https://github.com/qdrant/fastembed)) — torch-free and
CPU-friendly, same reasoning as the ONNX wake-word backend. No API key, no
per-query network call. A question with no close-enough match returns
nothing rather than a random chunk, so the model can honestly say it doesn't
know instead of guessing — the distance cutoff (`_MAX_DISTANCE` in
`knowledge.py`) is tuned to favor that over confidently answering an
unrelated question; see the module's comments if a deployment needs it
retuned for a larger knowledge set.

Knowledge is split into **profiles** — each one its own sqlite-vec
collection under `kb/` (gitignored, generated data), selected by
`ORIO_KB_PROFILE` (see Configuration below). The default profile, `"orio"`,
self-seeds on first query with facts about Orio itself, the nex-ON platform
it runs on, and the CozmoBot Robotics team — no setup needed. To point Orio
at a different domain (e.g. turn it into a grocery-store assistant), ingest
that venue's documents into a new profile and switch to it:

```bash
uv run python -m orio.kb_ingest --profile grocery-store aisles.md hours.md
ORIO_KB_PROFILE=grocery-store uv run main.py
```

```powershell
uv run python -m orio.kb_ingest --profile grocery-store aisles.md hours.md
$env:ORIO_KB_PROFILE = "grocery-store"; uv run main.py
```

`kb_ingest.py` splits each `.md`/`.txt` file on blank lines — one paragraph
becomes one chunk — so write source documents as short, self-contained
paragraphs; a chunk is returned to the LLM verbatim and read aloud. Add
`--replace` to clear a profile's existing chunks before ingesting (e.g. when
re-ingesting an updated document set). The first embedding call downloads the
ONNX model (~130 MB, cached by `fastembed`/`huggingface_hub`) — needs
internet once, same pattern as the YOLO checkpoint.

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
`uv run main.py`. See `docs/eyes_animation_plan.md` — the expressions are
procedural code in `orio/eyes.py`, not asset files.

## Scope

The system prompt in `config.py` keeps the small local model on-task: it talks
only about what Orio can do (driving, looking, moving arms/head, status) and
declines off-topic requests. Vision is real (see above) — it uses the tool and
reports the actual result instead of guessing, and so is driving. Arms still
aren't wired up: it won't pretend to pick anything up, and says it'll be able to
once those controls are connected. The movement half of the prompt is chosen at
startup from the tools that actually bound, so what Orio claims about moving
always matches what it can really do.

Small local models (llama3.2:3b, the previous default) were occasionally
over-eager about invoking the tool, or invoked one that doesn't exist, on
questions that have nothing to do with vision — the main reason the default
backend switched to Claude. The system prompt explicitly guards against this
("only call it when actually asked about what you can see... never invent a
tool... never write JSON in your reply") regardless of backend, but it's worth
knowing this was largely a small-model quirk, not a bug in the tool-calling
code, if you see it recur on `ORIO_LLM_PROVIDER=ollama`.

## Next

**Rear sensing.** The one hole the current guard cannot cover: reverse is blind,
including the policy's own back-off. Everything else is bounded by something;
this is bounded only by keeping it short.

**Give the vision tool its resolution back.** It currently reads the 320×240
stereo frame because both sensors are spoken for. Either run YOLO on a
full-resolution grab between hops, or accept the smaller frame and say so.

After that: arm tools (`move_arm_to`) on the motion board, and `get_status`
telemetry. See the `orio_kb` notes (`orio_llm_command_layer.md`).
