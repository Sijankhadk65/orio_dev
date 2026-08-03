# Orio — LLM operator layer

The conversational brain for Orio, the wheeled mobile robot. It speaks *as* the
robot: you talk to it through the **mic**, a local LLM (via **Ollama**) replies
within the robot's scope, and the reply is spoken aloud with **ElevenLabs**
cloud TTS. (A keyboard mode is available too.) Runs on Windows, Linux (Jetson),
and macOS — mic/speaker I/O goes through `sounddevice` (PortAudio), not any
OS-specific CLI tool.

This is the top of the stack — it produces words now and, later, *semantic tool
calls* (`drive_to`, `move_arm_to`, `stop`, `get_status`) handed to the
deterministic mediator. **The LLM never talks to hardware directly and is never
in the control loop.**

```
mic → ASR (Whisper) → LLM (Ollama) → TTS (ElevenLabs) → speaker
                          │  later: semantic tool calls
                     deterministic mediator → STM32 → motors/servos
```

## Layout

| File | Role |
|---|---|
| `main.py` | Entry point (`uv run main.py`) |
| `orio/config.py` | All tunables (model, mic, voice, scope prompt) — env-overridable |
| `orio/audio_input.py` | Cross-platform mic capture (`sounddevice`) shared by ASR + wake word |
| `orio/asr.py` | RMS voice-activity gate + Whisper transcription |
| `orio/llm.py` | Ollama chat wrapper + rolling history + preflight checks |
| `orio/tts.py` | Pluggable TTS (`elevenlabs` engine, `console` fallback) |
| `orio/conversation.py` | The interactive talk loop (voice or keyboard) |
| `orio/eyes.py` | Animated face (`EyesController`), subscribes to FSM state changes |
| `tools/eyes_demo.py` | Cycles the eyes through every state, no mic/LLM needed |
| `settings.example.json` | Template for local `settings.json` (gitignored) — non-secret config, no env vars needed |

## One-time setup

Python deps are already managed by `uv` (`ollama`, `elevenlabs`, `faster-whisper`,
`sounddevice`). Three things live outside Python:

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

**3. Mic + speech recognizer** — list input/output devices with
`uv run python -m sounddevice`; if the wrong one is picked by default, pin it
with `ORIO_MIC_DEVICE`/`ORIO_SPEAKER_DEVICE` (index or name substring — see
Configuration below). The Whisper model (`base`, ~140 MB) downloads from
Hugging Face on first run and is cached under `~/.cache/huggingface`.

**4. Local settings (optional)** — for anything you want to keep set across
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
| `ORIO_EYES_CLIPS_DIR` | `assets/eyes` | Per-state Lottie clip directory |
| `ORIO_WAKE` | `1` | `0` to disable wake-word gating (always listening) |
| `ORIO_WAKE_ENGINE` | `whisper` | `whisper` (reuse ASR) or `oww` (dedicated openWakeWord model) |
| `ORIO_WAKE_PHRASES` | `hey orio,okay orio,…` | Comma-separated accepted phrases |
| `ORIO_WAKE_FUZZY` | `0.82` | Fuzzy-match tolerance (0–1); `>1.0` disables |
| `ORIO_FOLLOWUP_WINDOW_S` | `8.0` | Seconds to stay awake for a follow-up after replying |
| `ORIO_WAKE_OWW_MODEL` | `hey_jarvis` | openWakeWord model path or built-in name |
| `ORIO_WAKE_OWW_THRESHOLD` | `0.5` | Detection score above which a frame counts as the wake word |
| `ORIO_WAKE_OWW_FRAMEWORK` | `onnx` | `onnx` or `tflite` inference backend |

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
declines off-topic requests. Since no tools are wired yet, it won't pretend to
physically act — it says it'll be able to once its controls are connected.

## Next

Add the tool layer: define semantic tools with a strict schema, use constrained
decoding for valid tool-call JSON, and route calls through the deterministic
mediator. See the `orio_kb` notes (`orio_llm_command_layer.md`).
