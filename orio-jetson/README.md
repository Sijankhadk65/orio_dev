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

## Run

```bash
uv run main.py
```

Speak when you see `🎤 listening…`; Orio transcribes you, replies in text, and
speaks it. Say "power down" / "goodbye orio" or press Ctrl-C to stop.

Prefer the keyboard? `ORIO_INPUT=text uv run main.py`.

## Configuration (env vars)

| Var | Default | Notes |
|---|---|---|
| `ORIO_LLM_MODEL` | `llama3.2:3b` | Any pulled Ollama model |
| `OLLAMA_HOST` | _(localhost)_ | Point at a remote daemon |
| `ORIO_LLM_TEMPERATURE` | `0.3` | Low — Orio has a narrow job |
| `ORIO_INPUT` | `voice` | `voice` (mic) or `text` (keyboard) |
| `ORIO_MIC_DEVICE` | _(system default)_ | `sounddevice` input device: index or name substring |
| `ORIO_ASR_MODEL` | `base` | Whisper size: `tiny`/`base`/`small`… |
| `ORIO_VAD_MIN_RMS` | `300` | Speech-gate floor (raise in a noisy room) |
| `ORIO_VAD_SILENCE_MS` | `800` | Trailing silence that ends a phrase |
| `ORIO_TTS` | `elevenlabs` | `elevenlabs` or `console` (no audio) |
| `ELEVENLABS_API_KEY` | _(required)_ | Set in `.env` — see `.env.example` |
| `ORIO_ELEVENLABS_VOICE_ID` | Rachel | Any ElevenLabs voice ID |
| `ORIO_ELEVENLABS_MODEL` | `eleven_turbo_v2_5` | ElevenLabs model ID |
| `ORIO_SPEAKER_DEVICE` | _(system default)_ | `sounddevice` output device: index or name substring |

No mic/speaker handy? Run keyboard + text-only:

```bash
ORIO_INPUT=text ORIO_TTS=console uv run main.py
```

## Scope

The system prompt in `config.py` keeps the small local model on-task: it talks
only about what Orio can do (driving, looking, moving arms/head, status) and
declines off-topic requests. Since no tools are wired yet, it won't pretend to
physically act — it says it'll be able to once its controls are connected.

## Next

Add the tool layer: define semantic tools with a strict schema, use constrained
decoding for valid tool-call JSON, and route calls through the deterministic
mediator. See the `orio_kb` notes (`orio_llm_command_layer.md`).
