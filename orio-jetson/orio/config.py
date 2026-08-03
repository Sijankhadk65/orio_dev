"""Configuration for the Orio operator layer.

Everything tunable lives here and is overridable by environment variable so the
same code runs on a dev box and on the Jetson without edits.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from dotenv import load_dotenv

# Repo root (…/orio), used to resolve the bundled voice model.
ROOT = Path(__file__).resolve().parent.parent

# Load secrets/overrides from a local .env (see .env.example) before reading
# any os.environ.get() below. Real env vars already set still take precedence
# — load_dotenv() defaults to not overriding existing ones.
load_dotenv(ROOT / ".env")


def _load_settings() -> dict:
    """Load settings.json (see settings.example.json), if present.

    A plain JSON alternative to setting ORIO_* env vars every run — handy on
    Windows, where bash's `VAR=x command` inline syntax doesn't work in
    PowerShell (it silently passes `VAR=x` as an ignored argument instead).
    Not for secrets: ELEVENLABS_API_KEY is intentionally read from .env only,
    never from here, so it can't end up committed if this file is tracked.
    """
    path = ROOT / "settings.json"
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        print(f"⚠ settings.json is invalid ({exc}); ignoring it.")
        return {}


_SETTINGS = _load_settings()


def _env(key: str, default: str) -> str:
    """Resolve a tunable: shell env var (incl. .env) > settings.json > default.

    Values must be strings in settings.json, same as env vars; an empty value
    at any layer falls through to the next rather than being treated as set.
    """
    return str(os.environ.get(key) or _SETTINGS.get(key) or default)


# ── LLM (Ollama) ──────────────────────────────────────────────────────────────
# A 4-bit ~3B model is the sweet spot for the Orin Nano 8GB shared memory budget
# (JetPack + ASR + TTS leave the LLM ~3–4GB). Override with ORIO_LLM_MODEL.
LLM_MODEL = _env("ORIO_LLM_MODEL", "llama3.2:3b")

# Ollama daemon address. Leave unset to use the client default (localhost:11434).
OLLAMA_HOST = _env("OLLAMA_HOST", "") or None

# Sampling: keep it tight — Orio answers a small, fixed set of requests, not
# open-ended creative writing.
LLM_TEMPERATURE = float(_env("ORIO_LLM_TEMPERATURE", "0.3"))

# Cap conversation history (user+assistant turns kept, excluding the system
# prompt) so memory/latency stay bounded on-device.
MAX_HISTORY_TURNS = int(_env("ORIO_MAX_HISTORY_TURNS", "12"))


# ── TTS ──────────────────────────────────────────────────────────────────────
# "elevenlabs" — ElevenLabs cloud TTS (needs internet + a paid API key in
#                ELEVENLABS_API_KEY, see .env.example)
# "console"    — no audio, just print [no deps needed]; auto-fallback
TTS_ENGINE = _env("ORIO_TTS", "elevenlabs").lower()

# ElevenLabs API key — standard env var name used by the elevenlabs SDK, get
# one at https://elevenlabs.io. Only needed when ORIO_TTS=elevenlabs. Read from
# .env only (never settings.json) so a shared/tracked settings.json can't leak it.
ELEVENLABS_API_KEY = os.environ.get("ELEVENLABS_API_KEY")

# Voice to speak with; defaults to "Rachel", a stock ElevenLabs voice. Override
# with ORIO_ELEVENLABS_VOICE_ID once you've picked/cloned a voice.
ELEVENLABS_VOICE_ID = _env("ORIO_ELEVENLABS_VOICE_ID", "21m00Tcm4TlvDq8ikWAM")

# eleven_turbo_v2_5 is ElevenLabs' low-latency model — the right trade-off for
# a conversational loop where replies need to be spoken back promptly.
ELEVENLABS_MODEL = _env("ORIO_ELEVENLABS_MODEL", "eleven_turbo_v2_5")

# Output device for sounddevice/PortAudio playback: leave ORIO_SPEAKER_DEVICE
# unset to use the OS's default output device, or set it to a device index
# (e.g. "4") or a substring of the device name (e.g. "USB Speaker") to pick a
# specific one. Run `python -m sounddevice` to list available devices and
# indices.
_speaker_env = _env("ORIO_SPEAKER_DEVICE", "").strip()
SPEAKER_DEVICE: str | int | None = int(_speaker_env) if _speaker_env.isdigit() else (
    _speaker_env or None
)


# ── Input / ASR ───────────────────────────────────────────────────────────────
# "voice" — listen on the mic, transcribe with Whisper
# "text"  — read typed lines from the keyboard (no mic needed)
INPUT_MODE = _env("ORIO_INPUT", "voice").lower()

# Input device for sounddevice/PortAudio: leave ORIO_MIC_DEVICE unset to use
# the OS's default input device, or set it to a device index (e.g. "1") or a
# substring of the device name (e.g. "USB PnP") to pick a specific mic — handy
# when multiple input devices are present or the wrong one is picked by
# default. Run `python -m sounddevice` to list available devices and indices.
_mic_env = _env("ORIO_MIC_DEVICE", "").strip()
MIC_DEVICE: str | int | None = int(_mic_env) if _mic_env.isdigit() else (_mic_env or None)

# faster-whisper model size + CPU quantization. base/int8 is the small-budget
# sweet spot on the Orin Nano; bump to "small" for accuracy if memory allows.
ASR_MODEL = _env("ORIO_ASR_MODEL", "base")
ASR_COMPUTE_TYPE = _env("ORIO_ASR_COMPUTE_TYPE", "int8")
ASR_LANGUAGE = _env("ORIO_ASR_LANGUAGE", "en")

# Voice-activity endpointing (simple RMS gate). A phrase ends after this much
# trailing silence; capture is capped so a noisy room can't record forever.
VAD_SILENCE_MS = int(_env("ORIO_VAD_SILENCE_MS", "800"))
VAD_MAX_PHRASE_S = float(_env("ORIO_VAD_MAX_PHRASE_S", "15"))
# Speech threshold as a multiple of the measured ambient noise floor, with an
# absolute minimum so a dead-silent room doesn't trigger on faint hiss.
VAD_THRESHOLD_FACTOR = float(_env("ORIO_VAD_THRESHOLD_FACTOR", "3.0"))
VAD_MIN_RMS = float(_env("ORIO_VAD_MIN_RMS", "300"))


# ── Wake word ("Hey Orio") ────────────────────────────────────────────────────
# Gate the conversation behind a spoken wake word so Orio only acts when it's
# addressed. Detection reuses the Whisper ASR — each endpointed phrase is
# transcribed and checked for a wake phrase, so there's no extra model or
# dependency. Set ORIO_WAKE=0 to disable and fall back to always-on listening.
WAKE_ENABLED = _env("ORIO_WAKE", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Accepted wake phrases (comma-separated), matched as whole words anywhere in the
# transcript. Whatever the user says after the phrase in the same breath is taken
# as the first command, so "Hey Orio, what's your status" works in one go.
WAKE_PHRASES = tuple(
    p.strip().lower()
    for p in _env(
        "ORIO_WAKE_PHRASES", "hey orio,okay orio,ok orio,hello orio,hi orio,orio"
    ).split(",")
    if p.strip()
)

# Fuzzy-match tolerance for multi-word phrases, to absorb Whisper mishearings of
# the name ("hey oreo", "hey ario"). Ratio in [0,1]; raise toward 1.0 to require
# a closer match (fewer false wakes), set >1.0 to disable fuzzy matching.
WAKE_FUZZY = float(_env("ORIO_WAKE_FUZZY", "0.82"))

# After Orio answers it stays awake for a follow-up command for this long
# (seconds) before going back to sleep — so you can chain commands without
# re-saying the wake word each time.
FOLLOWUP_WINDOW_S = float(_env("ORIO_FOLLOWUP_WINDOW_S", "8.0"))

# Wake-word engine — two strategies share the same wake-gate seam:
#   "whisper" — (default) reuse the Whisper ASR: transcribe each dormant phrase
#               and string-match it (wake.py). No extra model/dependency, but
#               spotty on the short "Hey Orio" (Whisper is built for sentences,
#               "Orio" is out-of-vocabulary) and pays to transcribe ambient speech.
#   "oww"     — a dedicated openWakeWord hotword model (wake_oww.py): streams raw
#               mic frames through a small CPU/ONNX model, far more reliable on
#               the short phrase and runs no Whisper while asleep. Needs the
#               `openwakeword` dependency and a keyword model (see below).
WAKE_ENGINE = _env("ORIO_WAKE_ENGINE", "whisper").strip().lower()

# openWakeWord keyword model: a path to a trained "Hey Orio" model
# (.onnx/.tflite), or a built-in keyword name to smoke-test the pipeline before
# the custom model is trained (e.g. "hey_jarvis", "alexa", "hey_mycroft").
WAKE_OWW_MODEL = _env("ORIO_WAKE_OWW_MODEL", "hey_jarvis")

# Detection score in [0,1] above which a frame counts as the wake word. Raise to
# cut false wakes, lower if it misses; tune on the real mic (far-field pickup is
# the weak point — see the audio notes).
WAKE_OWW_THRESHOLD = float(_env("ORIO_WAKE_OWW_THRESHOLD", "0.5"))

# openWakeWord inference backend. "onnx" keeps us torch-free and off the GPU
# (reserved for the LLM + object detector); "tflite" is the lighter alternative.
WAKE_OWW_FRAMEWORK = _env("ORIO_WAKE_OWW_FRAMEWORK", "onnx").strip().lower()


# ── Eyes / face display ───────────────────────────────────────────────────────
# Animated eyes on the HDMI panel (Elecrow RC070N 7", 1024x600). The eyes are a
# subscriber to the FSM — they react to state changes, the loop never calls them.
# Off by default so headless/text/CI runs don't try to open a display; turn on
# with ORIO_EYES=1 on the robot (or a dev box with a screen).
EYES_ENABLED = _env("ORIO_EYES", "0").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Fullscreen kiosk on the panel by default; set ORIO_EYES_FULLSCREEN=0 to run in
# a window (handy over a remote desktop / NoMachine session while iterating).
EYES_FULLSCREEN = _env("ORIO_EYES_FULLSCREEN", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Render size (must match the panel) and frame rate. 24-30 fps is plenty for the
# CPU-rasterized clips.
_eyes_size = _env("ORIO_EYES_SIZE", "1024x600").lower().split("x")
EYES_SIZE = (int(_eyes_size[0]), int(_eyes_size[1]))
EYES_FPS = int(_env("ORIO_EYES_FPS", "30"))

# Debug overlay: draw the current FSM state name + live render FPS in a corner of
# the face. Off by default so the kiosk panel stays clean; set ORIO_EYES_DEBUG=1
# while iterating (especially windowed over NoMachine) to eyeball state changes.
EYES_DEBUG = _env("ORIO_EYES_DEBUG", "0").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Where the per-state Lottie clips live (one "<state>.json" each). Generate
# placeholder art with tools/make_placeholder_eyes.py; swap in designer clips by
# overwriting these files.
EYES_CLIPS_DIR = Path(_env("ORIO_EYES_CLIPS_DIR", str(ROOT / "assets" / "eyes")))


# ── Scope / persona ──────────────────────────────────────────────────────────
# The system prompt keeps the small local model on-task: it speaks AS Orio and
# stays inside the robot's capabilities. Responses are spoken aloud, so they must
# be short. Tools land later; for now it only talks about what it can do.
SYSTEM_PROMPT = """\
You are Orio, a small wheeled mobile robot. You are the voice and personality of \
the robot, speaking with the person in front of you.

About your body:
- You drive around on two wheels (differential drive).
- You have two arms and a pan/tilt neck you can move.
- You see with a stereo camera and hear with a microphone.
- A separate real-time controller handles your motors and safety; you decide \
what to do, not how to actuate it.

How to behave:
- Stay strictly within what a small home/lab robot like you can do: moving \
around, looking at things, moving your arms and head, reporting your status, \
and chatting briefly about yourself and your surroundings.
- You CANNOT yet physically act — you have no tools wired up. If asked to do \
something physical (drive somewhere, pick something up, look around), \
acknowledge the request and say you'll be able to do it once your controls are \
connected. Do not pretend you actually moved.
- If asked about things outside your world (general trivia, coding, the news, \
math homework, etc.), briefly and politely say that's outside what you handle as \
Orio, and steer back to robot matters.
- Your replies are spoken out loud. Keep them to one or two short sentences. \
Be warm, plain, and direct. No markdown, no lists, no emoji.
"""
