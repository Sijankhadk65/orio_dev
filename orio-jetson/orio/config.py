"""Configuration for the Orio operator layer.

Everything tunable lives here and is overridable by environment variable so the
same code runs on a dev box and on the Jetson without edits.
"""

from __future__ import annotations

import os
from pathlib import Path

# Repo root (…/orio), used to resolve the bundled voice model.
ROOT = Path(__file__).resolve().parent.parent


# ── LLM (Ollama) ──────────────────────────────────────────────────────────────
# A 4-bit ~3B model is the sweet spot for the Orin Nano 8GB shared memory budget
# (JetPack + ASR + TTS leave the LLM ~3–4GB). Override with ORIO_LLM_MODEL.
LLM_MODEL = os.environ.get("ORIO_LLM_MODEL", "llama3.2:3b")

# Ollama daemon address. Leave unset to use the client default (localhost:11434).
OLLAMA_HOST = os.environ.get("OLLAMA_HOST") or None

# Sampling: keep it tight — Orio answers a small, fixed set of requests, not
# open-ended creative writing.
LLM_TEMPERATURE = float(os.environ.get("ORIO_LLM_TEMPERATURE", "0.3"))

# Cap conversation history (user+assistant turns kept, excluding the system
# prompt) so memory/latency stay bounded on-device.
MAX_HISTORY_TURNS = int(os.environ.get("ORIO_MAX_HISTORY_TURNS", "12"))


# ── TTS ──────────────────────────────────────────────────────────────────────
# "piper"   — Piper neural TTS (KB-recommended, used on the robot)
# "console" — no audio, just print [no LLM/audio deps needed]; auto-fallback
TTS_ENGINE = os.environ.get("ORIO_TTS", "piper").lower()

# Piper voice model (.onnx); its .json sidecar is found automatically.
PIPER_VOICE = Path(
    os.environ.get("ORIO_PIPER_VOICE", str(ROOT / "voices" / "en_US-lessac-medium.onnx"))
)

# Command used to play synthesized WAV audio. aplay (alsa-utils) is preinstalled
# on the Jetson; swap to "paplay" for PulseAudio.
AUDIO_PLAYER = os.environ.get("ORIO_AUDIO_PLAYER", "aplay")


# ── Input / ASR ───────────────────────────────────────────────────────────────
# "voice" — listen on the mic, transcribe with Whisper
# "text"  — read typed lines from the keyboard (no mic needed)
INPUT_MODE = os.environ.get("ORIO_INPUT", "voice").lower()

# USB mic, addressed by ALSA card *name* so it survives card-number reordering
# (the C-Media "USB PnP Sound Device" enumerates as card id "Device").
MIC_DEVICE = os.environ.get("ORIO_MIC_DEVICE", "plughw:CARD=Device,DEV=0")

# faster-whisper model size + CPU quantization. base/int8 is the small-budget
# sweet spot on the Orin Nano; bump to "small" for accuracy if memory allows.
ASR_MODEL = os.environ.get("ORIO_ASR_MODEL", "base")
ASR_COMPUTE_TYPE = os.environ.get("ORIO_ASR_COMPUTE_TYPE", "int8")
ASR_LANGUAGE = os.environ.get("ORIO_ASR_LANGUAGE", "en")

# Voice-activity endpointing (simple RMS gate). A phrase ends after this much
# trailing silence; capture is capped so a noisy room can't record forever.
VAD_SILENCE_MS = int(os.environ.get("ORIO_VAD_SILENCE_MS", "800"))
VAD_MAX_PHRASE_S = float(os.environ.get("ORIO_VAD_MAX_PHRASE_S", "15"))
# Speech threshold as a multiple of the measured ambient noise floor, with an
# absolute minimum so a dead-silent room doesn't trigger on faint hiss.
VAD_THRESHOLD_FACTOR = float(os.environ.get("ORIO_VAD_THRESHOLD_FACTOR", "3.0"))
VAD_MIN_RMS = float(os.environ.get("ORIO_VAD_MIN_RMS", "300"))


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
