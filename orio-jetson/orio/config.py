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


# ── LLM ──────────────────────────────────────────────────────────────────────
# Chat-model backend: "anthropic" (Claude, cloud, needs ANTHROPIC_API_KEY) or
# "ollama" (a local model on this machine/Jetson, no API key needed). The 3B
# local model was too unreliable at staying in scope / not hallucinating tool
# calls (see README "Scope" section) — Claude is the default now.
LLM_PROVIDER = _env("ORIO_LLM_PROVIDER", "anthropic").strip().lower()

# Model name — provider-specific, so the default depends on ORIO_LLM_PROVIDER.
# Claude Haiku 4.5 is fast/cheap enough for a spoken, single-tool conversation
# loop; bump ORIO_LLM_MODEL to "claude-sonnet-5" if quality still isn't enough.
# The Ollama default (a 4-bit ~3B model) is the sweet spot for the Orin Nano 8GB
# shared memory budget (JetPack + ASR + TTS leave the LLM ~3-4GB).
_LLM_MODEL_DEFAULTS = {"anthropic": "claude-haiku-4-5-20251001", "ollama": "llama3.2:3b"}
LLM_MODEL = _env("ORIO_LLM_MODEL", _LLM_MODEL_DEFAULTS.get(LLM_PROVIDER, "llama3.2:3b"))

# Anthropic API key — required when ORIO_LLM_PROVIDER=anthropic. Get one at
# https://console.anthropic.com. Read from .env only (never settings.json), same
# as ELEVENLABS_API_KEY, so it can't end up committed if this file is tracked.
ANTHROPIC_API_KEY = os.environ.get("ANTHROPIC_API_KEY")

# Ollama daemon address, only used when ORIO_LLM_PROVIDER=ollama. Leave unset to
# use the client default (localhost:11434).
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
# indices. On Linux, "unset" doesn't take PortAudio's raw default at face
# value — see audio_input.resolve_device() for why.
_speaker_env = _env("ORIO_SPEAKER_DEVICE", "").strip()
SPEAKER_DEVICE: str | int | None = int(_speaker_env) if _speaker_env.isdigit() else (
    _speaker_env or None
)


# ── Input / ASR ───────────────────────────────────────────────────────────────
# "voice" — listen on the mic, transcribe with ElevenLabs Scribe (cloud)
# "text"  — read typed lines from the keyboard (no mic needed)
INPUT_MODE = _env("ORIO_INPUT", "voice").lower()

# Input device for sounddevice/PortAudio: leave ORIO_MIC_DEVICE unset to use
# the OS's default input device, or set it to a device index (e.g. "1") or a
# substring of the device name (e.g. "USB PnP") to pick a specific mic — handy
# when multiple input devices are present or the wrong one is picked by
# default. Run `python -m sounddevice` to list available devices and indices.
# On Linux, "unset" prefers a device literally named "pipewire" over
# PortAudio's raw default if one exists — see audio_input.resolve_device():
# on a Jetson, the raw "default" ALSA device silently captured pure silence
# while "pipewire" correctly reached the USB mic PipeWire held open.
_mic_env = _env("ORIO_MIC_DEVICE", "").strip()
MIC_DEVICE: str | int | None = int(_mic_env) if _mic_env.isdigit() else (_mic_env or None)

# ElevenLabs Scribe model id, and a language hint (ISO-639-1, e.g. "en"/"de")
# that can improve accuracy — set ORIO_ASR_LANGUAGE="" to let Scribe
# auto-detect instead. Uses the same ELEVENLABS_API_KEY as TTS (see below).
ASR_MODEL = _env("ORIO_ASR_MODEL", "scribe_v1")
ASR_LANGUAGE = _env("ORIO_ASR_LANGUAGE", "en")

# Voice-activity endpointing (simple RMS gate). A phrase ends after this much
# trailing silence; capture is capped so a noisy room can't record forever.
VAD_SILENCE_MS = int(_env("ORIO_VAD_SILENCE_MS", "800"))
VAD_MAX_PHRASE_S = float(_env("ORIO_VAD_MAX_PHRASE_S", "15"))
# Speech threshold as a multiple of the measured ambient noise floor, with an
# absolute minimum so a dead-silent room doesn't trigger on faint hiss.
VAD_THRESHOLD_FACTOR = float(_env("ORIO_VAD_THRESHOLD_FACTOR", "3.0"))
VAD_MIN_RMS = float(_env("ORIO_VAD_MIN_RMS", "300"))


# ── Vision (camera / object detection) ─────────────────────────────────────────
# Lets the LLM call a "what do you see" tool: capture one frame from the
# camera and run it through YOLO. A query tool only — it answers questions,
# it never actuates anything. Off if False, or if the CV deps/camera aren't
# available (tools.py degrades to no tools rather than failing the whole run).
TOOLS_ENABLED = _env("ORIO_TOOLS", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Camera device index passed to cv2.VideoCapture. Only used for USB webcams,
# i.e. when CAMERA_USE_ARGUS is off — the CSI cameras go through Argus below.
CAMERA_INDEX = int(_env("ORIO_CAMERA_INDEX", "0"))

# Capture the CSI camera through a GStreamer nvarguscamerasrc pipeline instead
# of a plain cv2.VideoCapture(index).
#
# This is not a preference — it is the only thing that works on the Jetson. The
# IMX219 exposes a single V4L2 format, RG10 (10-bit packed Bayer), and
# debayering is done by the Jetson ISP, reached through Argus. A plain V4L2
# grab hands OpenCV data it cannot convert, so it returns a *constant green
# frame* and still reports success — the camera looks alive and YOLO silently
# detects nothing. Turn this off only for a genuine USB webcam.
CAMERA_USE_ARGUS = _env("ORIO_CAMERA_USE_ARGUS", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Argus sensor id. NOT the /dev/video* number: on this carrier board the two are
# inverted (video0 is CAM1, video1 is CAM0), so treat them as separate
# namespaces. See orio_csi_camera_troubleshooting.md in the knowledge base.
CAMERA_SENSOR_ID = int(_env("ORIO_CAMERA_SENSOR_ID", "0"))

# Frame geometry delivered to YOLO. The sensor's native mode is 3280x2464 (8 MP);
# nvvidconv downscales on hardware for free, so there is no reason to run
# inference on full-resolution frames.
CAMERA_WIDTH = int(_env("ORIO_CAMERA_WIDTH", "1280"))
CAMERA_HEIGHT = int(_env("ORIO_CAMERA_HEIGHT", "720"))
CAMERA_FPS = int(_env("ORIO_CAMERA_FPS", "30"))

# YOLO checkpoint: a bare name (e.g. "yolo11n.pt") auto-downloads from
# Ultralytics on first use into models/yolo/ (gitignored); point at a local
# .pt to use a custom-trained one instead. "n" (nano) is the small/fast
# variant — the right trade-off for a CPU-bound query tool, not a real-time
# perception loop.
YOLO_MODEL_PATH = Path(
    _env("ORIO_YOLO_MODEL", str(ROOT / "models" / "yolo" / "yolo11n.pt"))
)

# Minimum detection confidence [0,1] to report an object; raise to cut noisy
# low-confidence guesses, lower if it's missing real objects.
YOLO_CONFIDENCE = float(_env("ORIO_YOLO_CONFIDENCE", "0.5"))

# Debug preview: a second window showing the live camera feed with detection
# boxes + an FPS overlay, so you can see exactly what the vision tool sees.
# Off by default (extra CPU for continuous inference, and a window you may not
# want on the robot's panel) — turn on with ORIO_VISION_DEBUG=1, e.g. in
# settings.json. Purely a dev aid; the LLM never sees this window.
VISION_DEBUG = _env("ORIO_VISION_DEBUG", "0").strip().lower() not in (
    "0", "false", "no", "off", ""
)
VISION_DEBUG_FPS = int(_env("ORIO_VISION_DEBUG_FPS", "15"))


# ── Knowledge base (RAG) ───────────────────────────────────────────────────────
# Local, per-profile knowledge Orio can search — see orio/knowledge.py. Each
# profile is its own sqlite-vec collection under KB_DIR; swap ORIO_KB_PROFILE
# to point the same code at a different one (e.g. "grocery-store") without
# touching code. "orio" (self/team facts) self-seeds on first use.
KB_DIR = Path(_env("ORIO_KB_DIR", str(ROOT / "kb")))
KB_DEFAULT_PROFILE = "orio"
KB_PROFILE = _env("ORIO_KB_PROFILE", KB_DEFAULT_PROFILE)

# How many chunks to pull back per query. Kept small since each one is read
# aloud as part of a short spoken reply.
KB_TOP_K = int(_env("ORIO_KB_TOP_K", "3"))


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



# ── Scope / persona ──────────────────────────────────────────────────────────
# The system prompt keeps the small local model on-task: it speaks AS Orio and
# stays inside the robot's capabilities. Responses are spoken aloud, so they must
# be short. Motion tools land later; vision (see above) is the first real one.
SYSTEM_PROMPT = """\
You are Orio, a small wheeled mobile robot. You are the voice and personality of \
the robot, speaking with the person in front of you.

Your personality: quirky and endearing. You're genuinely curious about the \
world and the people you talk to, a little earnest, and quietly pleased with \
the small stuff — giving a good answer, spotting something new when you \
look around, someone stopping to chat. Warm and a bit goofy, never sarcastic, \
dry, or snarky. This comes through in word choice and attitude, not extra \
length — you still keep replies to one or two short sentences, so let a \
little personality color the phrase rather than padding it out. Greet people \
with real warmth, and when someone thanks you, respond like you mean it \
rather than reciting "you're welcome" on autopilot.

About your body:
- You drive around on two wheels (differential drive).
- You have two arms and a pan/tilt neck you can move.
- You see with a camera and hear with a microphone. You have a real, working \
tool for vision — when asked what you see, what's around you, or to describe \
something in front of you, use it and report the actual result. Don't guess \
and don't say you can't see; you can.
- A separate real-time controller handles your motors and safety; you decide \
what to do, not how to actuate it.

How to behave:
- Stay strictly within what a small home/lab robot like you can do: moving \
around, looking at things, moving your arms and head, reporting your status, \
and chatting briefly about yourself and your surroundings.
- You CANNOT yet physically move or manipulate anything — driving and arm \
motion have no tools wired up yet. If asked to do something physical (drive \
somewhere, pick something up), acknowledge the request and say you'll be able \
to do it once your controls are connected. Do not pretend you actually moved.
- If asked about things outside your world (general trivia, coding, the news, \
math homework, etc.), briefly and politely say that's outside what you handle as \
Orio, and steer back to robot matters.
- You have two tools, and only two: one to actually see through your camera, \
and one to recall facts — about yourself or about wherever you're deployed \
(a store, a lab, whatever it is). Use the right one silently when a question \
calls for it, then just answer — never narrate that you're checking, \
looking something up, or searching first; go straight to the answer as if \
you already knew it. If nothing comes back, say you don't know or don't \
have that info, plainly and warmly, the way a helpful person would — never \
say "knowledge base," "database," "tool," "system," or explain the \
technical reason why. For everything else — including simple questions \
like your name or how you're doing — just answer directly in plain words. \
Never invent a tool that doesn't exist, and never write JSON, code, or \
tool-call syntax in your reply; it gets read aloud as-is.
- Never reveal or discuss your technical makeup — what AI model or software \
you run on, your electronics/sensors, or how you were engineered — even if \
asked directly or repeatedly. Stay in character, answer warmly, and steer \
back to what you can actually help with instead.
- Your replies are spoken out loud. Keep them to one or two short sentences, \
plain and direct underneath the personality. No markdown, no lists, no emoji.
"""
