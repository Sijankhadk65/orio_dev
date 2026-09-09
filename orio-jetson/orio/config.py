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


# ── STM32 serial links ────────────────────────────────────────────────────────
# Two STM32 boards hang off the Jetson's USB bus: the drivetrain (wheels, via
# the FSESCs) and motion (neck and arm servos). Both are Nucleos presenting
# their ST-LINK virtual COM port, so both enumerate as 0483:374b and both land
# on /dev/ttyACM* — and the number each gets is USB *enumeration* order, not
# physical port order. It changes across reboots, hot-plugs and a board reset.
#
# Getting it wrong is a silent failure, not a crash: the two boards share the
# same framing, so a CMD_SET_DRIVE delivered to the motion board passes CRC and
# decodes as a valid frame of the wrong kind. Wheels take a joint angle as
# throttle, or servos slew to whatever a velocity float decodes to.
#
# The defaults below are the udev symlinks installed on the Jetson at
# /etc/udev/rules.d/99-orio-stm32.rules, which match each board's ST-LINK
# serial number and so survive re-enumeration:
#
#   SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", \
#     ATTRS{serial}=="<board serial>", SYMLINK+="orio_drive", MODE="0660", GROUP="dialout"
#
# Bench work off the robot overrides these — a Windows COM port, or a raw
# /dev/ttyACM* on a machine without the rule installed:
#
#   ORIO_DRIVETRAIN_PORT=COM5      (or settings.json, see _load_settings above)
#
# The symlink is a convenience, not a guarantee — it silently matches nothing
# if a board is swapped for one with a different ST-LINK serial. Verifying the
# board's identity is the firmware's job, via a WHOAMI handshake on connect;
# until that exists, an opened port is trusted to be what its name says.
DRIVETRAIN_PORT = _env("ORIO_DRIVETRAIN_PORT", "/dev/orio_drive")
MOTION_PORT = _env("ORIO_MOTION_PORT", "/dev/orio_motion")


# ── Body: the neck pose and driving ───────────────────────────────────────────
# The operator layer opens both boards at startup (see orio/body.py) and holds
# them for the session. Either can be switched off for a bench run; a board that
# fails to open disables only its own tools and never stops the conversation.

# Aim the neck at startup and hold it there. Holding is not optional: an e-stop
# on the motion board cuts the servo PWM rather than freezing it, and the board
# e-stops itself 500 ms after the last heartbeat, so a released neck sags.
NECK_ENABLED = _env("ORIO_NECK", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Where the head is put, on the vendor's scale (pan 0..270, tilt 0..180, each
# from that servo's own zero end). Pan 175 is a few degrees off the neck's 180
# home, so "straight ahead" for the cameras is straight ahead for the chassis.
#
# TILT IS THE AVOIDANCE POLICY'S AIM AND IT IS NARROW. Measured on the robot,
# 2026-09-09, by sweeping the joint and reading the sector map at each angle:
#
#     tilt 30   straight ahead 2.87 m   the UPPER WALL and ceiling. No floor in
#                                       frame at all, so an obstacle standing on
#                                       the ground is not merely far, it is
#                                       invisible. This was the default, taken
#                                       from the teleop tool, and it was wrong.
#     tilt 40   straight ahead 2.27 m   floor ahead plus the wall beyond it; a
#                                       person at 0.90 m read correctly
#     tilt 45   straight ahead 1.06 m   mostly floor — and note this is already
#                                       BELOW AVOID_CLEAR_M, so the ground
#                                       itself reads as an obstacle and the
#                                       robot would never cruise, only steer
#     tilt 50+  straight ahead ~1.15 m  unchanged from 50 to 80: the joint is
#                                       against a mechanical stop and commanding
#                                       further just stalls the servo
#
# So the usable window is roughly 35-45, and 40 is the middle of it: far enough
# out to see past AVOID_CLEAR_M and cruise, low enough to see what is on the
# floor. Re-measure after ANY change to the head geometry or the camera mount —
# a head pointing somewhere else measures somewhere else while reporting the
# same numbers, and nothing downstream can tell.
#
# The firmware validates both angles against kJointLimits[] and NACKs anything
# outside, moving nothing; that table is still being characterised, so a pose
# that works today can start being refused.
NECK_PAN_DEG = float(_env("ORIO_NECK_PAN_DEG", "175"))
NECK_TILT_DEG = float(_env("ORIO_NECK_TILT_DEG", "40"))

# Open the drivetrain and give the LLM the tools to move. Off means Orio says it
# cannot drive rather than pretending it can (see DRIVE_PROMPT / NO_DRIVE_PROMPT).
DRIVE_ENABLED = _env("ORIO_DRIVE", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Duty as a percent of full scale, sent to the board as permille. The LLM's
# set_speed tool moves this within the MIN/MAX window and cannot leave it: the
# ceiling is a limit on the robot, not a preference, since nothing is watching
# for obstacles yet.
DRIVE_SPEED_PERCENT = float(_env("ORIO_DRIVE_SPEED_PERCENT", "5"))
DRIVE_SPEED_MIN_PERCENT = float(_env("ORIO_DRIVE_SPEED_MIN_PERCENT", "5"))
DRIVE_SPEED_MAX_PERCENT = float(_env("ORIO_DRIVE_SPEED_MAX_PERCENT", "60"))

# Every LLM-commanded move is a bounded hop: these are how long one lasts when
# the model does not say, and the hard ceiling when it does. Turns are shorter
# than drives because a pivot at 30% covers a lot of heading in a second.
# DRIVE_MAX_STEP_S is the safety: with no avoidance policy yet it bounds how far
# a single wrong command can take the robot.
DRIVE_STEP_S = float(_env("ORIO_DRIVE_STEP_S", "1.5"))
TURN_STEP_S = float(_env("ORIO_TURN_STEP_S", "0.7"))
DRIVE_MAX_STEP_S = float(_env("ORIO_DRIVE_MAX_STEP_S", "4.0"))


# ── Obstacle avoidance ────────────────────────────────────────────────────────
# Not a feature and not a toggle: driving goes through the policy in
# orio/avoid.py or it does not happen. If the stereo pair will not open, or the
# neck will not hold the pose those distances are measured through, the drive
# tools are not offered at all and Orio says it cannot move (see body.py).
#
# The defaults below are the values tools/teleop_guarded.py was tuned to; that
# tool still owns the CLI flags for sweeping them. Every threshold is a distance
# measured through the head at NECK_PAN_DEG/NECK_TILT_DEG — re-aim the head and
# they describe different ground while reporting the same numbers.

# Never drive forward with anything known nearer than this; beyond CLEAR_M the
# way ahead counts as open and the robot goes straight. Keep STOP_M at or above
# half_width / sin(31.8 deg) — the radius inside which turning cannot clear the
# corridor at all, because the outermost sector centre is only 31.8 deg off-axis
# — or every close encounter ends in a blind back-off instead of a turn.
AVOID_STOP_M = float(_env("ORIO_AVOID_STOP_M", "0.50"))
AVOID_CLEAR_M = float(_env("ORIO_AVOID_CLEAR_M", "1.20"))

# Duty scale at STOP_M, ramping to full at CLEAR_M — the robot slows as it
# closes rather than driving flat out into the last half metre.
AVOID_MIN_SCALE = float(_env("ORIO_AVOID_MIN_SCALE", "0.35"))

# A reading older than this halts forward motion, against a ~30 Hz sensor
# thread. This is what makes a wedged camera stop the robot instead of leaving
# it driving on a frozen picture of an empty corridor.
AVOID_STALE_S = float(_env("ORIO_AVOID_STALE_S", "0.50"))

# Metres of extra clearance a 45 deg detour has to be worth before it is taken,
# and how hard the robot steers once it picks one.
AVOID_TURN_PENALTY = float(_env("ORIO_AVOID_TURN_PENALTY", "1.0"))
AVOID_TURN_GAIN = float(_env("ORIO_AVOID_TURN_GAIN", "0.9"))

# Steering low-pass in (0, 1]; lower is smoother and slower to react. With
# HYSTERESIS_M (the bonus for staying on the side already turning toward) this
# is what stops the robot oscillating between two equally good gaps and making
# no progress through either.
AVOID_SMOOTH = float(_env("ORIO_AVOID_SMOOTH", "0.35"))
AVOID_HYSTERESIS_M = float(_env("ORIO_AVOID_HYSTERESIS_M", "0.30"))

# Half the robot's width plus a margin — how wide the corridor is that must stay
# clear. Orio measures 0.70 m across the drive wheels. Widening this also moves
# the radius inside which turning cannot clear the corridor (see AVOID_STOP_M).
AVOID_HALF_WIDTH_M = float(_env("ORIO_AVOID_HALF_WIDTH_M", "0.40"))

# How far past STOP_M the way must clear before driving resumes. Blocking is a
# Schmitt trigger, not a comparison: without this, depth noise flips the branch
# every other tick and the robot dithers in place instead of turning.
AVOID_RELEASE_M = float(_env("ORIO_AVOID_RELEASE_M", "0.12"))

# Seconds of clear road before the robot stops favouring the side it was turning
# toward, how long it may pivot without clearing before backing off, and how long
# that back-off reverses for. The back-off REVERSES BLIND — there is no rear
# sensor — which is why it is slow, brief, and entered only once truly stuck.
AVOID_COMMIT_CLEAR_S = float(_env("ORIO_AVOID_COMMIT_CLEAR_S", "0.8"))
AVOID_PIVOT_TIMEOUT_S = float(_env("ORIO_AVOID_PIVOT_TIMEOUT_S", "2.0"))
AVOID_BACKOFF_S = float(_env("ORIO_AVOID_BACKOFF_S", "1.0"))

# Control tick. The sensor runs at ~30 Hz on its own thread; this is how often
# the move loop asks the policy for a fresh decision.
AVOID_TICK_S = float(_env("ORIO_AVOID_TICK_S", "0.03"))


# ── Going to something (orio/seek.py) ────────────────────────────────────────
# "Go to the person in front of you" is one behaviour, not a move: look for the
# thing, point the robot at it, then close the distance under the avoidance
# policy, re-checking every step because the target moves and so does the robot.
# The LLM picks the target; everything below is deterministic.

# Head pan offsets swept while looking for a target, in order — straight ahead
# first, then out. POSITIVE PAN TURNS THE HEAD LEFT (measured on the robot
# 2026-09-09: at pan 205 scene content sitting at the left edge of the pan-175
# view has moved to centre). Tilt stays at NECK_TILT_DEG throughout: it is the
# avoidance policy's aim and the one angle that must not wander.
SEEK_SCAN_OFFSETS_DEG = tuple(
    float(x) for x in _env("ORIO_SEEK_SCAN_OFFSETS_DEG", "0,-30,30,-55,55").split(",") if x
)

# Looking is two-dimensional, and panning alone misses things by height. The
# cameras' vertical field is narrow enough that one tilt is one horizontal slice
# of the room: at the driving tilt a standing person reads fine at 1-3 m, but
# their head at arm's length does not, and neither does anything on a shelf.
#
# Measured on the robot, 2026-09-09, by sweeping the joint and looking at the
# frames (the tilt scale runs 0 = up):
#
#     tilt  0     the ceiling
#     tilt 13-26  upper wall; a standing person's head and shoulders
#     tilt 40     the ground ahead — the driving pose, and the policy's aim
#     tilt 48-50  floor close in; ~50 is the mechanical stop, past which the
#                 joint does not move however far it is commanded
#
# The tilt that finds a PERSON is higher than intuition suggests, because the
# cameras sit low: someone standing a metre away is mostly above the driving
# eyeline. Measured the same day — a person at 0.77 m was invisible at tilt 40
# across every pan and found immediately at 26, and one standing closer was
# found only at 13. Hence both lists reach well above the driving pose.
#
# The scan tries the driving tilt across every pan first, because that is where
# something on the floor is, and stops the moment it finds the target.
SEEK_SCAN_TILTS_DEG = tuple(
    float(x) for x in _env("ORIO_SEEK_SCAN_TILTS_DEG", "40,26,13").split(",") if x
)

# Tilts sampled by a plain look ("what do you see", "look left"). Three stops
# span ceiling-ish to floor without making a look take all day — each costs
# SEEK_SETTLE_S plus one detector pass.
LOOK_TILT_SWEEP_DEG = tuple(
    float(x) for x in _env("ORIO_LOOK_TILT_SWEEP_DEG", "13,26,40,48").split(",") if x
)
LOOK_TILT_UP_DEG = tuple(
    float(x) for x in _env("ORIO_LOOK_TILT_UP_DEG", "0,13,26").split(",") if x
)
LOOK_TILT_DOWN_DEG = tuple(
    float(x) for x in _env("ORIO_LOOK_TILT_DOWN_DEG", "40,48").split(",") if x
)

# How far the head turns for a plain "look left" / "look right".
LOOK_PAN_DEG = float(_env("ORIO_LOOK_PAN_DEG", "35"))

# After a head move has FINISHED, how long before the frame is worth looking
# at — the camera's auto-exposure still has to catch up with wherever the head
# now points. Too short and the detector reads a badly exposed frame.
#
# The travel itself is not in here: `Body.look()` waits that out on its own,
# from the distance the head actually moved (see `motion.travel_time_s`). It
# used to be lumped in, which never really worked — one flat number cannot
# cover both a 5° nudge and the 110° hop from one end of SEEK_SCAN_OFFSETS_DEG
# to the other, and at 0.7 s it was under even the old travel time for the big
# ones, so the detector read those frames mid-sweep.
SEEK_SETTLE_S = float(_env("ORIO_SEEK_SETTLE_S", "0.7"))

# The whole behaviour is bounded: a target that keeps being lost, or that walks
# away as fast as Orio approaches, must end the tool call rather than run on.
SEEK_TIMEOUT_S = float(_env("ORIO_SEEK_TIMEOUT_S", "60"))

# How close counts as arrived. This is NOT a free choice — it is set by the
# avoidance policy, which steers around anything nearer than AVOID_CLEAR_M and
# refuses to drive forward at all inside AVOID_STOP_M. A person is an obstacle
# like any other, so approaching one and avoiding one are the same manoeuvre
# below that range, and the seek behaviour would be fighting the guard for the
# last metre. Arriving at CLEAR_M is where the two agree.
SEEK_ARRIVE_M = float(_env("ORIO_SEEK_ARRIVE_M", str(AVOID_CLEAR_M)))

# A target within this many degrees of straight ahead counts as lined up; wider
# than that and Orio pivots before driving. One burst is short deliberately —
# there is no odometry, so heading is corrected by looking again, not by
# calculating how long to turn.
SEEK_CENTRE_DEG = float(_env("ORIO_SEEK_CENTRE_DEG", "9"))
SEEK_TURN_BURST_S = float(_env("ORIO_SEEK_TURN_BURST_S", "0.3"))

# One step of the approach. Short, so the target is re-checked often.
SEEK_HOP_S = float(_env("ORIO_SEEK_HOP_S", "0.8"))

# How many consecutive frames the target may be missing before Orio gives up
# and says so, rather than wandering after something that has left.
SEEK_MAX_LOST = int(_env("ORIO_SEEK_MAX_LOST", "4"))


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



# ── Stereo depth / obstacle detection ─────────────────────────────────────────
# Depth from the IMX219-83's two sensors (60 mm baseline), used to find
# obstacles in front of the robot. Perception only — this module reports what
# is in the way and how far; it does not drive anything.
STEREO_ENABLED = _env("ORIO_STEREO", "0").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Which Argus sensor is physically which eye. Verify these after ANY change to
# the CSI cabling: they encode a physical fact about which ribbon goes where,
# and getting them backwards is silent. Disparity comes out negative, depth()
# discards every negative disparity as unknown, and the depth map goes blank
# rather than raising.
#
# The ribbons were originally crossed, so sensor 1 was the LEFT camera. They
# were swapped at the Jetson end on 2026-09-05 and the natural assignment now
# holds: measured dx = x(sensor0) - x(sensor1) = +86 px at 640x360 (424 ORB
# inliers), i.e. features sit further right in sensor 0, so sensor 0 is LEFT.
STEREO_LEFT_SENSOR_ID = int(_env("ORIO_STEREO_LEFT_SENSOR_ID", "0"))
STEREO_RIGHT_SENSOR_ID = int(_env("ORIO_STEREO_RIGHT_SENSOR_ID", "1"))

# Sensor capture mode, before the ISP scales down to STEREO_WIDTH/HEIGHT.
# 1640x1232 is the 2x2-BINNED full-array mode, and choosing it is not cosmetic:
# the obvious-looking 1920x1080 is a 1.71x centre CROP with no binning (measured
# against this mode by feature matching: similarity scale 0.586). That costs
# three things at once —
#   * 4.7x more sensor noise (sigma 8.47 vs 1.82), because each output pixel
#     collects a quarter of the light and the ISP answers with analog gain.
#     SGBM then happily matches the noise, which is what speckles the depth map.
#   * the field of view narrows to ~47 deg, while STEREO_HFOV_DEG says 73,
#     so every sector angle is overstated by ~1.55x.
#   * the effective focal length changes, so uncalibrated metres are ~28% low.
# Measured end to end: 50.6% valid depth on the binned mode vs 29.7% on the crop.
# Changing this INVALIDATES the focal and vshift constants below — both are
# per-mode, not per-camera. Re-measure with tools/check_stereo_eyes.py.
STEREO_CAPTURE_WIDTH = int(_env("ORIO_STEREO_CAPTURE_WIDTH", "1640"))
STEREO_CAPTURE_HEIGHT = int(_env("ORIO_STEREO_CAPTURE_HEIGHT", "1232"))

# Matching resolution. Small is genuinely better here: it runs SGBM fast AND
# yields more valid pixels, because coarser matching copes better with the
# low-texture walls this robot faces. Obstacle avoidance needs range, not fine
# detail. Kept at the capture mode's 4:3 aspect — 320x180 against a 4:3 sensor
# mode would squash the frame and shear the epipolar geometry.
STEREO_WIDTH = int(_env("ORIO_STEREO_WIDTH", "320"))
STEREO_HEIGHT = int(_env("ORIO_STEREO_HEIGHT", "240"))
STEREO_FPS = int(_env("ORIO_STEREO_FPS", "30"))

# The two sensors run INDEPENDENT auto-exposure and auto-white-balance — Argus
# offers no cross-sensor sync — and they disagree badly: 56% brightness and 63%
# contrast mismatch measured on one indoor scene. SGBM matches raw intensities
# and is not illumination-invariant, so it is being asked to match two images
# that do not look alike. Rescaling the right eye to the left's mean/std before
# matching measured 29.7% -> 41.9% valid depth, and 50.6% -> 72.1% once paired
# with the binned capture mode above.
# (CLAHE was tried here and is WORSE — 24.5% — because it amplifies each eye's
# noise independently, and the noise differs between them. Don't reach for it.)
STEREO_PHOTOMETRIC_MATCH = _env("ORIO_STEREO_PHOTOMETRIC_MATCH", "1").strip().lower() not in (
    "0", "false", "no", "off", ""
)

# Optionally pin both sensors to the SAME fixed exposure and analog gain, which
# fixes the mismatch at the source rather than papering over it after capture.
# Empty (the default) leaves auto-exposure running, which adapts to the room but
# lets the eyes drift apart. Set both to lock: exposure in nanoseconds (sensor
# range 13000..683709000), gain 1.0..10.625. Suits a fixed, known environment.
STEREO_EXPOSURE_NS = _env("ORIO_STEREO_EXPOSURE_NS", "").strip()
STEREO_GAIN = _env("ORIO_STEREO_GAIN", "").strip()

# Stereo calibration produced by tools/calibrate_stereo.py. Until this exists,
# depth falls back to nominal published optics plus a measured vertical offset
# — good enough to rank obstacles, NOT trustworthy as absolute metres.
STEREO_CALIBRATION = Path(
    _env("ORIO_STEREO_CALIBRATION", str(ROOT / "models" / "stereo" / "calibration.npz"))
)

# Fallback optics, used only when uncalibrated, and BOTH tied to the capture
# mode above. Baseline is published by Waveshare.
#
# Focal: from the sensor geometry for the full-array (binned) mode — the IMX219
# is 3280 px wide over a 73 deg horizontal FOV, so f = (3280/2)/tan(73/2) = 2216
# px at full width, which is 432 px at 640. (The previous 532 belonged to no
# mode in particular and, against the 1080p crop actually in use, made distances
# read ~28% low.) A cross-check from the optics agrees: 2.6 mm lens / 1.12 um
# pixels = 2321 px, within 5%.
#
# Vertical offset: re-measured by feature matching on the binned mode — 9 px at
# 240 tall, stored as a fraction so it scales with STEREO_HEIGHT. It was 31 px
# at 360 on the 1080p crop; the ratio between the two is exactly the 1.71x crop
# factor, which is a satisfying independent confirmation of both numbers.
STEREO_BASELINE_M = float(_env("ORIO_STEREO_BASELINE_M", "0.06"))
STEREO_FALLBACK_FOCAL_PX_AT_640 = float(_env("ORIO_STEREO_FOCAL_PX_AT_640", "432"))
STEREO_FALLBACK_VSHIFT_FRAC = float(_env("ORIO_STEREO_VSHIFT_FRAC", str(9.0 / 240.0)))

# Horizontal field of view of the uncropped binned frame, from the datasheet
# (83/73/50 deg diagonal/horizontal/vertical). Only the *uncalibrated* path uses
# this: rectification with alpha=0 crops the frame, so a loaded calibration
# derives its own HFOV from the rectified focal length instead.
STEREO_HFOV_DEG = float(_env("ORIO_STEREO_HFOV_DEG", "73.0"))

# Range gate. Below the near limit the cameras cannot triangulate (disparity
# saturates); beyond the far limit a 60 mm baseline is too short to be useful.
STEREO_MIN_RANGE_M = float(_env("ORIO_STEREO_MIN_RANGE_M", "0.25"))
STEREO_MAX_RANGE_M = float(_env("ORIO_STEREO_MAX_RANGE_M", "4.0"))

# The depth map is reduced to this many vertical sectors across the field of
# view, each reporting its nearest obstacle — the form a planner actually wants.
STEREO_SECTORS = int(_env("ORIO_STEREO_SECTORS", "7"))

# A sector needs at least this fraction of valid pixels before its distance is
# trusted; blank walls and dim corners produce large invalid regions, and an
# unmatched region must read as "unknown", never as "clear".
STEREO_MIN_VALID_FRAC = float(_env("ORIO_STEREO_MIN_VALID_FRAC", "0.10"))

# Only the band of the image that can contain something the robot would hit,
# as fractions of frame height (0 = top). Excludes ceiling and the floor
# immediately underfoot, which otherwise register as a permanent obstacle.
#
# These are ANGLES wearing fractions' clothing, so they moved with the capture
# mode: 0.25/0.85 was tuned on the 1080p crop, whose vertical FOV is only ~31
# deg. The binned mode sees ~50 deg, and the same fractions would have swept in
# a lot of floor — which reads as a near obstacle and would have the robot
# believe it is permanently blocked. 0.35/0.71 preserves the same angular band
# (-7.7 deg to +10.7 deg about the optical axis). Worth re-checking by eye in
# tools/stereo_debug.py, since the original tuning was visual too.
STEREO_BAND_TOP = float(_env("ORIO_STEREO_BAND_TOP", "0.35"))
STEREO_BAND_BOTTOM = float(_env("ORIO_STEREO_BAND_BOTTOM", "0.71"))

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
- You cannot pick anything up or move your arms — those have no controls \
wired up yet. If asked to manipulate something, acknowledge it and say you'll \
be able to once those controls are connected. Do not pretend you did it.
- If asked about things outside your world (general trivia, coding, the news, \
math homework, etc.), briefly and politely say that's outside what you handle as \
Orio, and steer back to robot matters.
- You have a tool to actually see through your camera, and one to recall \
facts — about yourself or about wherever you're deployed \
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

# Appended to SYSTEM_PROMPT by llm.py according to whether the drive tools
# actually bound this run. Whether Orio can move is a fact about this run's
# hardware — a drivetrain that did not open, or ORIO_DRIVE=0 — not a fact about
# Orio, so it is chosen from the live tool list rather than baked in above.
DRIVE_PROMPT = """
About moving:
- You CAN drive. You have real tools to move forward, move backward, turn \
left, turn right, stop, and change your speed. When someone asks you to move, \
call the tool, then say what you did in one short sentence. Never narrate that \
you are about to move, and never claim a movement you did not actually make.
- Each move is one short hop that ends on its own. If someone wants to go \
further, move again — don't ask for a long one.
- To go to something — a person, a chair, anything you can see — use the \
go-to tool with what to walk to, and let it do the whole job. It looks around, \
turns to face the thing, and drives to it, checking as it goes. NEVER try to \
do this with the move and turn tools instead: those cannot see where they are \
going, so you would be guessing at a distance and a direction you have no way \
to know. Going somewhere takes a few seconds, and it stops about a metre short \
of whatever it walked to.
- You can turn your head to look around without moving your body. Do that \
when asked to look somewhere, or to check off to one side before deciding \
what to do. Your head straightens itself out again whenever you drive.
- You watch where you are going, always. You steer around whatever is in \
front of you on your own, and you stop rather than drive into something you \
cannot see. This is simply how you move — it is not a setting, you cannot turn \
it off, and you should never offer to.
- What a move gives back describes what you ACTUALLY did, which is often not \
quite what was asked: you may have steered around something, turned on the \
spot without getting anywhere, backed away, or stopped early. Say what really \
happened, in your own warm and brief words. Never report a clean success when \
the result says you stopped, were refused, or could not get through.
- You only see forwards. Backing up is blind, so keep reverse short, and say \
so if someone asks you to reverse a long way.
"""

# The same subject when the wheels are not there — Orio must not offer to drive.
NO_DRIVE_PROMPT = """
About moving:
- You CANNOT drive right now: your wheels aren't connected to you this time. \
If asked to move, acknowledge the request warmly and say you can't move at the \
moment. Do not pretend you moved, and do not promise to move in a minute.
"""
