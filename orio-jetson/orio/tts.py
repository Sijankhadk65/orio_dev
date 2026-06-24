"""Text-to-speech for the operator layer.

Pluggable behind a tiny `TTS` interface so the engine can be swapped without
touching the conversation loop. Piper is the on-robot engine (KB-recommended);
the console engine is a no-audio fallback so the program always runs even before
a voice model or audio device is set up.

This layer is out of the real-time/safety path, like the LLM itself.
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import wave
from pathlib import Path
from typing import Protocol

from . import config


class TTS(Protocol):
    def speak(self, text: str) -> None: ...


class ConsoleTTS:
    """No audio — just marks that this is where speech would happen."""

    def speak(self, text: str) -> None:
        print(f"  🔇 (tts disabled) {text}")


class PiperTTS:
    """Piper neural TTS → WAV → system audio player (aplay/paplay)."""

    def __init__(
        self,
        voice_path: Path = config.PIPER_VOICE,
        player: str = config.AUDIO_PLAYER,
    ) -> None:
        from piper.voice import PiperVoice  # imported lazily; heavy + optional

        if not voice_path.exists():
            raise FileNotFoundError(
                f"Piper voice not found at {voice_path}. Download one with "
                f"`uv run python -m piper.download_voices en_US-lessac-medium "
                f"--download-dir voices`."
            )
        if shutil.which(player) is None:
            raise FileNotFoundError(
                f"Audio player '{player}' not found. Install alsa-utils (aplay) "
                f"or set ORIO_AUDIO_PLAYER."
            )
        self._voice = PiperVoice.load(str(voice_path))
        self._player = player

    def speak(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        # Synthesize to a temp WAV (header carries the sample rate) and play it.
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmp:
            with wave.open(tmp.name, "wb") as wav_file:
                self._voice.synthesize_wav(text, wav_file)
            subprocess.run([self._player, "-q", tmp.name], check=False)


def get_tts(engine: str = config.TTS_ENGINE) -> TTS:
    """Build the configured TTS engine, falling back to console on any failure."""
    if engine == "console":
        return ConsoleTTS()
    if engine == "piper":
        try:
            return PiperTTS()
        except Exception as exc:  # missing voice/player/deps → degrade gracefully
            print(f"  ⚠️  Piper TTS unavailable ({exc}); falling back to text only.")
            return ConsoleTTS()
    print(f"  ⚠️  Unknown TTS engine '{engine}'; using text only.")
    return ConsoleTTS()
