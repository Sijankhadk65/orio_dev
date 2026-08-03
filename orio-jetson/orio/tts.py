"""Text-to-speech for the operator layer.

Pluggable behind a tiny `TTS` interface so the engine can be swapped without
touching the conversation loop. ElevenLabs is the cloud TTS engine; the console
engine is a no-audio fallback so the program always runs even before an API key
or audio device is set up.

This layer is out of the real-time/safety path, like the LLM itself.
"""

from __future__ import annotations

from typing import Protocol

from . import config


class TTS(Protocol):
    def speak(self, text: str) -> None: ...


class ConsoleTTS:
    """No audio — just marks that this is where speech would happen."""

    def speak(self, text: str) -> None:
        print(f"  🔇 (tts disabled) {text}")


class ElevenLabsTTS:
    """ElevenLabs cloud TTS → sounddevice playback. Needs an API key + internet."""

    _SAMPLE_RATE = 24000  # matches the pcm_24000 output_format requested below

    def __init__(
        self,
        api_key: str | None = config.ELEVENLABS_API_KEY,
        voice_id: str = config.ELEVENLABS_VOICE_ID,
        model_id: str = config.ELEVENLABS_MODEL,
        device: str | int | None = config.SPEAKER_DEVICE,
    ) -> None:
        if not api_key:
            raise RuntimeError(
                "ELEVENLABS_API_KEY is not set. Get a key from elevenlabs.io and "
                "set it in your environment."
            )
        from elevenlabs.client import ElevenLabs  # imported lazily; heavy + optional

        self._client = ElevenLabs(api_key=api_key)
        self._voice_id = voice_id
        self._model_id = model_id
        self._device = device

    def speak(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        import numpy as np
        import sounddevice as sd

        from .audio_input import resolve_device

        device = resolve_device(self._device, "output")
        try:
            chunks = self._client.text_to_speech.convert(
                voice_id=self._voice_id,
                model_id=self._model_id,
                text=text,
                output_format=f"pcm_{self._SAMPLE_RATE}",
            )
            pcm = b"".join(chunks)
            audio = np.frombuffer(pcm, dtype=np.int16)
            sd.play(audio, samplerate=self._SAMPLE_RATE, device=device)
            sd.wait()
        except Exception as exc:  # network error, bad key, bad output device, etc.
            print(f"\n✗ ElevenLabs playback failed (device {device!r}): {exc}")


def get_tts(engine: str = config.TTS_ENGINE) -> TTS:
    """Build the configured TTS engine, falling back to console on any failure."""
    if engine == "console":
        return ConsoleTTS()
    if engine == "elevenlabs":
        try:
            return ElevenLabsTTS()
        except Exception as exc:  # missing key/deps/network → degrade gracefully
            print(f"  ⚠️  ElevenLabs TTS unavailable ({exc}); falling back to text only.")
            return ConsoleTTS()
    print(f"  ⚠️  Unknown TTS engine '{engine}'; using text only.")
    return ConsoleTTS()
