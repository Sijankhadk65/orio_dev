"""Speech-to-text input from the USB mic.

Captures audio from the mic via `sounddevice` (cross-platform, PortAudio-backed),
endpoints a single spoken phrase with a simple RMS voice-activity gate, and
transcribes it with ElevenLabs Scribe (cloud) — the same account already used
for TTS, and considerably more accurate than the local faster-whisper "base"
model this replaced. Like the LLM and TTS, this is out of the real-time path.

Design notes:
- 16 kHz mono S16_LE is captured natively and sent to Scribe as raw PCM
  (file_format="pcm_s16le_16") — no WAV container needed.
- We measure the ambient noise floor at the start of each listen() and set the
  speech threshold relative to it, so it adapts to a quiet vs. noisy room.
- A short pre-roll buffer is kept so the first word isn't clipped.
"""

from __future__ import annotations

import time

import numpy as np

from . import config
from .audio_input import MicStream

SAMPLE_RATE = 16000
FRAME_MS = 30
FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS // 1000  # 480 samples


def _rms(frame: np.ndarray) -> float:
    if frame.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))


class SpeechToText:
    def __init__(
        self,
        model_id: str = config.ASR_MODEL,
        mic_device: str | int | None = config.MIC_DEVICE,
        language: str = config.ASR_LANGUAGE,
    ) -> None:
        from elevenlabs.client import ElevenLabs  # heavy import, kept lazy

        self._client = ElevenLabs(api_key=config.ELEVENLABS_API_KEY)
        self._model_id = model_id
        self._mic = mic_device
        self._language = language

    def _capture_phrase(
        self, mic: MicStream, onset_timeout: float | None = None
    ) -> np.ndarray | None:
        """Endpoint one phrase: wait for speech, record until trailing silence.

        If `onset_timeout` is set and no speech starts within that many seconds
        (measured after noise calibration), return None — used for the post-reply
        follow-up window so Orio stops waiting and goes back to sleep.
        """
        # 1) Calibrate the noise floor from ~0.5s of ambient frames. Seed the
        # pre-roll with them (trimmed to the usual window) rather than
        # discarding them outright — if speech starts *during* calibration
        # (e.g. the command runs straight on from the wake word, no pause),
        # those frames are still the only record of it and shouldn't be lost.
        ambient = [mic.read_frame(FRAME_SAMPLES) for _ in range(16)]
        floor = float(np.median([_rms(f) for f in ambient if f is not None]) or 0.0)
        threshold = max(floor * config.VAD_THRESHOLD_FACTOR, config.VAD_MIN_RMS)

        preroll: list[np.ndarray] = [f for f in ambient if f is not None][-10:]
        voiced: list[np.ndarray] = []
        speaking = False
        silence_ms = 0.0
        start = time.monotonic()

        while True:
            frame = mic.read_frame(FRAME_SAMPLES)
            if frame is None:
                break
            loud = _rms(frame) >= threshold

            if not speaking:
                preroll.append(frame)
                if len(preroll) > 10:
                    preroll.pop(0)
                if loud:
                    speaking = True
                    voiced.extend(preroll)
                    voiced.append(frame)
                elif onset_timeout is not None and time.monotonic() - start > onset_timeout:
                    return None  # no one spoke within the window
                continue

            voiced.append(frame)
            silence_ms = 0.0 if loud else silence_ms + FRAME_MS
            if silence_ms >= config.VAD_SILENCE_MS:
                break
            if time.monotonic() - start > config.VAD_MAX_PHRASE_S:
                break

        if not voiced:
            return None
        return np.concatenate(voiced)

    def listen(self, onset_timeout: float | None = None) -> str | None:
        """Block until a phrase is spoken; return its transcript (or None).

        With `onset_timeout`, give up and return None if no speech begins within
        that many seconds (used for the follow-up window after a reply).
        """
        try:
            with MicStream(self._mic) as mic:
                pcm = self._capture_phrase(mic, onset_timeout=onset_timeout)
        except Exception as exc:
            # A bad/missing device raises from sounddevice/PortAudio directly;
            # surface it instead of letting the caller mistake it for silence.
            raise RuntimeError(f"audio capture failed (device {self._mic!r}): {exc}") from exc

        if pcm is None or pcm.size < SAMPLE_RATE // 2:  # <0.5s → noise, ignore
            return None

        kwargs = {"language_code": self._language} if self._language else {}
        try:
            resp = self._client.speech_to_text.convert(
                model_id=self._model_id,
                file=pcm.tobytes(),
                file_format="pcm_s16le_16",
                **kwargs,
            )
        except Exception as exc:  # network/API hiccup shouldn't kill the voice loop
            print(f"\n⚠ transcription failed: {exc}")
            return None
        text = (getattr(resp, "text", "") or "").strip()
        return text or None
