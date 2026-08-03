"""Speech-to-text input from the USB mic.

Captures audio from the mic via `sounddevice` (cross-platform, PortAudio-backed),
endpoints a single spoken phrase with a simple RMS voice-activity gate, and
transcribes it with faster-whisper. Like the LLM and TTS, this is out of the
real-time path.

Design notes:
- 16 kHz mono S16_LE is what Whisper wants, so we capture it natively.
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
        model_name: str = config.ASR_MODEL,
        compute_type: str = config.ASR_COMPUTE_TYPE,
        mic_device: str | int | None = config.MIC_DEVICE,
        language: str = config.ASR_LANGUAGE,
    ) -> None:
        from faster_whisper import WhisperModel  # heavy import, kept lazy

        self._model = WhisperModel(model_name, device="cpu", compute_type=compute_type)
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
        # 1) Calibrate the noise floor from ~0.5s of ambient frames.
        ambient = [mic.read_frame(FRAME_SAMPLES) for _ in range(16)]
        floor = float(np.median([_rms(f) for f in ambient if f is not None]) or 0.0)
        threshold = max(floor * config.VAD_THRESHOLD_FACTOR, config.VAD_MIN_RMS)

        preroll: list[np.ndarray] = []  # ~300ms kept so onset isn't clipped
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

        audio = pcm.astype(np.float32) / 32768.0
        segments, _ = self._model.transcribe(audio, language=self._language)
        text = "".join(seg.text for seg in segments).strip()
        return text or None
