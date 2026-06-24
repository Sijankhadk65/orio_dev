"""Speech-to-text input from the USB mic.

Captures audio from the mic via `arecord` (no PortAudio dependency), endpoints a
single spoken phrase with a simple RMS voice-activity gate, and transcribes it
with faster-whisper. Like the LLM and TTS, this is out of the real-time path.

Design notes:
- 16 kHz mono S16_LE is what Whisper wants, so we capture it natively.
- We measure the ambient noise floor at the start of each listen() and set the
  speech threshold relative to it, so it adapts to a quiet vs. noisy room.
- A short pre-roll buffer is kept so the first word isn't clipped.
"""

from __future__ import annotations

import subprocess
import time

import numpy as np

from . import config

SAMPLE_RATE = 16000
FRAME_MS = 30
FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS // 1000  # 480 samples
FRAME_BYTES = FRAME_SAMPLES * 2  # int16


def _rms(frame: np.ndarray) -> float:
    if frame.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(frame.astype(np.float32) ** 2)))


class SpeechToText:
    def __init__(
        self,
        model_name: str = config.ASR_MODEL,
        compute_type: str = config.ASR_COMPUTE_TYPE,
        mic_device: str = config.MIC_DEVICE,
        language: str = config.ASR_LANGUAGE,
    ) -> None:
        from faster_whisper import WhisperModel  # heavy import, kept lazy

        self._model = WhisperModel(model_name, device="cpu", compute_type=compute_type)
        self._mic = mic_device
        self._language = language

    def _arecord(self) -> subprocess.Popen[bytes]:
        return subprocess.Popen(
            [
                "arecord", "-q",
                "-D", self._mic,
                "-f", "S16_LE",
                "-r", str(SAMPLE_RATE),
                "-c", "1",
                "-t", "raw",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    @staticmethod
    def _read_frame(proc: subprocess.Popen[bytes]) -> np.ndarray | None:
        assert proc.stdout is not None
        buf = proc.stdout.read(FRAME_BYTES)
        if len(buf) < FRAME_BYTES:
            return None  # stream ended
        return np.frombuffer(buf, dtype=np.int16)

    def _capture_phrase(self, proc: subprocess.Popen[bytes]) -> np.ndarray | None:
        """Endpoint one phrase: wait for speech, record until trailing silence."""
        # 1) Calibrate the noise floor from ~0.5s of ambient frames.
        ambient = [self._read_frame(proc) for _ in range(16)]
        floor = float(np.median([_rms(f) for f in ambient if f is not None]) or 0.0)
        threshold = max(floor * config.VAD_THRESHOLD_FACTOR, config.VAD_MIN_RMS)

        preroll: list[np.ndarray] = []  # ~300ms kept so onset isn't clipped
        voiced: list[np.ndarray] = []
        speaking = False
        silence_ms = 0.0
        start = time.monotonic()

        while True:
            frame = self._read_frame(proc)
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

    def listen(self) -> str | None:
        """Block until a phrase is spoken; return its transcript (or None)."""
        proc = self._arecord()
        try:
            pcm = self._capture_phrase(proc)
        finally:
            proc.kill()
            proc.wait()

        if pcm is None or pcm.size < SAMPLE_RATE // 2:  # <0.5s → noise, ignore
            return None

        audio = pcm.astype(np.float32) / 32768.0
        segments, _ = self._model.transcribe(audio, language=self._language)
        text = "".join(seg.text for seg in segments).strip()
        return text or None
