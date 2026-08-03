"""Cross-platform microphone capture (Linux, Windows, macOS) via `sounddevice`.

Replaces the previous `arecord` subprocess capture used by `asr.py` and
`wake_oww.py`, which only worked on Linux/ALSA. Both need the same thing: a
live 16 kHz mono int16 PCM stream read out in fixed-size frames, blocking
until each frame is ready.
"""

from __future__ import annotations

import queue

import numpy as np
import sounddevice as sd

SAMPLE_RATE = 16000


class MicStream:
    """Blocking, frame-based mic capture. Use as a context manager.

    `sounddevice` delivers audio via a background-thread callback; we buffer
    it in a queue and hand out exactly `n_samples` at a time from
    `read_frame()`, mirroring the old `Popen.stdout.read(n_bytes)` behavior.
    """

    def __init__(self, device: str | int | None, sample_rate: int = SAMPLE_RATE) -> None:
        self._queue: queue.Queue[np.ndarray] = queue.Queue()
        self._buffer = np.empty(0, dtype=np.int16)
        self._stream = sd.InputStream(
            samplerate=sample_rate,
            channels=1,
            dtype="int16",
            device=device,
            callback=self._callback,
        )

    def _callback(self, indata: np.ndarray, frames: int, time_info: object, status: object) -> None:
        self._queue.put(indata[:, 0].copy())

    def __enter__(self) -> "MicStream":
        self._stream.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self._stream.stop()
        self._stream.close()

    def read_frame(self, n_samples: int) -> np.ndarray | None:
        """Block until `n_samples` are available; return None if the stream died."""
        while self._buffer.size < n_samples:
            if not self._stream.active:
                return None
            try:
                chunk = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue
            self._buffer = np.concatenate([self._buffer, chunk])
        frame, self._buffer = self._buffer[:n_samples], self._buffer[n_samples:]
        return frame
