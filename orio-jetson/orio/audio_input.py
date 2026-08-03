"""Cross-platform microphone capture (Linux, Windows, macOS) via `sounddevice`.

Replaces the previous `arecord` subprocess capture used by `asr.py` and
`wake_oww.py`, which only worked on Linux/ALSA. Both need the same thing: a
live 16 kHz mono int16 PCM stream read out in fixed-size frames, blocking
until each frame is ready.

Linux needs the system PortAudio library (`sudo apt install -y
libportaudio2` on Debian/Ubuntu/Jetson) — the sounddevice wheel bundles it
for Windows/macOS but dynamically loads the system one on Linux, so
`import sounddevice` raises `OSError('PortAudio library not found')` if it's
missing. conversation.py catches that at startup with an actionable message.
"""

from __future__ import annotations

import queue
import sys

import numpy as np
import sounddevice as sd

SAMPLE_RATE = 16000

# device -> resolved device, memoized per (device, kind) so a mic/speaker that
# gets re-opened every listen/speak cycle doesn't re-query the device list
# every time (the device list can't change mid-run in our use case).
_resolved_cache: dict[tuple[object, str], object] = {}


def resolve_device(device: str | int | None, kind: str = "input") -> str | int | None:
    """If no device was explicitly configured, prefer a device literally
    named "pipewire" over PortAudio's raw "default" pick — Linux only.

    Observed on a Jetson: PipeWire held the USB mic open elsewhere, and
    PortAudio's "default" ALSA device silently resolved to a dead stream
    (captured pure silence) while a device literally named "pipewire"
    correctly bridged to the real hardware — the same routing the pre-
    sounddevice `arecord`-based code used deliberately (see git history).
    Windows/macOS never expose a device named "pipewire", so this is a
    no-op there, and any explicit ORIO_MIC_DEVICE/ORIO_SPEAKER_DEVICE still
    wins outright — this only fires when nothing was configured.
    """
    if device is not None or not sys.platform.startswith("linux"):
        return device
    cache_key = (device, kind)
    if cache_key in _resolved_cache:
        return _resolved_cache[cache_key]

    resolved = device
    try:
        channel_key = "max_input_channels" if kind == "input" else "max_output_channels"
        for d in sd.query_devices():
            if d["name"].strip().lower() == "pipewire" and d[channel_key] > 0:
                resolved = d["name"]
                break
    except Exception:
        pass  # fall through to the plain default rather than fail here

    _resolved_cache[cache_key] = resolved
    return resolved


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
            device=resolve_device(device, "input"),
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
