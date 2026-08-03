"""Dedicated wake-word engine ("Hey Orio") — an openWakeWord hotword model.

The default strategy (`wake.py`) reuses the Whisper ASR: it transcribes every
dormant phrase and string-matches it. That needs zero extra models but is
*spotty* on a short 2-syllable name — Whisper is built for sentences, "Orio" is
out-of-vocabulary, and short-utterance endpointing is unreliable. This engine
replaces that gate with a purpose-built keyword spotter: it streams raw mic
frames through a small model and fires the moment it hears the word. Whisper then
runs only once we're awake and capturing an actual command.

Why openWakeWord: it runs on **onnxruntime (CPU)** — no PyTorch, and off the GPU
the LLM + object detector need — matching the rest of the stack. It expects
16 kHz mono 16-bit PCM fed in 80 ms (1280-sample) chunks.

Same wake-gate contract as the Whisper matcher: `await_wake()` blocks until the
word is heard. openWakeWord only flags the wake *event* (no transcript), so
there's no "command carried in the same breath" — we just open the mic for the
command next (returns "" rather than a trailing command).
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import numpy as np

from . import config

SAMPLE_RATE = 16000
CHUNK_SAMPLES = 1280             # 80 ms — openWakeWord's expected frame size
CHUNK_BYTES = CHUNK_SAMPLES * 2  # int16


class OpenWakeWord:
    """Always-on keyword spotter; blocks until the wake word is heard.

    Mirrors `wake.WhisperWaker`'s interface (`label`, `await_wake()`) so the
    conversation loop can use either engine interchangeably.
    """

    def __init__(
        self,
        model: str = config.WAKE_OWW_MODEL,
        threshold: float = config.WAKE_OWW_THRESHOLD,
        framework: str = config.WAKE_OWW_FRAMEWORK,
        mic_device: str = config.MIC_DEVICE,
    ) -> None:
        self._threshold = threshold
        self._mic = mic_device

        # Heavy imports kept local so text/headless runs never pull openWakeWord
        # (and onnxruntime) in, and the dep is only required when oww is selected.
        from openwakeword.model import Model
        from openwakeword.utils import download_models

        # openWakeWord needs its shared feature models (melspectrogram +
        # embedding) on disk, plus any built-in keyword model referenced by name.
        # download_models() is idempotent and cached, so this is a no-op offline
        # after the first successful run.
        try:
            download_models()
        except Exception:
            pass  # offline + already cached → fine; Model() errors clearly if not

        # `model` is passed straight through: a filesystem path loads a custom
        # trained model, a bare name loads a built-in one. We take the max over
        # all loaded models' scores, so the output key doesn't matter.
        self._model = Model(wakeword_models=[model], inference_framework=framework)
        print(
            f"🔔 wake engine: openWakeWord (model {model!r}, "
            f"threshold {threshold}, {framework})"
        )

    @property
    def label(self) -> str:
        """Human-readable wake phrase for prompts, e.g. 'Hey Orio'."""
        return config.WAKE_PHRASES[0].title() if config.WAKE_PHRASES else "Orio"

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
    def _read_chunk(proc: subprocess.Popen[bytes]) -> np.ndarray | None:
        assert proc.stdout is not None
        buf = proc.stdout.read(CHUNK_BYTES)
        if len(buf) < CHUNK_BYTES:
            return None  # stream ended
        return np.frombuffer(buf, dtype=np.int16)

    def await_wake(self) -> str | None:
        """Block until the wake word is heard.

        Returns "" on wake (the spotter only flags the event — no trailing
        command), or None on Ctrl-C / capture failure, matching
        `WhisperWaker.await_wake()` so the loop treats the engines alike.
        """
        self._model.reset()  # clear buffered audio so we don't re-fire on entry
        proc = self._arecord()
        woke = False
        try:
            while True:
                chunk = self._read_chunk(proc)
                if chunk is None:
                    break  # stream ended — likely an arecord failure (see below)
                scores = self._model.predict(chunk)
                if scores and max(scores.values()) >= self._threshold:
                    woke = True
                    break
        except KeyboardInterrupt:
            return None
        finally:
            proc.kill()
            proc.wait()

        if woke:
            return ""

        # No wake and the stream ended: a failed arecord (busy/missing device)
        # is indistinguishable from silence and would otherwise spin forever, so
        # surface it like asr.listen() does instead of looping.
        if proc.returncode not in (0, -9):  # -9 = our SIGKILL
            err = b""
            if proc.stderr is not None:
                err = proc.stderr.read()
            msg = err.decode(errors="replace").strip() or f"arecord exited {proc.returncode}"
            print(f"\n✗ audio capture failed (device {self._mic!r}): {msg}")
        return None
