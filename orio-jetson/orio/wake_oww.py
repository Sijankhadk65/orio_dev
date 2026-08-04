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

import numpy as np

from . import config
from .audio_input import MicStream

SAMPLE_RATE = 16000
CHUNK_SAMPLES = 1280  # 80 ms — openWakeWord's expected frame size


class OpenWakeWord:
    """Always-on keyword spotter; blocks until the wake word is heard.

    Mirrors `wake.TranscribeWaker`'s interface (`label`, `await_wake()`) so the
    conversation loop can use either engine interchangeably.
    """

    def __init__(
        self,
        model: str = config.WAKE_OWW_MODEL,
        threshold: float = config.WAKE_OWW_THRESHOLD,
        framework: str = config.WAKE_OWW_FRAMEWORK,
        mic_device: str | int | None = config.MIC_DEVICE,
    ) -> None:
        self._threshold = threshold
        self._mic = mic_device
        self._model_name = model

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
        """Human-readable wake phrase for prompts, derived from the loaded model.

        A bare keyword name (no path separators/extension) is a built-in
        openWakeWord model, whose name doubles as the phrase it listens for
        ("hey_jarvis" -> "Hey Jarvis") — WAKE_PHRASES plays no part in what
        this engine actually reacts to, so it'd be misleading to prompt with
        it. A custom trained model's filename isn't a readable phrase, so
        that case falls back to WAKE_PHRASES as a best-effort label.
        """
        name = self._model_name
        if "/" not in name and "\\" not in name and "." not in name:
            return name.replace("_", " ").title()
        return config.WAKE_PHRASES[0].title() if config.WAKE_PHRASES else "Orio"

    def await_wake(self) -> str | None:
        """Block until the wake word is heard.

        Returns "" on wake (the spotter only flags the event — no trailing
        command), or None on Ctrl-C / capture failure, matching
        `TranscribeWaker.await_wake()` so the loop treats the engines alike.
        """
        self._model.reset()  # clear buffered audio so we don't re-fire on entry
        woke = False
        try:
            with MicStream(self._mic) as mic:
                while True:
                    chunk = mic.read_frame(CHUNK_SAMPLES)
                    if chunk is None:
                        break  # stream ended
                    scores = self._model.predict(chunk)
                    if scores and max(scores.values()) >= self._threshold:
                        woke = True
                        break
        except KeyboardInterrupt:
            return None
        except Exception as exc:
            # A bad/missing device raises from sounddevice/PortAudio directly;
            # surface it like asr.listen() does instead of silently looping.
            print(f"\n✗ audio capture failed (device {self._mic!r}): {exc}")
            return None

        return "" if woke else None
