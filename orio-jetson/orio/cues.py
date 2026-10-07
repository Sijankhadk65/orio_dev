"""Short spoken cues — the "Hmm?" Orio makes when it starts listening.

The eyes already show LISTENING, but someone not looking at the face has no way
to know the wake word landed. A cue in Orio's own voice fixes that, so it is
synthesized with the same ElevenLabs voice as the replies rather than shipped
as a stock sound.

Each phrase is synthesized once and cached as a WAV under `config.CUE_CACHE_DIR`,
keyed by voice and phrase, so a wake costs a local playback, not a network round
trip, and changing the voice regenerates the clips instead of mixing two voices.
`prewarm()` fills the cache at startup so the first wake is not the slow one.

The cue plays to completion BEFORE the mic opens: asr.listen() calibrates its
noise floor on the first ~0.5 s and opens on the first loud frame, so a cue
playing into an open mic would either raise the floor or be taken for speech.
"""

from __future__ import annotations

import hashlib
import random
import wave
from pathlib import Path

import numpy as np

from . import config

# Synthesized clips start and end with near-silence; trimmed below this level
# (int16 amplitude) so the cue is as short as the sound itself.
_TRIM_LEVEL = 300
# Kept either side of the trimmed sound so its attack and tail aren't clipped.
_TRIM_PAD_S = 0.03


def _trim(audio: np.ndarray, rate: int) -> np.ndarray:
    loud = np.flatnonzero(np.abs(audio.astype(np.int32)) >= _TRIM_LEVEL)
    if loud.size == 0:
        return audio
    pad = int(rate * _TRIM_PAD_S)
    return audio[max(loud[0] - pad, 0) : loud[-1] + pad]


class ListenCue:
    """Plays one of `phrases` at random, in the TTS voice. Silent without one."""

    def __init__(
        self,
        tts,
        phrases: tuple[str, ...] = config.LISTEN_CUE_PHRASES,
        cache_dir: Path = config.CUE_CACHE_DIR,
    ) -> None:
        # Only an engine that can render audio can give a cue; the console
        # engine has nothing to play, so the cue is quietly off with it.
        self._tts = tts if hasattr(tts, "synthesize") else None
        self._phrases = phrases
        self._dir = cache_dir
        self._clips: dict[str, np.ndarray] = {}

    @property
    def enabled(self) -> bool:
        return self._tts is not None and bool(self._phrases)

    def _path(self, phrase: str) -> Path:
        key = hashlib.sha1(f"{self._tts.voice_id}\0{phrase}".encode()).hexdigest()[:16]
        return self._dir / f"{key}.wav"

    def _clip(self, phrase: str) -> np.ndarray | None:
        if phrase in self._clips:
            return self._clips[phrase]
        rate = self._tts.SAMPLE_RATE
        path = self._path(phrase)
        audio = None
        try:
            with wave.open(str(path), "rb") as w:
                if w.getframerate() == rate:
                    audio = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        except (FileNotFoundError, wave.Error, EOFError):
            pass
        if audio is None:
            audio = self._tts.synthesize(phrase)
            if audio is None:
                return None  # offline: no cue this time, try again next wake
            audio = _trim(audio, rate)
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                with wave.open(str(path), "wb") as w:
                    w.setnchannels(1)
                    w.setsampwidth(2)
                    w.setframerate(rate)
                    w.writeframes(audio.tobytes())
            except OSError as exc:  # unwritable cache: still play, just uncached
                print(f"  ⚠️  could not cache cue {phrase!r}: {exc}")
        self._clips[phrase] = audio
        return audio

    def prewarm(self) -> None:
        """Load or synthesize every phrase now, so no wake waits on the network."""
        if self.enabled:
            for phrase in self._phrases:
                self._clip(phrase)

    def play(self) -> str | None:
        """Play a cue and block until it has finished; return what was said."""
        if not self.enabled:
            return None
        phrase = random.choice(self._phrases)
        audio = self._clip(phrase)
        if audio is None:
            return None
        self._tts.play(audio)
        return phrase
