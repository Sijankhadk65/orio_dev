"""Short spoken cues — the "Hmm?" Orio makes when it starts listening, and the
yoga coach's corrections.

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

The yoga coach (yoga.py) speaks from a fixed set of sentences too, so it uses
the same cache through `CoachVoice` — but in the background, because a camera
loop that waited on audio would stop watching the person it is talking to.
"""

from __future__ import annotations

import hashlib
import random
import threading
import time
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


class PhraseCache:
    """Phrases in the TTS voice, synthesized once and kept as WAVs on disk.

    Only an engine that can render audio (`synthesize`) can fill it; with the
    console engine `enabled` is False and nothing is ever synthesized.
    """

    def __init__(self, tts, cache_dir: Path = config.CUE_CACHE_DIR) -> None:
        self._tts = tts if hasattr(tts, "synthesize") else None
        self._dir = cache_dir
        self._clips: dict[str, np.ndarray] = {}

    @property
    def enabled(self) -> bool:
        return self._tts is not None

    def _path(self, phrase: str) -> Path:
        key = hashlib.sha1(f"{self._tts.voice_id}\0{phrase}".encode()).hexdigest()[:16]
        return self._dir / f"{key}.wav"

    def clip(self, phrase: str) -> np.ndarray | None:
        """The phrase as int16 PCM, from memory, disk or the network; None offline."""
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
                return None  # offline: nothing this time, try again next call
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

    def play(self, audio: np.ndarray) -> None:
        self._tts.play(audio)


class ListenCue:
    """Plays one of `phrases` at random, in the TTS voice. Silent without one."""

    def __init__(
        self,
        tts,
        phrases: tuple[str, ...] = config.LISTEN_CUE_PHRASES,
        cache_dir: Path = config.CUE_CACHE_DIR,
    ) -> None:
        # The console engine has nothing to play, so the cue is quietly off with it.
        self._cache = PhraseCache(tts, cache_dir)
        self._phrases = phrases

    @property
    def enabled(self) -> bool:
        return self._cache.enabled and bool(self._phrases)

    def prewarm(self) -> None:
        """Load or synthesize every phrase now, so no wake waits on the network."""
        if self.enabled:
            for phrase in self._phrases:
                self._cache.clip(phrase)

    def play(self) -> None:
        """Play a cue and block until it has finished."""
        if not self.enabled:
            return
        audio = self._cache.clip(random.choice(self._phrases))
        if audio is not None:
            self._cache.play(audio)


class CoachVoice:
    """Speaks the yoga coach's cues on a background thread, newest first.

    `say()` never blocks: the camera loop hands the text over and keeps
    watching. While one cue plays, a newer one waits — and replaces any cue
    already waiting, because what the person needs to hear is what is wrong
    now, not a backlog. A cue that waited longer than `stale_s` is dropped
    rather than spoken late about a pose they have already changed.

    With an engine that cannot render audio (console), each cue goes to
    `tts.speak()` instead, which prints it.
    """

    def __init__(self, tts, cache_dir: Path = config.CUE_CACHE_DIR,
                 stale_s: float = 3.0) -> None:
        self._tts = tts
        self._cache = PhraseCache(tts, cache_dir)
        self._stale_s = stale_s
        self._pending: tuple[str, float] | None = None
        self._cond = threading.Condition()
        self._closed = False
        self.speaking = False
        self._thread = threading.Thread(target=self._run, name="coach-voice", daemon=True)
        self._thread.start()

    def prewarm(self, phrases) -> None:
        """Synthesize every phrase now (blocking), so no cue waits on the network.

        Only the first run for a voice pays this: after that they load from disk.
        """
        if not self._cache.enabled:
            return
        for phrase in phrases:
            self._cache.clip(phrase)

    def say(self, text: str) -> None:
        with self._cond:
            self._pending = (text, time.monotonic())
            self._cond.notify()

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._pending is None and not self._closed:
                    self._cond.wait()
                if self._closed:
                    return
                text, at = self._pending
                self._pending = None
            if time.monotonic() - at > self._stale_s:
                continue
            self.speaking = True
            try:
                if self._cache.enabled:
                    audio = self._cache.clip(text)
                    if audio is not None:
                        self._cache.play(audio)
                else:
                    self._tts.speak(text)
            except Exception as exc:  # noqa: BLE001 - a voice failure must not end the session
                print(f"  ⚠️  coach voice failed: {exc}")
            finally:
                self.speaking = False

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify()
        self._thread.join(timeout=1.0)
