"""Wake-word detection ("Hey Orio"), Whisper-match strategy.

Rather than run a dedicated always-on hotword model, we reuse the Whisper ASR
that's already loaded: the conversation loop endpoints each spoken phrase,
transcribes it, and asks this matcher whether it contains the wake word. This
adds no model or dependency and recognizes the literal name "Orio" out of the
box; the trade-off is the CPU cost of transcribing ambient phrases while
dormant (the same work the loop already did when it was always-on).

Matching is token-based so it's forgiving of how Whisper renders short phrases:
- configured wake *phrases* ("hey orio", …) are matched as whole-word sequences
  anywhere in the transcript, longest first;
- failing that, *multi-word* phrases are fuzzily matched against sliding windows
  to absorb mishearings of the name ("hey oreo", "hey ario"). Bare single-word
  matches must be exact — fuzzing a lone "orio" can't be told apart from real
  words like "ohio"/"radio" (measured), so we don't try.

`split()` also returns whatever followed the wake word in the same utterance
(in its original form, punctuation intact), so "Hey Orio, what's your status?"
yields the command "what's your status?" with no second listen. The detector is
stateless and cheap; swapping in a real hotword engine later means replacing
this class behind the same `split()` contract.
"""

from __future__ import annotations

import re
from difflib import SequenceMatcher

from . import config

_NON_WORD = re.compile(r"[^a-z0-9]+")


def _norm_token(token: str) -> str:
    """Lowercase a token and strip all non-alphanumerics ('what's' -> 'whats')."""
    return _NON_WORD.sub("", token.lower())


class WakeWord:
    """Detects the wake word in a transcript and splits off the trailing command."""

    def __init__(
        self,
        phrases: tuple[str, ...] = config.WAKE_PHRASES,
        fuzzy: float = config.WAKE_FUZZY,
    ) -> None:
        # Each phrase as a list of normalized words; longest (most words) first
        # so "hey orio" wins over a bare "orio" and we strip the whole phrase.
        seen: dict[tuple[str, ...], None] = {}
        for p in phrases:
            words = tuple(w for w in (_norm_token(t) for t in p.split()) if w)
            if words:
                seen.setdefault(words, None)
        self._phrases = sorted(seen, key=len, reverse=True)
        self._fuzzy = fuzzy

    @property
    def label(self) -> str:
        """Human-readable primary wake phrase for prompts, e.g. 'Hey Orio'."""
        return config.WAKE_PHRASES[0].title() if config.WAKE_PHRASES else "Orio"

    def split(self, transcript: str) -> tuple[bool, str]:
        """Return (woke, command) for a transcript.

        `woke` is True if the wake word was heard; `command` is whatever followed
        it in the same utterance, in its original form ('' if nothing followed).
        """
        raw = transcript.split()
        pairs = [(r, n) for r in raw if (n := _norm_token(r))]
        if not pairs:
            return False, ""
        raws = [r for r, _ in pairs]
        norms = [n for _, n in pairs]

        # 1) Exact whole-word phrase match (longest phrase first).
        for words in self._phrases:
            k = len(words)
            for i in range(len(norms) - k + 1):
                if tuple(norms[i : i + k]) == words:
                    return True, " ".join(raws[i + k :])

        # 2) Fuzzy window match for multi-word phrases (mishearings of the name).
        for words in self._phrases:
            k = len(words)
            if k < 2:
                continue  # single-word matches must be exact (see module docs)
            phrase = " ".join(words)
            for i in range(len(norms) - k + 1):
                window = " ".join(norms[i : i + k])
                if SequenceMatcher(None, window, phrase).ratio() >= self._fuzzy:
                    return True, " ".join(raws[i + k :])

        return False, ""

    def matches(self, transcript: str) -> bool:
        """True if the transcript contains the wake word."""
        return self.split(transcript)[0]


class WhisperWaker:
    """Wake gate using the Whisper ASR + the `WakeWord` matcher (the default).

    Endpoints and transcribes each dormant phrase via `stt`, then checks it for
    the wake word. Exposes the same `label` / `await_wake()` interface as the
    dedicated openWakeWord engine (`wake_oww.OpenWakeWord`), so the conversation
    loop can use either interchangeably. The matcher itself (`WakeWord`) stays a
    pure, dependency-free string brain; this just drives it with live audio.
    """

    def __init__(self, stt, matcher: WakeWord | None = None) -> None:
        self._stt = stt
        self._wake = matcher or WakeWord()

    @property
    def label(self) -> str:
        return self._wake.label

    def await_wake(self) -> str | None:
        """Block until the wake word is heard.

        Returns whatever followed the wake word in the same utterance ('' if
        nothing) so it can be run immediately, or None if interrupted / the mic
        failed.
        """
        while True:
            try:
                text = self._stt.listen()  # wait indefinitely for a phrase
            except KeyboardInterrupt:
                return None
            except RuntimeError as exc:  # audio capture failed (see asr.listen)
                print(f"\n✗ {exc}")
                return None
            if not text:
                continue
            woke, command = self._wake.split(text)
            if woke:
                return command
            # Heard speech, but Orio wasn't addressed — keep sleeping.
