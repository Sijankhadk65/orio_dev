"""Interactive conversation loop: talk to Orio by voice (or keyboard), it replies
in text and speech.

Ties the mic/ASR input (`asr`) and keyboard input to the LLM brain
(`llm.Conversation`) and the TTS engine (`tts`). Tools come later — for now this
is purely conversational, scoped by the system prompt.
"""

from __future__ import annotations

from collections.abc import Iterator

from . import config
from .llm import Conversation, OllamaUnavailable
from .tts import get_tts

BANNER = """\
╭──────────────────────────────────────────────╮
│  Orio — operator layer (conversation only)    │
│  model: {model:<37}│
│  input: {inp:<37}│
│  tts:   {tts:<37}│
╰──────────────────────────────────────────────╯
"""

# Spoken phrases that end the session (kept distinct from the robot command
# word "stop", which is reserved for a future motion-halt tool).
STOP_PHRASES = {"quit", "exit", ":q", "power down", "shut down", "goodbye orio"}


def _is_stop(text: str) -> bool:
    t = text.lower().strip(" .!?")
    return t in STOP_PHRASES


def _voice_inputs() -> Iterator[str]:
    """Yield transcribed phrases from the mic until interrupted."""
    from .asr import SpeechToText

    print("Loading speech recognizer…")
    stt = SpeechToText()
    print("Ready. Speak to Orio (Ctrl-C to stop).")
    while True:
        print("\n🎤 listening…", end="", flush=True)
        try:
            text = stt.listen()
        except KeyboardInterrupt:
            return
        if not text:
            print("\r              \r", end="")  # clear the listening line
            continue
        print(f"\ryou › {text}")
        yield text


def _text_inputs() -> Iterator[str]:
    """Yield typed lines until EOF/Ctrl-D."""
    print("Type to Orio (Ctrl-D to stop).")
    while True:
        try:
            line = input("\nyou › ").strip()
        except (EOFError, KeyboardInterrupt):
            return
        if line:
            yield line


def run() -> None:
    print(
        BANNER.format(
            model=config.LLM_MODEL, inp=config.INPUT_MODE, tts=config.TTS_ENGINE
        )
    )

    convo = Conversation()
    try:
        convo.preflight()
    except OllamaUnavailable as exc:
        print(f"✗ {exc}")
        return

    tts = get_tts()
    inputs = _voice_inputs() if config.INPUT_MODE == "voice" else _text_inputs()

    try:
        for user in inputs:
            if _is_stop(user):
                break

            # Stream the reply to the console as it generates, then speak it whole.
            print("orio › ", end="", flush=True)
            reply_parts: list[str] = []
            try:
                for piece in convo.send(user):
                    print(piece, end="", flush=True)
                    reply_parts.append(piece)
            except Exception as exc:  # daemon died mid-stream, etc.
                print(f"\n✗ LLM error: {exc}")
                continue
            print()

            tts.speak("".join(reply_parts))
    except KeyboardInterrupt:
        pass

    print("\nOrio: powering down. Bye!")
