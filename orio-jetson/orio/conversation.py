"""Interactive conversation loop: talk to Orio by voice (or keyboard), it replies
in text and speech.

Ties the mic/ASR input (`asr`) and keyboard input to the LLM brain
(`llm.Conversation`) and the TTS engine (`tts`), driving the operator state
machine (`fsm`) as it goes. In voice mode Orio is wake-word gated: it stays
ASLEEP until it hears "Hey Orio", then takes commands — with a short follow-up
window so you can chain commands without re-waking it (see `config.WAKE_*`).
Tools come later — for now this is purely conversational, scoped by the system
prompt.
"""

from __future__ import annotations

from . import config
from .fsm import State, StateMachine
from .llm import Conversation, OllamaUnavailable
from .tts import TTS, get_tts

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


def _handle_turn(convo: Conversation, tts: TTS, fsm: StateMachine, user: str) -> None:
    """Generate (THINKING) then speak (SPEAKING) a reply to `user`.

    Leaves the machine in SPEAKING on success so the caller picks the next
    resting state (LISTENING follow-up, ASLEEP, or IDLE); on an LLM failure it
    recovers through ERROR back to IDLE itself.
    """
    fsm.to(State.THINKING)
    print("orio › ", end="", flush=True)
    reply_parts: list[str] = []
    try:
        for piece in convo.send(user):
            print(piece, end="", flush=True)
            reply_parts.append(piece)
    except Exception as exc:  # daemon died mid-stream, etc.
        fsm.to(State.ERROR)
        print(f"\n✗ LLM error: {exc}")
        fsm.to(State.IDLE)
        return
    print()

    fsm.to(State.SPEAKING)
    tts.speak("".join(reply_parts))


def _make_waker(stt):
    """Build the wake-gate engine selected by `config.WAKE_ENGINE`.

    "oww"  → dedicated openWakeWord hotword model (`orio/wake_oww.py`).
    other  → Whisper ASR + string match (`orio/wake.py`), the default.

    Both expose `.label` and `.await_wake()`, so the loop stays engine-agnostic.
    Imports are local so selecting one engine never pulls in the other's deps.
    """
    if config.WAKE_ENGINE == "oww":
        from .wake_oww import OpenWakeWord

        return OpenWakeWord()
    from .wake import WhisperWaker

    return WhisperWaker(stt)


def _run_voice(convo: Conversation, tts: TTS, fsm: StateMachine) -> None:
    """Voice loop. Wake-word gated unless config.WAKE_ENABLED is off."""
    from .asr import SpeechToText

    print("Loading speech recognizer…")
    stt = SpeechToText()
    gated = config.WAKE_ENABLED
    waker = _make_waker(stt) if gated else None

    if gated:
        print(f'Ready. Say "{waker.label}" to wake Orio (Ctrl-C to stop).')
    else:
        print("Ready. Speak to Orio (Ctrl-C to stop).")

    while True:
        pending = ""  # command carried over from the wake utterance, if any
        if gated:
            fsm.to(State.ASLEEP)
            print(f'\n😴 asleep — say "{waker.label}"…', end="", flush=True)
            woke = waker.await_wake()
            if woke is None:
                return  # Ctrl-C or capture failure
            pending = woke
            print("\r" + " " * 40 + "\r", end="")  # clear the asleep line
            fsm.to(State.LISTENING)  # hearing the wake word is itself a listen

        # Awake: take commands, with a follow-up window between them so the user
        # can chain requests without re-waking. An empty window sends us back to
        # sleep (gated) or just keeps listening (always-on).
        while True:
            if pending:
                text: str | None = pending
                pending = ""
                print(f"you › {text}")
            else:
                fsm.to(State.LISTENING)
                print("\n🎤 listening…", end="", flush=True)
                timeout = config.FOLLOWUP_WINDOW_S if gated else None
                try:
                    text = stt.listen(onset_timeout=timeout)
                except KeyboardInterrupt:
                    return
                except RuntimeError as exc:  # audio capture failed
                    fsm.to(State.ERROR)
                    print(f"\n✗ {exc}")
                    fsm.to(State.IDLE)
                    return
                if not text:
                    print("\r              \r", end="")  # clear the listening line
                    if gated:
                        break  # follow-up window lapsed → back to sleep
                    continue  # always-on → keep listening
                print(f"\ryou › {text}")

            if _is_stop(text):
                return
            _handle_turn(convo, tts, fsm, text)


def _run_text(convo: Conversation, tts: TTS, fsm: StateMachine) -> None:
    """Keyboard loop (no wake word — typing is already intentional)."""
    print("Type to Orio (Ctrl-D to stop).")
    while True:
        fsm.to(State.IDLE)
        try:
            line = input("\nyou › ").strip()
        except (EOFError, KeyboardInterrupt):
            return
        if not line:
            continue
        if _is_stop(line):
            return
        _handle_turn(convo, tts, fsm, line)


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
    fsm = StateMachine()  # single source of truth for what Orio is doing

    eyes = _start_eyes(fsm)  # animated face, if enabled and a display is present
    vision_debug = _start_vision_debug()  # camera+detection preview, if enabled

    try:
        if config.INPUT_MODE == "voice":
            _run_voice(convo, tts, fsm)
        else:
            _run_text(convo, tts, fsm)
    except KeyboardInterrupt:
        pass
    finally:
        if eyes is not None:
            eyes.stop()
        if vision_debug is not None:
            vision_debug.stop()
        from .tools import close_tools

        close_tools()  # release the camera, if it was opened

    print("\nOrio: powering down. Bye!")


def _start_vision_debug():
    """Build and start the vision debug preview window if enabled; return it
    (or None).

    Kept import-local so a headless/no-camera run never pulls in cv2 unless
    it's actually wanted, and any failure just disables the preview — it
    never breaks the loop. Shares the same ObjectDetector as the LLM's
    `what_do_you_see` tool calls (see tools.get_detector), not a second one.
    """
    if not config.VISION_DEBUG:
        return None
    try:
        from .tools import get_detector
        from .vision import VisionDebugWindow

        window = VisionDebugWindow(get_detector(), fps=config.VISION_DEBUG_FPS)
        window.start()
        return window
    except Exception as exc:  # missing deps, no camera, etc.
        print(f"⚠ vision debug preview disabled: {exc}")
        return None


def _start_eyes(fsm: StateMachine):
    """Build and start the eyes face if enabled; return it (or None).

    Kept import-local so text/headless runs never pull in pygame/rlottie, and a
    display/clip failure only disables the face — it never breaks the loop.
    """
    if not config.EYES_ENABLED:
        return None
    try:
        from .eyes import EyesController

        eyes = EyesController(
            fsm,
            clips_dir=config.EYES_CLIPS_DIR,
            size=config.EYES_SIZE,
            fullscreen=config.EYES_FULLSCREEN,
            fps=config.EYES_FPS,
            debug=config.EYES_DEBUG,
            transition_ms=config.EYES_TRANSITION_MS,
        )
        eyes.start()
        return eyes
    except Exception as exc:  # missing deps, no display, etc.
        print(f"⚠ eyes disabled: {exc}")
        return None
