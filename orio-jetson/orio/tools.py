"""LangChain-compatible tools the LLM can call.

Kept separate from llm.py so the tool set can grow without touching the
conversation wrapper. Each tool is a plain `@tool`-decorated function; the
wrapper only ever calls tools that *answer questions* (query the world) —
nothing here talks to the STM32 or actuates the robot.

Missing deps or hardware (no camera, opencv/ultralytics not installed)
degrade to an empty tool list rather than breaking the conversation loop —
mirrors how tts.py/eyes.py fall back on missing deps.
"""

from __future__ import annotations

import logging

from langchain_core.tools import tool

from . import config

log = logging.getLogger(__name__)

_detector = None  # lazily-built ObjectDetector, shared across calls


def get_detector():
    """The shared ObjectDetector (built on first use). Also used by
    conversation.py to hand the same instance to VisionDebugWindow, so the
    debug preview and the LLM's tool calls never open a second camera handle.
    """
    global _detector
    if _detector is None:
        from .vision import ObjectDetector

        _detector = ObjectDetector()
    return _detector


@tool
def what_do_you_see() -> str:
    """Look through Orio's camera right now and report what objects are visible.

    Use this whenever asked what you can see, what's in front of you, or to
    describe your surroundings. Takes no arguments.
    """
    try:
        detections = get_detector().detect_once()
    except Exception as exc:
        return f"camera error: {exc}"
    if not detections:
        return "no recognizable objects in view right now"
    return "; ".join(
        f"{d.label} ({d.position}, {d.confidence:.0%} confidence)" for d in detections
    )


def get_tools() -> list:
    """Build the enabled tool list, degrading to none on missing deps."""
    if not config.TOOLS_ENABLED:
        return []
    try:
        import cv2  # noqa: F401
        import ultralytics  # noqa: F401
    except Exception as exc:
        log.warning("vision tool unavailable (%s); Orio has no tools this run", exc)
        return []
    return [what_do_you_see]


def close_tools() -> None:
    """Release any held hardware (camera). Call on shutdown."""
    global _detector
    if _detector is not None:
        _detector.close()
        _detector = None
