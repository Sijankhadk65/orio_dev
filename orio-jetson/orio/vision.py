"""Object detection from Orio's camera (YOLO via ultralytics).

A *query*, not a perception loop: `detect_once()` takes a single colour frame
and runs YOLO over it, called on demand by the `what_do_you_see` tool (see
tools.py). Out of the real-time path, like the LLM/ASR/TTS.

Frames come from the Gemini 336L's colour camera through `gemini.shared()`,
the same device the obstacle map reads depth from. The camera is shared rather
than owned, so the detector, the avoidance thread and the debug preview can all
run at once. When Orio can drive, the tool usually calls `detect_in()` with the
avoidance thread's latest frame instead, so the detection and the depth it is
paired with come from the same instant (see `avoid.Sensor.snapshot`).
`_lock` serializes access to the model so an on-demand `detect_once()` and the
continuous `VisionDebugWindow` loop below never run inference concurrently.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from . import config

log = logging.getLogger(__name__)


@dataclass
class Detection:
    label: str
    confidence: float
    position: str  # e.g. "center, close"
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2 in pixel coords


def _position(x1: float, y1: float, x2: float, y2: float, frame_w: int, frame_h: int) -> str:
    """Rough natural-language position: left/center/right + close/far by box size.

    Precise enough for the LLM to describe out loud; not meant for navigation.
    """
    cx = (x1 + x2) / 2
    if cx < frame_w / 3:
        horiz = "left"
    elif cx > frame_w * 2 / 3:
        horiz = "right"
    else:
        horiz = "center"
    area_frac = ((x2 - x1) * (y2 - y1)) / (frame_w * frame_h)
    depth = "close" if area_frac > 0.15 else "far"
    return f"{horiz}, {depth}"


class ObjectDetector:
    """Lazily opens the camera + YOLO model; call `detect_once()` per query."""

    def __init__(
        self,
        model_path: Path = config.YOLO_MODEL_PATH,
        confidence: float = config.YOLO_CONFIDENCE,
        camera=None,
    ) -> None:
        self._model_path = model_path
        self._confidence = confidence
        self._camera = camera
        self._seq = 0
        self._blank_checked = False
        self._model = None
        self._lock = threading.Lock()

    def _ensure_model(self) -> None:
        if self._model is None:
            from ultralytics import YOLO  # heavy import (torch), kept lazy

            self._model_path.parent.mkdir(parents=True, exist_ok=True)
            self._model = YOLO(str(self._model_path))

    def _ensure_camera(self):
        if self._camera is None:
            from .gemini import shared

            self._camera = shared()
        return self._camera

    def _detect_locked(self):
        """Take one colour frame + detect, returning (detections, frame).

        Internal: the frame is only needed by the debug window's overlay, and it
        is the camera's shared buffer — copy before drawing on it. Caller must
        hold `_lock` — both `detect_once()` and `VisionDebugWindow` go through
        this so they never run the model concurrently.
        """
        self._ensure_model()
        frames = self._ensure_camera().read(after=self._seq)
        self._seq = frames.seq
        frame = frames.color
        self._warn_if_blank(frame)
        return self._detect_frame(frame), frame

    def _detect_frame(self, frame):
        """Run the model over one frame. Caller must hold `_lock`."""
        h, w = frame.shape[:2]
        result = self._model.predict(frame, conf=self._confidence, verbose=False)[0]
        detections: list[Detection] = []
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(
                Detection(
                    label=result.names[int(box.cls[0])],
                    confidence=float(box.conf[0]),
                    position=_position(x1, y1, x2, y2, w, h),
                    bbox=(x1, y1, x2, y2),
                )
            )
        return detections

    def _warn_if_blank(self, frame) -> None:
        """One-shot guard against a silently broken capture path.

        A bad capture path need not raise — a mis-negotiated colour format can
        decode to a uniform buffer, which reads downstream as "the camera works
        but YOLO never detects anything". A frame holding exactly one value
        means the pipeline is wrong, not that the room is empty, so say so
        loudly. Subsampled, and only checked once.
        """
        if self._blank_checked:
            return
        self._blank_checked = True
        sample = frame[::16, ::16]
        if sample.max() == sample.min():
            log.error(
                "the Gemini returned a blank, single-colour frame — the capture "
                "path is broken, not the scene. Expect no detections. Run "
                "tools/gemini_check.py to see what the colour stream negotiated."
            )

    def detect_once(self) -> list[Detection]:
        """Take one fresh frame from the camera and detect in it."""
        with self._lock:
            detections, _frame = self._detect_locked()
        return detections

    def detect_in(self, frame) -> list[Detection]:
        """Detect in a frame captured by somebody else — reads no camera.

        Used with the avoidance thread's latest frame (`Body.snapshot()`), so a
        detection is paired with the depth map taken at the same instant. That
        pairing is what anything aiming at a detection relies on: the colour
        frame and the depth map share one pixel grid.
        """
        with self._lock:
            self._ensure_model()
            return self._detect_frame(frame)

    def labels(self) -> list[str]:
        """Every class this model can recognise.

        Used to refuse "go to the X" up front when X is not something the
        detector has ever heard of — otherwise the behaviour sweeps the head
        looking for it, finds nothing, and reports "couldn't find one", which
        reads as "it isn't here" rather than "I can't see those".
        """
        with self._lock:
            self._ensure_model()
            names = getattr(self._model, "names", None) or {}
        return list(names.values()) if isinstance(names, dict) else list(names)

    def close(self) -> None:
        """Nothing to release: the camera is shared, and its owner closes it."""


class VisionDebugWindow:
    """Live camera + detection preview window, for debugging only.

    Runs its own thread, sharing `detector`'s lock with any on-demand
    `detect_once()` tool calls so the two never run the model at once. Not part
    of what the LLM sees or uses — purely a "what is the camera/model actually
    looking at right now" window.
    """

    def __init__(self, detector: ObjectDetector, fps: int = 15) -> None:
        self._detector = detector
        self._fps = max(1, fps)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="vision-debug", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        import cv2

        window = "Orio vision debug"
        try:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        except Exception:
            log.exception("vision debug: could not open window — preview disabled")
            return

        period = 1.0 / self._fps
        last_tick = time.monotonic()
        fps_shown = 0.0

        while not self._stop.is_set():
            start = time.monotonic()
            try:
                with self._detector._lock:
                    detections, frame = self._detector._detect_locked()
                # Shared with every other reader of the camera: draw on a copy.
                frame = frame.copy()
            except Exception as exc:
                log.warning("vision debug: %s", exc)
                if cv2.waitKey(200) == 27:  # Esc — bail out of the retry wait too
                    break
                continue

            now = time.monotonic()
            fps_shown = 1.0 / max(now - last_tick, 1e-6)
            last_tick = now

            for d in detections:
                x1, y1, x2, y2 = (round(v) for v in d.bbox)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, f"{d.label} {d.confidence:.0%}", (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
                )
            cv2.putText(
                frame, f"{fps_shown:4.1f} fps", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
            )

            cv2.imshow(window, frame)
            elapsed = time.monotonic() - start
            wait_ms = max(1, round((period - elapsed) * 1000))
            if cv2.waitKey(wait_ms) == 27:  # Esc closes the preview early
                self._stop.set()

        cv2.destroyWindow(window)
