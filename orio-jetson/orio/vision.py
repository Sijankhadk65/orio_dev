"""Object detection from Orio's camera (YOLO via ultralytics).

A *query*, not a perception loop: `detect_once()` grabs a single frame and
runs YOLO over it, called on demand by the `what_do_you_see` tool (see
tools.py). Out of the real-time path, like the LLM/ASR/TTS.

The camera handle and model are opened lazily on first use and kept open
(re-opening a webcam is slow, ~1-2s) — call `close()` on shutdown to release
the camera. `_lock` serializes access to the camera/model so an on-demand
`detect_once()` (from a tool call) and the continuous `VisionDebugWindow`
loop below never read the same camera concurrently.
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
        camera_index: int = config.CAMERA_INDEX,
        model_path: Path = config.YOLO_MODEL_PATH,
        confidence: float = config.YOLO_CONFIDENCE,
    ) -> None:
        self._camera_index = camera_index
        self._model_path = model_path
        self._confidence = confidence
        self._cap = None
        self._model = None
        self._lock = threading.Lock()

    def _ensure_open(self) -> None:
        if self._model is None:
            from ultralytics import YOLO  # heavy import (torch), kept lazy

            self._model_path.parent.mkdir(parents=True, exist_ok=True)
            self._model = YOLO(str(self._model_path))

        if self._cap is None:
            import cv2  # heavy import, kept lazy

            cap = cv2.VideoCapture(self._camera_index)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError(f"could not open camera {self._camera_index}")
            self._cap = cap

    def _detect_locked(self):
        """Capture one frame + detect, returning (detections, raw_frame).

        Internal: the raw frame is only needed by the debug window's overlay.
        Caller must hold `_lock` — both `detect_once()` and
        `VisionDebugWindow` go through this so they never share the camera
        handle across threads unsynchronized.
        """
        self._ensure_open()
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError(f"could not read a frame from camera {self._camera_index}")

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
        return detections, frame

    def detect_once(self) -> list[Detection]:
        """Capture one frame and return the objects detected in it."""
        with self._lock:
            detections, _frame = self._detect_locked()
        return detections

    def close(self) -> None:
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None


class VisionDebugWindow:
    """Live camera + detection preview window, for debugging only.

    Runs its own thread, sharing `detector`'s lock with any on-demand
    `detect_once()` tool calls so the two never race for the camera. Not part
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
