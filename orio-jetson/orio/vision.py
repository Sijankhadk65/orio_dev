"""Object detection from Orio's camera (YOLO via ultralytics).

A *query*, not a perception loop: `detect_once()` grabs a single frame and
runs YOLO over it, called on demand by the `what_do_you_see` tool (see
tools.py). Out of the real-time path, like the LLM/ASR/TTS.

The camera handle and model are opened lazily on first use and kept open
(re-opening a webcam is slow, ~1-2s) — call `close()` on shutdown to release
the camera. `_lock` serializes access to the camera/model so an on-demand
`detect_once()` (from a tool call) and the continuous `VisionDebugWindow`
loop below never read the same camera concurrently.

On the robot the camera is usually NOT this class's to open: obstacle avoidance
holds both CSI sensors whenever Orio can drive, so the tool calls `detect_in()`
with the stereo pair's left frame instead. `detect_once()` remains the path for
a run with no driving (`ORIO_DRIVE=0`) and for bench use off the robot.
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


def _argus_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    """GStreamer pipeline pulling CSI frames through the Jetson ISP via Argus.

    `nvarguscamerasrc` yields NVMM (device-memory) buffers that `nvvidconv`
    debayers and rescales in hardware; only the final BGR copy touches the CPU.
    Scaling here rather than in numpy keeps YOLO off the sensor's native
    3280x2464 frames. See config.CAMERA_USE_ARGUS for why plain V4L2 is not an
    option on this hardware.
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=1920,height=1080,framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw,width={width},height={height},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=2"
    )


class ObjectDetector:
    """Lazily opens the camera + YOLO model; call `detect_once()` per query."""

    def __init__(
        self,
        camera_index: int = config.CAMERA_INDEX,
        model_path: Path = config.YOLO_MODEL_PATH,
        confidence: float = config.YOLO_CONFIDENCE,
        use_argus: bool = config.CAMERA_USE_ARGUS,
        sensor_id: int = config.CAMERA_SENSOR_ID,
        width: int = config.CAMERA_WIDTH,
        height: int = config.CAMERA_HEIGHT,
        fps: int = config.CAMERA_FPS,
    ) -> None:
        self._camera_index = camera_index
        self._model_path = model_path
        self._confidence = confidence
        self._use_argus = use_argus
        self._sensor_id = sensor_id
        self._width = width
        self._height = height
        self._fps = fps
        self._source = "camera"
        self._blank_checked = False
        self._cap = None
        self._model = None
        self._lock = threading.Lock()

    def _ensure_model(self) -> None:
        if self._model is None:
            from ultralytics import YOLO  # heavy import (torch), kept lazy

            self._model_path.parent.mkdir(parents=True, exist_ok=True)
            self._model = YOLO(str(self._model_path))

    def _ensure_open(self) -> None:
        self._ensure_model()

        if self._cap is None:
            import cv2  # heavy import, kept lazy

            if self._use_argus:
                self._source = f"Argus sensor {self._sensor_id}"
                cap = cv2.VideoCapture(
                    _argus_pipeline(self._sensor_id, self._width, self._height, self._fps),
                    cv2.CAP_GSTREAMER,
                )
                if not cap.isOpened():
                    cap.release()
                    raise RuntimeError(
                        f"could not open {self._source}. If OpenCV was built without "
                        "GStreamer support this always fails — Orio expects JetPack's "
                        "system cv2, not a PyPI opencv-python wheel."
                    )
            else:
                self._source = f"camera {self._camera_index}"
                cap = cv2.VideoCapture(self._camera_index)
                if not cap.isOpened():
                    cap.release()
                    raise RuntimeError(f"could not open {self._source}")
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
            raise RuntimeError(f"could not read a frame from {self._source}")
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

        A bad capture path does not raise here — a plain V4L2 grab from the
        IMX219 returns a uniform buffer and still reports success, which reads
        downstream as "the camera works but YOLO never detects anything". A
        frame holding exactly one value means the pipeline is wrong, not that
        the room is empty, so say so loudly. Subsampled, and only checked once.
        """
        if self._blank_checked:
            return
        self._blank_checked = True
        sample = frame[::16, ::16]
        if sample.max() == sample.min():
            log.error(
                "%s returned a blank, single-colour frame — the capture path is "
                "broken, not the scene. Expect no detections. Check that cv2 has "
                "GStreamer support and ORIO_CAMERA_USE_ARGUS=1.",
                self._source,
            )

    def detect_once(self) -> list[Detection]:
        """Capture one frame from this class's own camera and detect in it."""
        with self._lock:
            detections, _frame = self._detect_locked()
        return detections

    def detect_in(self, frame) -> list[Detection]:
        """Detect in a frame captured by somebody else — opens no camera.

        Whenever Orio can drive, the stereo thread in `avoid.py` holds BOTH CSI
        sensors for the whole session, and Argus will not open a third handle on
        a sensor it already owns. So the vision tool is handed the stereo pair's
        left eye instead of opening a camera of its own.

        That frame is smaller than this class's own capture would be
        (`config.STEREO_WIDTH` x `STEREO_HEIGHT`, against `CAMERA_WIDTH` x
        `CAMERA_HEIGHT`) and it is rectified rather than raw, so expect the model
        to miss small or distant objects it would otherwise catch. Sharing the
        one frame there is beats reporting that Orio cannot see at all.
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
