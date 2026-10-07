"""The WebSocket the orio-app connects to (docs/app-protocol.md).

A development window onto the robot, never its controller. Everything here is
built so the robot cannot tell whether an app is connected:

* It runs its own asyncio loop on its own daemon thread, so a slow or stuck
  client never reaches the conversation loop or a drive tick.
* It only reads. Status is polled from `Body.telemetry()` and the FSM, and the
  conversation arrives through `telemetry.hub`, whose producers never wait on a
  listener. A client that drops costs nothing; one that reconnects is sent the
  recent conversation and carries on.
* It never raises into the caller: a port that will not bind is a startup note,
  the same as a board that will not open.

The robot advertises `status` and `transcript`, plus `video` when there is a
frame source. `commands` and `drive` are advertised only when the caller hands
in handlers for them — the laptop mock does today; the robot will in Phases 5
and 6, after Phase 1 gives the wheels an owner. Without a handler a `command`
is answered with a refusal and `drive` is ignored, so the app greys both out
and a stray message can do nothing.

The joystick's deadman lives here, not in the handler, so every robot gets it:
a client whose `drive` messages stop for `ORIO_APP_DRIVE_DEADMAN_S` while the
stick is off centre, or that disconnects mid-drive, is driven to (0, 0) and
an `event` says why.

Video is JPEG frames over this same socket, sent only to clients that asked for
them. One thread grabs and encodes while anyone is watching and sleeps when no
one is; each viewer sends only the newest frame, so a slow link drops frames
instead of building a backlog.

`AppServer` takes the status and the frames as callables so
`tools/mock_server.py` can serve the same protocol from a laptop with nothing
behind it.
"""

from __future__ import annotations

import asyncio
import hmac
import json
import logging
import math
import struct
import threading
import time
from http import HTTPStatus
from typing import Any, Callable
from urllib.parse import urlsplit

from . import config
from . import telemetry

log = logging.getLogger(__name__)

PROTOCOL_VERSION = "0.1"
FEATURES = ("status", "transcript")

# A video frame source: given the seq of the last frame it returned, the next
# newer frame as (seq, capture time in Unix seconds, JPEG bytes), or None when
# there is no new frame. Called from the video thread, never the event loop.
FrameSource = Callable[[int], "tuple[int, float, bytes] | None"]

# A command handler: (name, target or None) -> (ok, text for the app). Runs on a
# worker thread, so a slow one (go_to) never holds up status or a `stop`.
CommandHandler = Callable[[str, "str | None"], "tuple[bool, str]"]

# A drive handler: joystick (x right, y forward), each -1..1. Called on the
# event loop for every message, so it must only latch the intent and return.
DriveHandler = Callable[[float, float], None]

# Binary message: "OJPG", uint32 seq, float64 capture time, then the JPEG.
VIDEO_MAGIC = b"OJPG"
_VIDEO_HEADER = struct.Struct(">4sId")

# How long a new connection has to send its hello before it is dropped.
_HELLO_TIMEOUT_S = 5.0
# Reports queued for one slow client before the oldest are dropped. The status
# stream is resent every tick anyway; this only bounds a burst of transcript.
_QUEUE_MAX = 256


def _major(version: object) -> str:
    return str(version).split(".")[0]


def _clean(value: Any) -> Any:
    """JSON-safe copy: NaN and infinities become null, as an unknown sector is."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


def _encode(message: dict[str, Any]) -> str:
    return json.dumps(_clean(message), allow_nan=False, default=str)


class AppServer:
    """Serves `hello`, `status` and `transcript` to any number of apps."""

    def __init__(
        self,
        status: Callable[[], dict[str, Any]],
        *,
        robot: str = "orio",
        features: tuple[str, ...] = FEATURES,
        hub: telemetry.Telemetry = telemetry.hub,
        host: str = config.APP_HOST,
        port: int = config.APP_PORT,
        path: str = config.APP_PATH,
        token: str = config.APP_TOKEN,
        status_hz: float = config.APP_STATUS_HZ,
        video: FrameSource | None = None,
        video_fps: float = config.APP_VIDEO_FPS,
        commands: CommandHandler | None = None,
        drive: DriveHandler | None = None,
        drive_deadman_s: float = config.APP_DRIVE_DEADMAN_S,
    ) -> None:
        self._status = status
        self._robot = robot
        self._features = list(features)
        self._video = video
        if video is not None and "video" not in self._features:
            self._features.append("video")
        self._video_period = 1.0 / max(video_fps, 0.5)
        self._commands = commands
        self._drive = drive
        self._deadman_s = drive_deadman_s
        for feature, handler in (("commands", commands), ("drive", drive)):
            if handler is not None and feature not in self._features:
                self._features.append(feature)
        self._viewers: set[_Client] = set()   # touched only on the event loop
        self._watching = 0                    # viewers with video on
        self._want_video = threading.Event()  # set while _watching > 0; the video thread idles on it
        self._frame: tuple[int, float, bytes] | None = None
        self._video_thread: threading.Thread | None = None
        self._hub = hub
        self.host, self.port, self.path = host, port, path
        self._token = token
        self._period = 1.0 / max(status_hz, 0.5)
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stopping: asyncio.Event | None = None
        self._ready = threading.Event()
        self._error: BaseException | None = None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self, timeout_s: float = 3.0) -> str:
        """Start serving; return one line for the startup notes. Never raises."""
        self._thread = threading.Thread(target=self._run, name="app-server", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout_s):
            return f"⚠ app server did not start within {timeout_s:g} s"
        if self._error is not None:
            return f"⚠ app server disabled: {self._error}"
        where = f"ws://{self.host}:{self.port}{self.path}"
        if not self._token:
            return (f"app server on {where} — no ORIO_APP_TOKEN set, so any client "
                    f"on the network can read status and the conversation")
        return f"app server on {where}"

    @property
    def running(self) -> bool:
        return self._ready.is_set() and self._error is None

    def stop(self) -> None:
        loop, stopping = self._loop, self._stopping
        if loop is not None and stopping is not None:
            loop.call_soon_threadsafe(stopping.set)
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        try:
            asyncio.run(self._main())
        except BaseException as exc:  # bind failure, missing dependency, ...
            self._error = exc
            self._ready.set()
            log.warning("app server stopped: %s", exc)

    async def _main(self) -> None:
        from websockets.asyncio.server import serve

        self._loop = asyncio.get_running_loop()
        self._stopping = asyncio.Event()
        async with serve(self._session, self.host, self.port,
                         process_request=self._check_path, ping_interval=10, ping_timeout=10):
            self._ready.set()
            await self._stopping.wait()

    def _check_path(self, connection, request):
        """Refuse anything but the app's path, with a hint, before the upgrade."""
        if urlsplit(request.path).path != self.path:
            return connection.respond(
                HTTPStatus.NOT_FOUND, f"Orio app server: connect a WebSocket to {self.path}\n")
        return None

    # ── one client ───────────────────────────────────────────────────────────

    async def _session(self, ws) -> None:
        from websockets.exceptions import ConnectionClosed

        peer = ws.remote_address
        try:
            raw = await asyncio.wait_for(ws.recv(), _HELLO_TIMEOUT_S)
        except (TimeoutError, ConnectionClosed):
            return
        hello = self._parse(raw)
        if hello is None or hello.get("type") != "hello":
            await self._refuse(ws, "send a hello first")
            return

        # Our hello goes out even to a client that is about to be refused, so an
        # incompatible app can show why instead of retrying forever.
        await ws.send(_encode({"type": "hello", "version": PROTOCOL_VERSION,
                               "robot": self._robot, "features": self._features}))
        if _major(hello.get("version")) != _major(PROTOCOL_VERSION):
            await self._refuse(ws, f"robot speaks protocol {PROTOCOL_VERSION}, "
                                   f"app speaks {hello.get('version')}")
            return
        if self._token and not hmac.compare_digest(str(hello.get("token", "")), self._token):
            log.warning("app client %s refused: wrong token", peer)
            await self._refuse(ws, "wrong token", code=1008)
            return

        log.info("app client %s connected (%s)", peer, hello.get("client", "?"))
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=_QUEUE_MAX)

        def offer(message: dict[str, Any]) -> None:
            if queue.full():
                queue.get_nowait()  # drop the oldest; never block the producer
            queue.put_nowait(message)

        # Subscribe before taking the backlog: a line reported in between then
        # arrives twice rather than not at all.
        unsubscribe = self._hub.subscribe(lambda m: loop.call_soon_threadsafe(offer, m))
        viewer = _Client()
        self._viewers.add(viewer)
        try:
            for line in self._hub.backlog():
                await ws.send(_encode(line))
            jobs = [self._send_status(ws), self._forward(ws, queue),
                    self._receive(ws, viewer), self._send_video(ws, viewer)]
            if self._drive is not None:
                jobs.append(self._deadman(viewer))
            tasks = [asyncio.create_task(job) for job in jobs]
            try:
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            finally:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
        except ConnectionClosed:
            pass
        finally:
            self._set_watching(viewer, False)
            self._viewers.discard(viewer)
            if viewer.driving:
                self._halt(viewer, "the app disconnected while driving")
            unsubscribe()
            log.info("app client %s disconnected", peer)

    async def _send_status(self, ws) -> None:
        failed = False
        while True:
            started = time.monotonic()
            try:
                status = self._status()
                failed = False
            except Exception:
                if not failed:  # once per run of failures, not 5 times a second
                    log.exception("app status failed")
                failed = True
                status = {"state": "ERROR", "behaviour": "unknown",
                          "behaviour_detail": "status unavailable"}
            await ws.send(_encode({"type": "status", "t": time.time(), **status}))
            await asyncio.sleep(max(0.0, self._period - (time.monotonic() - started)))

    async def _forward(self, ws, queue: asyncio.Queue) -> None:
        while True:
            await ws.send(_encode(await queue.get()))

    async def _receive(self, ws, viewer: "_Client") -> None:
        async for raw in ws:
            message = self._parse(raw)
            if message is None:
                continue
            kind = message.get("type")
            if kind == "video":
                if self._video is not None:
                    self._set_watching(viewer, bool(message.get("on")))
            elif kind == "command":
                if self._commands is None:
                    # Not advertised, so the app should not send one; answer anyway
                    # so a stray command is visibly refused, never silently lost.
                    await ws.send(_encode({
                        "type": "result", "id": message.get("id"), "ok": False,
                        "text": "this robot doesn't take commands from the app yet",
                    }))
                else:
                    task = asyncio.create_task(self._run_command(ws, message))
                    viewer.jobs.add(task)
                    task.add_done_callback(viewer.jobs.discard)
            elif kind == "drive" and self._drive is not None:
                self._on_drive(viewer, message)
            # Without a drive handler `drive` is ignored: the wheels never hear of it.

    # ── commands and the joystick ────────────────────────────────────────────

    async def _run_command(self, ws, message: dict[str, Any]) -> None:
        from websockets.exceptions import ConnectionClosed

        name = str(message.get("name", ""))
        target = message.get("target")
        target = target.strip() if isinstance(target, str) and target.strip() else None
        try:
            ok, text = await asyncio.get_running_loop().run_in_executor(
                None, self._commands, name, target)
        except Exception as exc:
            log.exception("app command %s failed", name)
            ok, text = False, f"{name} failed: {exc}"
        try:
            await ws.send(_encode({"type": "result", "id": message.get("id"),
                                   "ok": bool(ok), "text": str(text)}))
        except ConnectionClosed:
            pass  # the command still happened; the app sees its effect in status

    def _on_drive(self, client: "_Client", message: dict[str, Any]) -> None:
        seq = message.get("seq")
        if isinstance(seq, int):
            if seq <= client.drive_seq:
                return  # late or repeated; a newer position already went through
            client.drive_seq = seq

        def axis(v: object) -> float:
            return max(-1.0, min(1.0, float(v))) if isinstance(v, (int, float)) and math.isfinite(v) else 0.0

        x, y = axis(message.get("x")), axis(message.get("y"))
        client.drive_at = time.monotonic()
        client.driving = (x, y) != (0.0, 0.0)
        self._call_drive(x, y)

    async def _deadman(self, client: "_Client") -> None:
        """Stop a client whose stick is off centre but whose messages stopped."""
        while True:
            await asyncio.sleep(min(0.05, self._deadman_s / 4))
            if client.driving and time.monotonic() - client.drive_at > self._deadman_s:
                self._halt(client, f"no joystick message for {self._deadman_s:g} s (deadman)")

    def _halt(self, client: "_Client", why: str) -> None:
        client.driving = False
        self._call_drive(0.0, 0.0)
        log.info("app drive stopped: %s", why)
        self._hub.event("halted", why)

    def _call_drive(self, x: float, y: float) -> None:
        try:
            self._drive(x, y)
        except Exception:
            log.exception("app drive handler failed")

    # ── video ────────────────────────────────────────────────────────────────

    def _set_watching(self, viewer: "_Client", on: bool) -> None:
        """Turn one client's frames on or off; start the grabber for the first."""
        if viewer.on == on:
            return
        viewer.on = on
        self._watching += 1 if on else -1
        if self._watching > 0:
            self._want_video.set()
        else:
            self._want_video.clear()
        if on:
            viewer.wake.set()  # send the latest frame now rather than at the next one
            if self._video_thread is None:
                self._video_thread = threading.Thread(
                    target=self._grab_loop, name="app-video", daemon=True)
                self._video_thread.start()

    def _grab_loop(self) -> None:
        """Grab and encode at most video_fps while anyone watches; idle otherwise."""
        loop, source = self._loop, self._video
        if loop is None or source is None:
            return
        seq, failed = 0, False
        while True:
            self._want_video.wait()
            started = time.monotonic()
            try:
                frame = source(seq)
                failed = False
            except Exception:
                if not failed:
                    log.exception("app video frame failed")
                failed, frame = True, None
            if frame is not None:
                seq = frame[0]
                loop.call_soon_threadsafe(self._publish, frame)
            time.sleep(max(0.0, self._video_period - (time.monotonic() - started)))

    def _publish(self, frame: tuple[int, float, bytes]) -> None:
        self._frame = frame
        for viewer in self._viewers:
            if viewer.on:
                viewer.wake.set()

    async def _send_video(self, ws, viewer: "_Client") -> None:
        sent = -1
        while True:
            await viewer.wake.wait()
            viewer.wake.clear()
            frame = self._frame
            if not viewer.on or frame is None or frame[0] == sent:
                continue
            seq, t, jpeg = frame
            # While this send is in flight newer frames only overwrite
            # self._frame, so a slow client gets the newest, never a backlog.
            await ws.send(_VIDEO_HEADER.pack(VIDEO_MAGIC, seq & 0xFFFFFFFF, t) + jpeg)
            sent = seq

    @staticmethod
    def _parse(raw: Any) -> dict[str, Any] | None:
        if not isinstance(raw, str):
            return None
        try:
            message = json.loads(raw)
        except ValueError:
            return None
        return message if isinstance(message, dict) else None

    @staticmethod
    async def _refuse(ws, text: str, code: int = 1002) -> None:
        try:
            await ws.send(_encode({"type": "error", "text": text}))
            await ws.close(code=code, reason=text[:120])
        except Exception:
            pass


class _Client:
    """One connected app: its video, joystick and in-flight commands."""

    __slots__ = ("on", "wake", "driving", "drive_at", "drive_seq", "jobs")

    def __init__(self) -> None:
        self.on = False               # wants video frames
        self.wake = asyncio.Event()   # a newer frame is ready for it
        self.driving = False          # last drive was off centre
        self.drive_at = 0.0           # monotonic time of the last drive
        self.drive_seq = -1
        self.jobs: set[asyncio.Task] = set()  # running commands, kept from GC


# ── the real robot's status ──────────────────────────────────────────────────


def robot_status(fsm) -> dict[str, Any]:
    """Status from the FSM and the body, in the protocol's `status` shape.

    `wheel_owner` stays null until Phase 1 gives the wheels an owner to report.
    """
    from . import body as body_mod

    status: dict[str, Any] = {"state": str(fsm.state) if fsm is not None else "UNKNOWN",
                              "wheel_owner": None}
    b = body_mod.get()
    if b is None:
        return {**status, "behaviour": "idle", "behaviour_detail": "body not started",
                "sensors": {}}
    tel = b.telemetry()
    distances = tel.pop("sectors")
    tel.pop("can_drive")
    status.update(tel)
    if distances:
        status["sectors"] = {"fov_deg": config.STEREO_HFOV_DEG, "distance_m": distances,
                             "stop_m": config.AVOID_STOP_M, "clear_m": config.AVOID_CLEAR_M}
    return status


_RETRY_S = 5.0


def camera_frames(width: int = config.APP_VIDEO_WIDTH,
                  quality: int = config.APP_VIDEO_QUALITY) -> FrameSource:
    """Frames from the shared Gemini colour stream, scaled and JPEG-encoded.

    Reads the same camera as avoidance and the detector, through `read()`,
    which only looks at the newest frameset — it never takes a frame away from
    anyone else. A camera that has failed or stalled gives None, and the app
    shows its frames going stale.

    `read()` opens the camera if nothing has yet, so with no camera attached
    every grab would retry the SDK's open. After a failure it waits
    `_RETRY_S` before asking again.
    """
    import cv2

    from .gemini import shared

    params = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, quality))]
    retry_at = 0.0

    def grab(after: int) -> tuple[int, float, bytes] | None:
        nonlocal retry_at
        if time.monotonic() < retry_at:
            return None
        try:
            frames = shared().read(after=after, timeout=0.5)
        except Exception:
            retry_at = time.monotonic() + _RETRY_S
            return None
        image = frames.color
        h, w = image.shape[:2]
        if w > width:
            image = cv2.resize(image, (width, round(h * width / w)), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", image, params)
        if not ok:
            return None
        # The frameset carries a monotonic arrival time; the app wants wall time.
        captured = time.time() - (time.monotonic() - frames.timestamp)
        return frames.seq, captured, jpeg.tobytes()

    return grab


def start(fsm) -> tuple[AppServer | None, str]:
    """Start the app server for this robot run, if enabled. Never raises."""
    if not config.APP_SERVER_ENABLED:
        return None, "app server off (ORIO_APP_SERVER=0)"
    video, video_note = None, ""
    if config.APP_VIDEO_ENABLED:
        try:
            video = camera_frames()
        except Exception as exc:  # no cv2: serve everything but the camera view
            video_note = f"; no camera view ({exc})"
    server = AppServer(lambda: robot_status(fsm), video=video)
    note = server.start() + video_note
    return (server if server.running else None), note
