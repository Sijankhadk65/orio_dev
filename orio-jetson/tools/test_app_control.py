#!/usr/bin/env python3
"""Check the app's commands and joystick (`server.RobotControl`) against fake boards.

    uv run python tools/test_app_control.py

No hardware: the fake drivetrain, sensor and simulated approach come from
`test_cruise.py`. The joystick deadman is checked over a real WebSocket, through
the same `AppServer` the robot runs, so "the phone vanished mid-drive" is the
thing being measured rather than a call standing in for it.

What none of this can say is how the robot behaves on a floor: run the phases'
acceptance tests on the real robot for that.
"""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from orio import config, telemetry  # noqa: E402
from orio.body import NO_WHEELS  # noqa: E402
from orio.server import AppServer, RobotControl, _Stick  # noqa: E402
from test_cruise import FAILURES, FakeLink, World, check, fake_body, no_cruise_threads  # noqa: E402


class FakeDetector:
    def __init__(self, world: World | None = None) -> None:
        self._world = world

    def labels(self) -> list[str]:
        return ["person", "chair"]

    def detect_in(self, frame):
        return self._world.detect(frame) if self._world is not None else []


def wait_for(condition, timeout: float, step: float = 0.005) -> float | None:
    """Seconds until `condition()` held, or None if it never did."""
    started = time.monotonic()
    while time.monotonic() - started < timeout:
        if condition():
            return time.monotonic() - started
        time.sleep(step)
    return None


def forward(pair) -> bool:
    return pair is not None and pair[0] > 0 and pair[1] > 0


class Events:
    """Every `event` the hub publishes, from now on."""

    def __init__(self) -> None:
        self.seen: list[dict] = []
        self._off = telemetry.hub.subscribe(
            lambda m: self.seen.append(m) if m.get("type") == "event" else None)

    def first(self, timeout: float) -> dict | None:
        wait_for(lambda: bool(self.seen), timeout)
        return self.seen[0] if self.seen else None

    def close(self) -> None:
        self._off()


def hold_stick(control: RobotControl, x: float, y: float, stop: threading.Event) -> threading.Thread:
    """Send a position at ~10 Hz, like a held thumb, until `stop`."""
    def run() -> None:
        while not stop.is_set():
            control.drive(x, y)
            time.sleep(0.1)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


# ── the joystick ─────────────────────────────────────────────────────────────


def test_stick_directions() -> None:
    print("\nthe stick is a 4-way pad")
    cases = {(0.0, 1.0): "forward", (0.0, -0.6): "backward", (0.9, 0.2): "right",
             (-0.9, -0.3): "left", (0.5, 0.5): "forward", (0.0, 0.0): None}
    wrong = {k: _Stick.direction(*k) for k, v in cases.items() if _Stick.direction(*k) != v}
    check("the larger axis picks the direction", not wrong, f"{wrong}")


def test_stick_drives_and_releases() -> None:
    print("\nholding and releasing the stick")
    link = FakeLink()
    body = fake_body(link=link)
    control = RobotControl(lambda: body, detector=FakeDetector)
    stop = threading.Event()
    hold_stick(control, 0.0, 0.8, stop)
    took = wait_for(lambda: forward(link.standing), 1.0)
    check("held forward drives forward", took is not None, f"standing {link.standing}")
    check("at the body's speed, not the stick's",
          link.standing is not None and max(link.standing) <= round(config.DRIVE_SPEED_MAX_PERCENT * 10) + 1,
          f"{link.standing}")
    status = control.status()
    check("status says the app is driving",
          status.get("wheel_owner") == "app" and status.get("behaviour") == "driving", f"{status}")
    stop.set()
    time.sleep(0.15)
    control.drive(0.0, 0.0)
    took = wait_for(link.stopped, 0.5)
    check("releasing stops the wheels at once", took is not None and took < 0.1, f"{took}")
    check("no cruise is left running", wait_for(no_cruise_threads, 1.0) is not None)
    check("status no longer claims the wheels", control.status() == {}, f"{control.status()}")
    body.close()


def test_voice_stop_beats_a_held_stick() -> None:
    print("\na spoken stop while the thumb is still down")
    link = FakeLink()
    body = fake_body(link=link)
    control = RobotControl(lambda: body, detector=FakeDetector)
    stop = threading.Event()
    hold_stick(control, 0.0, 1.0, stop)
    wait_for(lambda: forward(link.standing), 1.0)
    body.stop()  # what stop_moving calls
    took = wait_for(link.stopped, 0.5)
    check("the wheels stop", took is not None and took < 0.1, f"{took}")
    time.sleep(0.6)
    check("the held stick does not start them again", link.stopped(), f"{link.standing}")
    stop.set()
    time.sleep(0.15)
    control.drive(0.0, 0.0)
    time.sleep(0.1)
    control.drive(0.0, 1.0)
    check("after the centre, the stick drives again",
          wait_for(lambda: forward(link.standing), 1.0) is not None, f"{link.standing}")
    control.drive(0.0, 0.0)
    wait_for(link.stopped, 0.5)
    body.close()


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def connect(port: int):
    from websockets.sync.client import connect as ws_connect

    ws = ws_connect(f"ws://127.0.0.1:{port}/ws")
    ws.send(json.dumps({"type": "hello", "version": "0.1", "client": "test", "token": ""}))
    hello = json.loads(ws.recv())
    return ws, hello


def test_deadman_over_the_socket() -> None:
    print("\nthe joystick deadman, over a real WebSocket")
    link = FakeLink()
    body = fake_body(link=link)
    control = RobotControl(lambda: body, detector=FakeDetector)
    port = free_port()
    server = AppServer(lambda: {"state": "IDLE", **control.status()}, host="127.0.0.1", port=port,
                       token="", commands=control.command, drive=control.drive,
                       hub=telemetry.Telemetry())
    note = server.start()
    check("the server starts", server.running, note)
    if not server.running:
        return

    ws, hello = connect(port)
    check("it advertises commands and drive",
          {"commands", "drive"} <= set(hello.get("features", [])), f"{hello}")

    # Disconnect mid-drive.
    for seq in range(1, 6):
        ws.send(json.dumps({"type": "drive", "seq": seq, "x": 0.0, "y": 0.9}))
        time.sleep(0.1)
    check("driving over the socket", forward(link.standing), f"{link.standing}")
    t0 = time.monotonic()
    ws.close()
    took = wait_for(link.stopped, 1.0)
    check("closing the app mid-drive stops the wheels within 0.3 s",
          took is not None and took <= config.APP_DRIVE_DEADMAN_S, f"{took}")

    # Go silent mid-drive, connection still open.
    ws, _ = connect(port)
    for seq in range(1, 6):
        ws.send(json.dumps({"type": "drive", "seq": seq, "x": 0.0, "y": 0.9}))
        time.sleep(0.1)
    check("driving again", forward(link.standing), f"{link.standing}")
    t0 = time.monotonic()
    took = wait_for(link.stopped, 1.5)
    check("silence stops the wheels within the deadman (+ one check)",
          took is not None and took <= config.APP_DRIVE_DEADMAN_S + 0.1, f"{took}")
    ws.close()
    server.stop()
    body.close()


# ── commands ─────────────────────────────────────────────────────────────────


def test_refusals() -> None:
    print("\nwhat the robot can't do is refused in words")
    body = fake_body()
    control = RobotControl(lambda: body, detector=FakeDetector)
    for name, target in (("stay", None), ("follow_me", None), ("dance", None),
                         ("go_to", None), ("go_to", "dragon")):
        ok, text = control.command(name, target)
        check(f"{name} {target or ''}".strip() + f" -> {text!r}", not ok and bool(text))
    ok, text = RobotControl(lambda: None, detector=FakeDetector).command("stop", None)
    check("no body: no wheels", not ok and text == NO_WHEELS, text)
    body.close()


def test_go_to_arrives() -> None:
    print("\nan app go_to, start to finish")
    world = World(distance_m=2.0)
    body = fake_body(link=world, sensor=world)
    control = RobotControl(lambda: body, detector=lambda: FakeDetector(world))
    events = Events()
    ok, text = control.command("go_to", "me")  # aliased to person, as voice does
    check("answered at once", ok and text == "on my way to the person", text)
    check("status says go_to", control.status().get("behaviour") == "go_to", f"{control.status()}")
    event = events.first(20.0)
    events.close()
    check("it ends in an arrived event",
          event is not None and event["kind"] == "arrived", f"{event}")
    check("the wheels are left stopped", world.standing == (0, 0), f"{world.standing}")
    check("no cruise is left running", wait_for(no_cruise_threads, 1.0) is not None)
    body.close()


def _stop_mid_go_to(how: str, stop) -> None:
    world = World(distance_m=8.0)
    body = fake_body(link=world, sensor=world)
    control = RobotControl(lambda: body, detector=lambda: FakeDetector(world))
    events = Events()
    control.command("go_to", "person")
    rolling = wait_for(lambda: world.standing[0] > 0 and world.standing[1] > 0, 5.0)
    check(f"{how}: the go_to is rolling", rolling is not None, f"{world.standing}")
    time.sleep(0.5)
    t0 = time.monotonic()
    stop(control, body)
    took = wait_for(lambda: world.standing == (0, 0), 1.0)
    check(f"{how}: the wheels stop within 0.5 s", took is not None and took <= 0.5, f"{took}")
    event = events.first(3.0)
    events.close()
    check(f"{how}: a halted event follows",
          event is not None and event["kind"] == "halted", f"{event}")
    errand = control._errand
    check(f"{how}: the go_to is over within 3 s",
          errand is not None and not errand[0].is_alive(), "still running")
    time.sleep(0.5)
    check(f"{how}: and the wheels stay stopped", world.standing == (0, 0), f"{world.standing}")
    check(f"{how}: status is clear", control.status() == {}, f"{control.status()}")
    body.close()


def test_stop_mid_go_to() -> None:
    print("\nstopping an app go_to")
    _stop_mid_go_to("stop from the app", lambda c, b: c.command("stop", None))
    _stop_mid_go_to("stop_moving by voice", lambda c, b: b.stop())


def test_stick_takes_over_a_go_to() -> None:
    print("\nthe stick during an app go_to")
    world = World(distance_m=8.0)
    body = fake_body(link=world, sensor=world)
    control = RobotControl(lambda: body, detector=lambda: FakeDetector(world))
    events = Events()
    control.command("go_to", "person")
    wait_for(lambda: world.standing[0] > 0, 5.0)
    stop = threading.Event()
    hold_stick(control, -1.0, 0.0, stop)
    turning = wait_for(lambda: world.standing[0] < 0 < world.standing[1], 2.0)
    check("the stick takes the wheels (pivoting left)", turning is not None, f"{world.standing}")
    event = events.first(3.0)
    events.close()
    check("the go_to reports it was halted", event is not None and event["kind"] == "halted", f"{event}")
    check("status shows the stick", control.status().get("wheel_owner") == "app", f"{control.status()}")
    stop.set()
    time.sleep(0.15)
    control.drive(0.0, 0.0)
    check("release stops", wait_for(lambda: world.standing == (0, 0), 0.5) is not None)
    body.close()


def main() -> int:
    for test in (
        test_stick_directions,
        test_stick_drives_and_releases,
        test_voice_stop_beats_a_held_stick,
        test_deadman_over_the_socket,
        test_refusals,
        test_go_to_arrives,
        test_stop_mid_go_to,
        test_stick_takes_over_a_go_to,
    ):
        test()
    if FAILURES:
        print(f"\n{len(FAILURES)} failed: " + ", ".join(FAILURES))
        return 1
    print("\nall passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
