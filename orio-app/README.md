# orio-app

Flutter app for **viewing and debugging Orio** from a phone or tablet. It is a
window into the robot, not its controller: Orio runs on its own, and the app
only shows what it is doing and sends it intents. The design lives in the
*Orio Mobile App Plan* doc.

It talks to `orio-jetson/orio/server.py` on the robot, or to the laptop mock
(`uv run tools/mock_server.py` in `orio-jetson/`), which serves the same
protocol with made-up numbers and a test-pattern camera. **Demo** on the
connect screen is a fake Orio inside the app, for when neither is running; it
has no video.

## What's in v1

| Screen | Shows / does | Needs from the robot |
|---|---|---|
| Status | behaviour, voice FSM state, wheel owner, IMU heading, sensor health, recent events | `status`, `event` |
| Sector map | the forward fan, coloured against the stop (0.20 m) and clear (0.70 m) distances; unknown drawn grey, never clear | `status.sectors` |
| Talk | what Orio heard, said, and which tools the LLM called | `transcript` |
| Drive | STOP / Stay / Follow / Go to, plus a joystick over the video pane | `command`, `drive` |
| Video | the robot's colour camera as JPEG frames (~10 fps, 640 px), with fps and lag; requested only while the pane is on screen. WebRTC is Phase 7 | `video` |

The robot lists its features in `hello`. Anything it doesn't list shows as
"not available on this robot yet" instead of breaking, so screens can ship
before their server side.

Phones (< 840 px wide) get tabs; tablets get two panes: video, sector map and
commands on the left, status and conversation on the right.

## Run it

```bash
cd orio-app
flutter pub get
flutter run                      # attached phone or a running emulator
flutter run -d chrome            # quickest way to look at it on the laptop
flutter build apk --debug        # build/app/outputs/flutter-apk/app-debug.apk
flutter test
```

Server addresses are settings, edited on the connect screen and saved on the
device. The defaults are:

| Profile | Address |
|---|---|
| Demo | none (in-app fake) |
| Laptop mock (emulator) | `ws://10.0.2.2:8765/ws` (the emulator's name for the laptop) |
| Robot on Wi-Fi | `ws://orio.local:8765/ws` |
| Robot over Tailscale | `ws://orio:8765/ws` |

Port 8765 and the `/ws` path are placeholders until `server.py` picks them.
iPad builds need a Mac with Xcode (see the plan's open questions).

## Protocol

The contract lives in [`docs/app-protocol.md`](../docs/app-protocol.md) at the
repo root; the Dart models are `lib/protocol/messages.dart`. A message change
updates both, and the robot's `orio-jetson/orio/server.py`, in the same commit.

To develop against a real WebSocket without the robot, run the mock on the
laptop: `cd orio-jetson && uv run tools/mock_server.py` (it installs only
`websockets`), then pick the "Laptop mock" profile.

## Layout

```
lib/
  main.dart                    app + theme
  protocol/messages.dart       message models (the contract, in Dart)
  connection/
    robot_link.dart            WebSocket transport
    demo_link.dart             in-app fake robot
    robot_session.dart         one connection: state, reconnect, staleness, commands
    profiles.dart              saved server addresses + token
  screens/                     connect, home (phone tabs / tablet panes)
  widgets/                     status, sector map, conversation, commands, joystick, video
```
