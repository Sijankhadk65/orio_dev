# orio-app

Flutter app for **viewing and debugging Orio** from a phone or tablet. It is a
window into the robot, not its controller: Orio runs on its own, and the app
only shows what it is doing and sends it intents. The design lives in the
*Orio Mobile App Plan* doc.

The robot side (`orio-jetson/orio/server.py`) and the laptop mock are not built
yet. Until they are, use **Demo** on the connect screen: a fake Orio inside the
app that speaks the same protocol.

## What's in v1

| Screen | Shows / does | Needs from the robot |
|---|---|---|
| Status | behaviour, voice FSM state, wheel owner, IMU heading, sensor health, recent events | `status`, `event` |
| Sector map | the forward fan, coloured against the stop (0.20 m) and clear (0.70 m) distances; unknown drawn grey, never clear | `status.sectors` |
| Talk | what Orio heard, said, and which tools the LLM called | `transcript` |
| Drive | STOP / Stay / Follow / Go to, plus a joystick over the video pane | `command`, `drive` |
| Video | placeholder until Phase 7 (WebRTC) | `video` |

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

## Protocol (draft for `docs/app-protocol.md`)

One WebSocket. Every message is JSON with a `type`; the major `version` must
match or the app refuses the robot. Models are in `lib/protocol/messages.dart`.

```jsonc
// app -> robot, first message
{"type":"hello","version":"0.1","client":"orio-app","token":"..."}
// robot -> app, once
{"type":"hello","version":"0.1","robot":"orio","features":["status","transcript","commands","drive","video"]}
// robot -> app, ~5 Hz
{"type":"status","t":1760000000.2,"state":"LISTENING","behaviour":"go_to","behaviour_detail":"approaching chair",
 "wheel_owner":"go_to","heading_deg":12.5,"speed_percent":5,
 "sensors":{"camera":{"ok":true,"detail":"Gemini 336L · 30 fps"},"imu":{"ok":true},"tof":{"ok":true}},
 "sectors":{"fov_deg":94,"distance_m":[1.6,1.2,null,0.9,1.4,2.0,1.8],"stop_m":0.2,"clear_m":0.7}}
// robot -> app, as it happens (the last ~50 transcript lines on connect)
{"type":"event","t":...,"kind":"arrived|halted|blocked|lost","text":"stopped about a metre from the chair"}
{"type":"transcript","t":...,"kind":"wake|heard|said|tool","text":"go to the chair"}
{"type":"transcript","t":...,"kind":"tool","name":"go_to","args":{"target":"chair"},"result":"arrived"}
// app -> robot, and the reply
{"type":"command","id":"c7","name":"stop|stay|follow_me|go_to","target":"chair"}
{"type":"result","id":"c7","ok":true,"text":"on my way to the chair"}
// app -> robot, ~10 Hz while the pad is held, one zero on release
{"type":"drive","seq":42,"x":0.0,"y":0.8}
// robot -> app, anything it wants the user to see
{"type":"error","text":"wrong token"}
```

The app never sends wheel duties, only intent. The robot's avoider, 5% duty
ceiling and deadman (no `drive` for ~300 ms = stop) decide what the wheels do.

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
