# Orio app protocol

The contract between `orio-app/` (Flutter, built on the laptop) and the robot's
`orio-jetson/orio/server.py`. The laptop mock, `orio-jetson/tools/mock_server.py`,
serves it through the same server code. The Dart models are
`orio-app/lib/protocol/messages.dart`.

**Any change to a message changes this file, `messages.dart` and the server (or
mock) in the same commit.**

Version **0.1**. The app is a development window onto the robot, never its
controller: nothing on the robot waits for it, and a dropped connection changes
nothing on the robot except that joystick driving stops (once it exists).

## Connection

- One WebSocket at `ws://<host>:8765/ws` (`ORIO_APP_PORT`, `ORIO_APP_PATH`).
  Any other path gets HTTP 404.
- Every message is a JSON object with a `type`. Times (`t`) are Unix seconds.
- The app speaks first, with `hello`. A connection that sends nothing for 5 s is
  dropped.
- The robot always answers with its own `hello`, then checks the app's:
  - different **major** version: `error`, close code 1002;
  - wrong token, when `ORIO_APP_TOKEN` is set on the robot: `error`, close 1008;
  - first message not a `hello`: `error`, close 1002.
- After a good handshake the robot sends the recent conversation (up to the last
  50 `transcript` lines, oldest first), then `status` at ~5 Hz and everything
  else as it happens. The app treats 1.5 s without `status` as stale.
- The robot pings every 10 s and drops a client that does not answer.

## Features

The robot's `hello` lists what it serves. The app disables everything not listed.

| Feature | Messages | Served since |
| --- | --- | --- |
| `status` | `status` | Phase 2 |
| `transcript` | `transcript` | Phase 2 |
| `commands` | `command`, `result`, `event` | the mock and the robot, 2026-10-07 |
| `drive` | `drive` | the mock and the robot, 2026-10-07 |
| `video` | `video` (app), binary camera frames (robot) | JPEG fallback, 2026-10-07; WebRTC is Phase 7 |
| `lidar` | (not yet specified) | after the L2 is fitted |

Until `commands` is listed, a `command` is answered with `result` `ok: false`.
Until `drive` is listed, `drive` messages are ignored.

## Messages

```jsonc
// app -> robot, first message
{"type":"hello","version":"0.1","client":"orio-app","token":"..."}

// robot -> app, once, always
{"type":"hello","version":"0.1","robot":"orio","features":["status","transcript"]}

// robot -> app, ~5 Hz
{"type":"status","t":1760000000.2,
 "state":"LISTENING",              // FSM: ASLEEP IDLE LISTENING THINKING SPEAKING ERROR
 "behaviour":"cruise",             // idle | cruise (voice) | go_to (app) | driving (joystick)
 "behaviour_detail":"forward",     // free text, may be ""
 "wheel_owner":null,               // "app" (joystick), "go_to" (an app go_to), or null
 "heading_deg":12.5,               // IMU yaw, null when there is no fresh IMU reading
 "speed_percent":5,                // drive duty ceiling
 "sensors":{                       // name -> {ok, detail}; names present depend on what is fitted
   "camera":{"ok":true,"detail":"Gemini 336L · 33 ms old"},
   "tof":{"ok":true,"detail":"left, right"},   // only when ORIO_TOF=1
   "imu":{"ok":true,"detail":"BNO085"},
   "drivetrain":{"ok":true,"detail":"..."},
   "neck":{"ok":true,"detail":"holding pose"}},
 "sectors":{                       // absent when there is no depth reading
   "fov_deg":94,
   "distance_m":[1.6,1.2,null,0.9,1.4,2.0,1.8],   // left to right; null = unknown, treat as blocked
   "stop_m":0.2,"clear_m":0.7}}

// robot -> app, one conversation line, as it happens
{"type":"transcript","t":...,"kind":"wake","text":"Hey Orio"}
{"type":"transcript","t":...,"kind":"heard","text":"go to the chair"}     // ASR text, or typed in text mode
{"type":"transcript","t":...,"kind":"said","text":"Hmm?"}                 // TTS text, including the wake cue
{"type":"transcript","t":...,"kind":"tool","text":"go_to","name":"go_to",
 "args":{"target":"chair"},"result":"went to the chair — standing 0.7 m away, facing them"}

// robot -> app, as it happens
{"type":"event","t":...,"kind":"arrived|halted|blocked|lost","text":"stopped about a metre from the chair"}

// app -> robot, and the reply. stay and follow_me are always refused (ok: false):
// the robot has no such behaviours.
{"type":"command","id":"c7","name":"stop|go_to|stay|follow_me","target":"chair"}
{"type":"result","id":"c7","ok":true,"text":"on my way to the chair"}

// app -> robot, ~10 Hz while the pad is held, one zero on release
{"type":"drive","seq":42,"x":0.0,"y":0.8}

// robot -> app, anything the user should see; usually followed by a close
{"type":"error","text":"wrong token"}

// app -> robot, start or stop camera frames for this client (feature `video`)
{"type":"video","on":true}
```

## Camera frames

The only binary messages. The robot sends them to a client only between its
`{"type":"video","on":true}` and `on:false` (or the disconnect), so a pane that
is off screen costs the robot nothing. The app turns video off whenever the
video pane is hidden, and on again after a reconnect.

| Bytes | Type (big-endian) | Meaning |
| --- | --- | --- |
| 0–3 | ASCII | `OJPG` |
| 4–7 | uint32 | frame seq, increasing (wraps at 2³²) |
| 8–15 | float64 | capture time, Unix seconds (the app shows lag only if the clocks agree) |
| 16– | bytes | one JPEG, 640 px wide by default, the camera's aspect |

The robot grabs and encodes at most `ORIO_APP_VIDEO_FPS`, only while someone
watches, once per frame for all viewers. Each viewer is sent the newest frame
when its previous send completes, so a slow link skips frames instead of
queueing them. A stalled camera simply stops the frames; the app marks the
picture frozen after 2 s.

The frames are the colour stream avoidance and the detector already read
(`gemini.shared()`), never a second capture. Encoding runs on the CPU (the Orin
Nano has no hardware encoder): if avoidance slows while someone watches, lower
the fps or width first.

## Commands and the joystick

`orio/server.py` serves both when it is given handlers for them; the laptop
mock passes its own, and the robot passes `RobotControl`. The app fits what the
robot already does; the robot's behaviours are not changed for the app.

On the robot:

- **`stop`** is `Body.stop()`, the same as the voice `stop_moving`. It ends an
  app `go_to` and the joystick too.
- **`go_to`** runs the same approach as the voice tool (`seek.Seeker`), on its
  own thread: the `result` is "on my way to the chair", and the outcome follows
  as an `event` (`arrived`, `blocked`, `lost`, or `halted` when it was stopped
  or the wheels were taken). A second `go_to` replaces the first.
- **`stay` and `follow_me`** are refused, with a reason.
- **The joystick is a 4-way pad**, not proportional: the larger axis picks
  forward, backward, left or right, at the robot's speed (5%), under the
  avoider. **Backward is blind** — the cameras face forward. A stop from voice
  or the app ends joystick driving, and the stick does nothing more until it has
  been back to the centre.
- One set of wheels, newest wins: the joystick ends an app `go_to`; a voice move
  or `go_to` ends the joystick. A voice `go_to` still holds the conversation
  until it finishes, as it always has; stopping it from the app stops the wheels
  at once. A voice `move_*` or `turn_*` hop is the exception: it holds the
  wheels for its whole length (up to `ORIO_DRIVE_MAX_STEP_S`, 4 s), and a stop
  or the stick from the app takes effect when it ends.

- **Commands** run on a worker thread each, so a slow one (`go_to`) never holds
  up status or a `stop` sent after it. Every `command` gets exactly one
  `result` with its `id`. `ok: false` means refused or failed; `text` says why
  in the words the voice tools use. Anything that happens later (arrived, lost
  the target, blocked) is an `event`.
- **`drive`** carries the joystick, x right and y forward, each -1..1. The app
  applies a 15% dead zone, so a thumb resting near the centre sends (0, 0),
  never a direction. A `seq` at or below the last one seen is ignored.
- **Deadman, in the server, for every robot:** a client whose last `drive` was
  off centre and that sends nothing for `ORIO_APP_DRIVE_DEADMAN_S` (0.3 s) is
  driven to (0, 0) and gets `{"type":"event","kind":"halted","text":"no
  joystick message for 0.3 s (deadman)"}`. Disconnecting mid-drive does the
  same at once. A clean release, a single (0, 0), raises no event.

The app never sends wheel duties, only intent. The robot's avoider, 5% duty
ceiling and the deadman decide what the wheels do.

## Robot settings

| Variable | Default | Meaning |
| --- | --- | --- |
| `ORIO_APP_SERVER` | `1` | `0` turns the server off |
| `ORIO_APP_HOST` | `0.0.0.0` | bind address |
| `ORIO_APP_PORT` | `8765` | port |
| `ORIO_APP_PATH` | `/ws` | WebSocket path |
| `ORIO_APP_TOKEN` | unset | shared secret, `.env` only; unset accepts any client (with a startup warning) |
| `ORIO_APP_STATUS_HZ` | `5` | status rate |
| `ORIO_APP_TRANSCRIPT_BACKLOG` | `50` | lines sent on connect |
| `ORIO_APP_VIDEO` | `1` | `0` drops `video` from the features |
| `ORIO_APP_VIDEO_FPS` | `10` | most frames per second sent |
| `ORIO_APP_VIDEO_WIDTH` | `640` | frame width in px; height keeps the aspect |
| `ORIO_APP_VIDEO_QUALITY` | `60` | JPEG quality, 1–100 |
| `ORIO_APP_DRIVE_DEADMAN_S` | `0.3` | joystick silence before the server stops the wheels |
