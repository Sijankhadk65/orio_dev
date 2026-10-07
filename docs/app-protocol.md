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
| `commands` | `command`, `result`, `event` | Phase 5 |
| `drive` | `drive` | Phase 6 |
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
 "behaviour":"cruise",             // idle | cruise today; go_to, follow_me, stay, driving from Phase 1/5/6
 "behaviour_detail":"forward",     // free text, may be ""
 "wheel_owner":null,               // null until Phase 1 gives the wheels an owner
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

// robot -> app, as it happens (Phase 5)
{"type":"event","t":...,"kind":"arrived|halted|blocked|lost","text":"stopped about a metre from the chair"}

// app -> robot, and the reply (Phase 5)
{"type":"command","id":"c7","name":"stop|stay|follow_me|go_to","target":"chair"}
{"type":"result","id":"c7","ok":true,"text":"on my way to the chair"}

// app -> robot, ~10 Hz while the pad is held, one zero on release (Phase 6)
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

The app never sends wheel duties, only intent. The robot's avoider, 5% duty
ceiling and deadman (no `drive` for ~300 ms = stop) decide what the wheels do.

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
