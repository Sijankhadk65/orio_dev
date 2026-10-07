// Message shapes for the app <-> robot WebSocket.
//
// Draft of the contract the plan puts in `docs/app-protocol.md` (Phase 0).
// Every message is a JSON object with a `type`. Parsing is lenient on purpose:
// the robot side is not built yet, so a missing field falls back to "unknown"
// instead of dropping the whole message.

import 'dart:typed_data';

const String protocolVersion = '0.1';

/// Features a robot can advertise in its `hello`. The app hides or disables
/// whatever the robot does not list, so screens can exist before the robot
/// side of them does.
class Feature {
  static const status = 'status';
  static const transcript = 'transcript';
  static const commands = 'commands';
  static const drive = 'drive';
  static const video = 'video';
  static const lidar = 'lidar';
}

double? _toDouble(Object? v) => v is num ? v.toDouble() : null;

DateTime _toTime(Object? v) {
  final secs = _toDouble(v);
  if (secs == null) return DateTime.now();
  return DateTime.fromMillisecondsSinceEpoch((secs * 1000).round());
}

/// robot -> app, once on connect.
class Hello {
  final String version;
  final String robot;
  final Set<String> features;

  const Hello({required this.version, required this.robot, required this.features});

  factory Hello.fromJson(Map<String, dynamic> j) => Hello(
    version: '${j['version'] ?? '?'}',
    robot: '${j['robot'] ?? 'orio'}',
    features: {for (final f in (j['features'] as List? ?? const [])) '$f'},
  );

  bool has(String feature) => features.contains(feature);

  /// Major versions must match; the server enforces this too.
  bool get compatible => version.split('.').first == protocolVersion.split('.').first;
}

class SensorHealth {
  final bool ok;
  final String detail;

  const SensorHealth({required this.ok, this.detail = ''});

  factory SensorHealth.fromJson(Object? j) {
    if (j is Map) return SensorHealth(ok: j['ok'] == true, detail: '${j['detail'] ?? ''}');
    if (j is bool) return SensorHealth(ok: j);
    return const SensorHealth(ok: false, detail: 'no data');
  }
}

/// The forward sector map, as `orio/sectors.py` reduces it. `null` distance
/// means unknown, which the robot treats as blocked — so does the app.
class SectorMap {
  final double fovDeg;
  final List<double?> distancesM; // left to right
  final double stopM;
  final double clearM;

  const SectorMap({
    required this.fovDeg,
    required this.distancesM,
    required this.stopM,
    required this.clearM,
  });

  factory SectorMap.fromJson(Map<String, dynamic> j) => SectorMap(
    fovDeg: _toDouble(j['fov_deg']) ?? 94,
    distancesM: [for (final d in (j['distance_m'] as List? ?? const [])) _toDouble(d)],
    stopM: _toDouble(j['stop_m']) ?? 0.20,
    clearM: _toDouble(j['clear_m']) ?? 0.70,
  );
}

/// robot -> app, ~5 Hz.
class Status {
  final DateTime time;
  final String fsmState; // ASLEEP, IDLE, LISTENING, THINKING, SPEAKING, ERROR
  final String behaviour; // idle, go_to, follow_me, driving, ...
  final String behaviourDetail;
  final String? wheelOwner; // null = nobody
  final double? headingDeg;
  final double? speedPercent;
  final Map<String, SensorHealth> sensors;
  final SectorMap? sectors;

  const Status({
    required this.time,
    required this.fsmState,
    required this.behaviour,
    this.behaviourDetail = '',
    this.wheelOwner,
    this.headingDeg,
    this.speedPercent,
    this.sensors = const {},
    this.sectors,
  });

  factory Status.fromJson(Map<String, dynamic> j) {
    final sensors = j['sensors'];
    final sectors = j['sectors'];
    return Status(
      time: _toTime(j['t']),
      fsmState: '${j['state'] ?? 'UNKNOWN'}',
      behaviour: '${j['behaviour'] ?? 'unknown'}',
      behaviourDetail: '${j['behaviour_detail'] ?? ''}',
      wheelOwner: j['wheel_owner'] as String?,
      headingDeg: _toDouble(j['heading_deg']),
      speedPercent: _toDouble(j['speed_percent']),
      sensors: sensors is Map
          ? {for (final e in sensors.entries) '${e.key}': SensorHealth.fromJson(e.value)}
          : const {},
      sectors: sectors is Map<String, dynamic> ? SectorMap.fromJson(sectors) : null,
    );
  }
}

/// robot -> app, when something happens (arrived, blocked, halted, ...).
class RobotEvent {
  final DateTime time;
  final String kind;
  final String text;

  const RobotEvent({required this.time, required this.kind, required this.text});

  factory RobotEvent.fromJson(Map<String, dynamic> j) =>
      RobotEvent(time: _toTime(j['t']), kind: '${j['kind'] ?? 'event'}', text: '${j['text'] ?? ''}');
}

enum TranscriptKind { heard, said, tool, wake }

/// robot -> app, one line of the conversation.
class TranscriptLine {
  final DateTime time;
  final TranscriptKind kind;
  final String text; // heard/said: the words; tool: the tool name
  final Map<String, dynamic> args; // tool only
  final String? result; // tool only

  const TranscriptLine({
    required this.time,
    required this.kind,
    required this.text,
    this.args = const {},
    this.result,
  });

  factory TranscriptLine.fromJson(Map<String, dynamic> j) {
    final kind = TranscriptKind.values.firstWhere(
      (k) => k.name == j['kind'],
      orElse: () => TranscriptKind.said,
    );
    return TranscriptLine(
      time: _toTime(j['t']),
      kind: kind,
      text: '${j['text'] ?? j['name'] ?? ''}',
      args: (j['args'] as Map?)?.cast<String, dynamic>() ?? const {},
      result: j['result'] as String?,
    );
  }
}

/// robot -> app, the outcome of one `command`.
class CommandResult {
  final String id;
  final bool ok;
  final String text;

  const CommandResult({required this.id, required this.ok, required this.text});

  factory CommandResult.fromJson(Map<String, dynamic> j) =>
      CommandResult(id: '${j['id']}', ok: j['ok'] != false, text: '${j['text'] ?? ''}');
}

/// robot -> app, one camera frame: a binary WebSocket message of "OJPG",
/// uint32 seq and float64 capture time (Unix seconds), all big-endian, then
/// the JPEG bytes.
class VideoFrame {
  static const headerLength = 16;
  static const _magic = [0x4F, 0x4A, 0x50, 0x47]; // "OJPG"

  final int seq;
  final DateTime captured;
  final Uint8List jpeg;

  const VideoFrame({required this.seq, required this.captured, required this.jpeg});

  /// Null for anything that isn't a well-formed frame.
  static VideoFrame? parse(Uint8List bytes) {
    if (bytes.length <= headerLength) return null;
    for (var i = 0; i < _magic.length; i++) {
      if (bytes[i] != _magic[i]) return null;
    }
    final header = ByteData.sublistView(bytes, 0, headerLength);
    return VideoFrame(
      seq: header.getUint32(4),
      captured: DateTime.fromMillisecondsSinceEpoch((header.getFloat64(8) * 1000).round()),
      jpeg: Uint8List.sublistView(bytes, headerLength),
    );
  }
}

// ── app -> robot ────────────────────────────────────────────────────────────

Map<String, dynamic> clientHello(String token) => {
  'type': 'hello',
  'version': protocolVersion,
  'client': 'orio-app',
  'token': token,
};

/// `name` is one of stop, stay, follow_me, go_to. The app sends intent only —
/// never wheel duties (plan: "the robot decides what the wheels do").
Map<String, dynamic> commandMessage(String id, String name, {String? target}) => {
  'type': 'command',
  'id': id,
  'name': name,
  'target': ?target,
};

/// Ask for camera frames, or stop them. The robot only encodes while someone
/// watches, so the app turns this off whenever the video pane is out of view.
Map<String, dynamic> videoMessage(bool on) => {'type': 'video', 'on': on};

/// Joystick position, x (right +) and y (forward +) in -1..1.
Map<String, dynamic> driveMessage(int seq, double x, double y) => {
  'type': 'drive',
  'seq': seq,
  'x': double.parse(x.toStringAsFixed(3)),
  'y': double.parse(y.toStringAsFixed(3)),
};
