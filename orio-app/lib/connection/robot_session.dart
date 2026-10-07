import 'dart:async';

import 'package:flutter/foundation.dart';

import '../protocol/messages.dart';
import 'demo_link.dart';
import 'profiles.dart';
import 'robot_link.dart';

enum LinkState {
  connecting,
  online, // hello received, status fresh
  stale, // still connected, but status has stopped arriving
  lost, // dropped; retrying
  refused, // robot turned us away (wrong token or protocol version); no retry
  closed, // user disconnected
}

typedef LinkFactory = RobotLink Function(ServerProfile profile);

RobotLink defaultLinkFactory(ServerProfile p) => p.demo ? DemoLink() : WebSocketLink(Uri.parse(p.url));

/// One connection to one robot, and everything the screens show about it.
///
/// Reconnects on its own after a drop — the robot never depends on the app
/// (safety rule 5), so the app's job after a drop is just to catch up.
class RobotSession extends ChangeNotifier {
  final ServerProfile profile;
  final LinkFactory _linkFactory;

  static const staleAfter = Duration(milliseconds: 1500);
  static const connectTimeout = Duration(seconds: 6);
  static const maxLines = 300;

  RobotSession(this.profile, {LinkFactory linkFactory = defaultLinkFactory})
    : _linkFactory = linkFactory; // ignore: prefer_initializing_formals

  LinkState state = LinkState.connecting;
  String? error;
  Hello? hello;
  Status? status;
  DateTime? lastMessageAt;
  DateTime? lastStatusAt;
  final List<TranscriptLine> transcript = [];
  final List<RobotEvent> events = [];
  int attempt = 0;
  String? _robotError; // the robot's last `error` text, shown if it then closes
  Duration? lastLatency; // round trip of the most recent command

  RobotLink? _link;
  StreamSubscription<Map<String, dynamic>>? _sub;
  Timer? _watchdog;
  Timer? _retry;
  int _commandSeq = 0;
  int _driveSeq = 0;
  final Map<String, (String, DateTime)> _pending = {};
  bool _disposed = false;

  bool has(String feature) => hello?.has(feature) ?? false;
  bool get connected => state == LinkState.online || state == LinkState.stale;

  Future<void> connect() async {
    _retry?.cancel();
    await _teardown();
    _set(LinkState.connecting);
    attempt++;
    _robotError = null;
    final link = _linkFactory(profile);
    _link = link;
    try {
      await link.ready.timeout(connectTimeout);
    } catch (e) {
      if (_link != link) return;
      _onDrop('could not connect: ${_short(e)}');
      return;
    }
    if (_link != link) return;
    _sub = link.messages.listen(
      _onMessage,
      onError: (e) => _onDrop(_short(e)),
      onDone: () => _onDrop('connection closed', closeCode: link.closeCode),
    );
    link.send(clientHello(profile.token));
    _watchdog = Timer.periodic(const Duration(milliseconds: 250), (_) => _checkStale());
  }

  void _onMessage(Map<String, dynamic> m) {
    lastMessageAt = DateTime.now();
    switch (m['type']) {
      case 'hello':
        hello = Hello.fromJson(m);
        if (!hello!.compatible) {
          error = 'robot speaks protocol ${hello!.version}, app speaks $protocolVersion';
          _set(LinkState.refused);
          _teardown();
          return;
        }
        // The robot re-sends its recent conversation on connect.
        transcript.clear();
        lastStatusAt = DateTime.now();
        error = null;
        _set(LinkState.online);
      case 'status':
        status = Status.fromJson(m);
        lastStatusAt = DateTime.now();
        attempt = 0; // only a working link resets the backoff: a refusal also starts with hello
        if (state == LinkState.stale) state = LinkState.online;
      case 'transcript':
        _push(transcript, TranscriptLine.fromJson(m));
      case 'event':
        _push(events, RobotEvent.fromJson(m));
      case 'result':
        final r = CommandResult.fromJson(m);
        final pending = _pending.remove(r.id);
        if (pending != null) lastLatency = DateTime.now().difference(pending.$2);
        _push(
          events,
          RobotEvent(
            time: DateTime.now(),
            kind: r.ok ? 'result' : 'refused',
            text: '${pending?.$1 ?? 'command'}: ${r.text}',
          ),
        );
      case 'error':
        error = _robotError = '${m['text'] ?? 'robot reported an error'}';
        _push(events, RobotEvent(time: DateTime.now(), kind: 'error', text: error!));
    }
    _notify();
  }

  void _push<T>(List<T> list, T item) {
    list.add(item);
    if (list.length > maxLines) list.removeRange(0, list.length - maxLines);
  }

  void _checkStale() {
    if (state != LinkState.online) return;
    final last = lastStatusAt;
    if (last != null && DateTime.now().difference(last) > staleAfter) _set(LinkState.stale);
  }

  /// Close codes the robot uses to refuse a client (docs/app-protocol.md):
  /// 1002 protocol error or version mismatch, 1008 wrong token. Retrying
  /// cannot fix either, so the session stops and says why.
  static const refusalCodes = {1002, 1008};

  void _onDrop(String why, {int? closeCode}) {
    if (state == LinkState.closed || state == LinkState.refused || _disposed) return;
    if (refusalCodes.contains(closeCode)) {
      error = 'Refused by the robot: ${_robotError ?? 'close code $closeCode'}';
      _set(LinkState.refused);
      _teardown();
      return;
    }
    error = why;
    _set(LinkState.lost);
    _teardown();
    final delay = Duration(seconds: [1, 2, 4, 8][attempt.clamp(1, 4) - 1]);
    _retry = Timer(delay, connect);
  }

  /// Sends one of stop, stay, follow_me, go_to. Returns false if not sent.
  bool command(String name, {String? target}) {
    final link = _link;
    if (link == null || !connected) return false;
    final id = 'c${++_commandSeq}';
    _pending[id] = (target == null ? name : '$name $target', DateTime.now());
    link.send(commandMessage(id, name, target: target));
    return true;
  }

  /// Joystick position. The joystick widget calls this ~10 Hz while held and
  /// once with (0, 0) on release; the robot's deadman covers everything else.
  void drive(double x, double y) {
    final link = _link;
    if (link == null || !connected || !has(Feature.drive)) return;
    link.send(driveMessage(++_driveSeq, x, y));
  }

  Future<void> disconnect() async {
    _retry?.cancel();
    _set(LinkState.closed);
    await _teardown();
  }

  Future<void> _teardown() async {
    _watchdog?.cancel();
    _watchdog = null;
    final sub = _sub, link = _link;
    _sub = null;
    _link = null;
    _pending.clear();
    // Start both now: a link's close() must run even if the cancel never settles.
    await Future.wait([?sub?.cancel(), ?link?.close()]);
  }

  void _set(LinkState s) {
    state = s;
    _notify();
  }

  void _notify() {
    if (!_disposed) notifyListeners();
  }

  static String _short(Object e) {
    final s = e.toString();
    return s.length > 120 ? '${s.substring(0, 120)}…' : s;
  }

  @override
  void dispose() {
    _disposed = true;
    _retry?.cancel();
    _teardown();
    super.dispose();
  }
}
