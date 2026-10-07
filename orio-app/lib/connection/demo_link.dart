import 'dart:async';
import 'dart:math';
import 'dart:typed_data';

import '../protocol/messages.dart';
import 'robot_link.dart';

/// A fake Orio inside the app, speaking the same protocol as the real server.
///
/// It exists so every screen can be built and shown before the Jetson serves
/// anything — and with no laptop mock running either. It is not a simulation:
/// numbers wander plausibly, commands get plausible answers, nothing more.
class DemoLink implements RobotLink {
  final _out = StreamController<Map<String, dynamic>>.broadcast();
  final _rng = Random();
  final List<Timer> _timers = [];

  String _state = 'ASLEEP';
  String _behaviour = 'idle';
  String _detail = '';
  String? _owner;
  String? _goingTo; // the target of a running go_to
  bool _stickLatched = false; // after a stop the stick waits for the centre, as on the robot
  double _heading = 0;
  double _turnRate = 0; // deg per status tick, from the joystick
  final List<double?> _sectors = List.filled(7, 1.5);
  DateTime _lastDrive = DateTime.fromMillisecondsSinceEpoch(0);
  int _script = 0;

  DemoLink() {
    // Let the session subscribe before the robot's hello goes out.
    Timer.run(_start);
  }

  static double _now() => DateTime.now().millisecondsSinceEpoch / 1000;

  void _emit(Map<String, dynamic> m) {
    if (!_out.isClosed) _out.add(m);
  }

  void _start() {
    _emit({
      'type': 'hello',
      'version': protocolVersion,
      'robot': 'orio (demo)',
      // No video: encoding JPEG in Dart isn't worth it for a demo. The laptop
      // mock (tools/mock_server.py) serves a test pattern instead.
      'features': [Feature.status, Feature.transcript, Feature.commands, Feature.drive],
    });
    _timers.add(Timer.periodic(const Duration(milliseconds: 200), (_) => _tick()));
    _timers.add(Timer.periodic(const Duration(seconds: 4), (_) => _converse()));
  }

  void _tick() {
    // Deadman, as the real robot will do it: no drive for 300 ms stops.
    if (_owner == 'app' && DateTime.now().difference(_lastDrive) > const Duration(milliseconds: 300)) {
      _setIdle();
      _event('halted', 'joystick released (deadman)');
    }
    _heading = (_heading + _turnRate + (_rng.nextDouble() - 0.5) * 0.3) % 360;
    for (var i = 0; i < _sectors.length; i++) {
      if (_rng.nextDouble() < 0.03) {
        _sectors[i] = null; // a dropout now and then
      } else {
        final base = _sectors[i] ?? 1.5;
        _sectors[i] = (base + (_rng.nextDouble() - 0.5) * 0.25).clamp(0.12, 3.0);
      }
    }
    _emit({
      'type': 'status',
      't': _now(),
      'state': _state,
      'behaviour': _behaviour,
      'behaviour_detail': _detail,
      'wheel_owner': _owner,
      'heading_deg': _heading,
      'speed_percent': 5,
      'sensors': {
        'camera': {'ok': true, 'detail': 'Gemini 336L · 30 fps'},
        'imu': {'ok': true, 'detail': 'BNO085 · RVC'},
        'tof': {'ok': _rng.nextDouble() > 0.02, 'detail': 'low fan'},
        'drivetrain': {'ok': true, 'detail': 'STM32 link'},
      },
      'sectors': {'fov_deg': 94, 'distance_m': _sectors, 'stop_m': 0.20, 'clear_m': 0.70},
    });
  }

  static const _dialogue = [
    ['wake', 'hey orio'],
    ['said', 'Hmm?'],
    ['heard', 'what can you see'],
    ['tool', 'what_do_you_see'],
    ['said', 'I can see a person and a chair.'],
    ['heard', 'go to the chair'],
    ['said', "Okay, I'll walk over to the chair."],
  ];

  void _converse() {
    if (_owner == 'app') return; // don't talk over someone driving
    final line = _dialogue[_script % _dialogue.length];
    _script++;
    _state = switch (line[0]) {
      'wake' || 'heard' => 'LISTENING',
      'tool' => 'THINKING',
      _ => 'SPEAKING',
    };
    _emit({
      'type': 'transcript',
      't': _now(),
      'kind': line[0],
      'text': line[1],
      if (line[0] == 'tool') ...{'args': {}, 'result': 'person (0.91), chair (0.78)'},
    });
    if (_script % _dialogue.length == 0) {
      _state = 'ASLEEP';
    }
  }

  void _event(String kind, String text) => _emit({'type': 'event', 't': _now(), 'kind': kind, 'text': text});

  void _setIdle() {
    _behaviour = 'idle';
    _detail = '';
    _owner = null;
    _turnRate = 0;
  }

  @override
  Stream<Map<String, dynamic>> get messages => _out.stream;

  @override
  Future<void> get ready => Future.value();

  @override
  Stream<Uint8List> get binary => const Stream.empty();

  @override
  int? get closeCode => null;

  @override
  void send(Map<String, dynamic> m) {
    switch (m['type']) {
      case 'command':
        _command('${m['id']}', '${m['name']}', m['target'] as String?);
      case 'drive':
        _lastDrive = DateTime.now();
        final x = (m['x'] as num?)?.toDouble() ?? 0;
        final y = (m['y'] as num?)?.toDouble() ?? 0;
        if (x == 0 && y == 0) {
          _stickLatched = false;
          if (_owner == 'app') _setIdle();
          return;
        }
        if (_stickLatched) return;
        _owner = 'app';
        _behaviour = 'driving';
        _detail = y.abs() >= x.abs() ? (y > 0 ? 'forward' : 'backward') : (x > 0 ? 'right' : 'left');
        _turnRate = x * 6;
    }
  }

  // The robot's refusals, word for word (UNSUPPORTED_COMMANDS in orio/server.py).
  static const _unsupported = {
    'stay': 'Orio has no stay behaviour — it already stays put whenever nothing is driving it',
    'follow_me': "Orio can't follow anyone yet; use go to person instead",
  };

  void _command(String id, String name, String? target) {
    String reply;
    switch (name) {
      case 'stop':
        // As on the robot, a stop only raises an event when it ends an app go_to.
        if (_owner == 'go_to') _event('halted', 'stopped on the way to the $_goingTo');
        if (_owner == 'app') _stickLatched = true;
        _setIdle();
        reply = 'stopped';
      case 'stay' || 'follow_me':
        _emit({'type': 'result', 'id': id, 'ok': false, 'text': _unsupported[name]});
        return;
      case 'go_to':
        _behaviour = 'go_to';
        _detail = 'approaching the ${target ?? '?'}';
        _goingTo = target;
        _owner = 'go_to';
        reply = 'on my way to the ${target ?? '?'}';
        _timers.add(
          Timer(const Duration(seconds: 5), () {
            if (_behaviour != 'go_to') return;
            _setIdle();
            _event('arrived', 'stopped about a metre from the ${target ?? '?'}');
          }),
        );
      default:
        _emit({'type': 'result', 'id': id, 'ok': false, 'text': 'unknown command $name'});
        return;
    }
    _emit({'type': 'result', 'id': id, 'ok': true, 'text': reply});
  }

  @override
  Future<void> close() async {
    for (final t in _timers) {
      t.cancel();
    }
    await _out.close();
  }
}
