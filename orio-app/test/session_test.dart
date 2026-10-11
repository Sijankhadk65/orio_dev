import 'dart:async';
import 'dart:io';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:orio_app/connection/profiles.dart';
import 'package:orio_app/connection/robot_link.dart';
import 'package:orio_app/connection/robot_session.dart';

class FakeLink implements RobotLink {
  final _in = StreamController<Map<String, dynamic>>.broadcast();
  final _bin = StreamController<Uint8List>.broadcast();
  final sent = <Map<String, dynamic>>[];
  @override
  int? closeCode;

  void push(Map<String, dynamic> m) => _in.add(m);
  void pushBinary(Uint8List b) => _bin.add(b);
  Future<void> drop({int? code}) {
    closeCode = code;
    return _in.close();
  }

  @override
  Stream<Map<String, dynamic>> get messages => _in.stream;
  @override
  Stream<Uint8List> get binary => _bin.stream;
  @override
  Future<void> get ready => Future.value();
  @override
  void send(Map<String, dynamic> m) => sent.add(m);
  @override
  Future<void> close() async {
    if (!_in.isClosed) await _in.close();
  }
}

void main() {
  const profile = ServerProfile(name: 'test', url: 'ws://x:1/ws', token: 'secret');

  Future<(RobotSession, FakeLink)> connected({
    List<String> features = const ['status', 'commands', 'drive'],
  }) async {
    final link = FakeLink();
    final s = RobotSession(profile, linkFactory: (_) => link);
    await s.connect();
    link.push({'type': 'hello', 'version': '0.1', 'robot': 'orio', 'features': features});
    await pumpEventQueue();
    return (s, link);
  }

  test('sends hello with the token, goes online on the robot hello', () async {
    final (s, link) = await connected();
    expect(link.sent.first, containsPair('token', 'secret'));
    expect(s.state, LinkState.online);
    expect(s.hello!.robot, 'orio');
    s.dispose();
  });

  test('refuses a robot with another major version', () async {
    final link = FakeLink();
    final s = RobotSession(profile, linkFactory: (_) => link);
    await s.connect();
    link.push({'type': 'hello', 'version': '2.0', 'features': []});
    await pumpEventQueue();
    expect(s.state, LinkState.refused);
    s.dispose();
  });

  test('commands carry an id and results are matched to them', () async {
    final (s, link) = await connected();
    expect(s.command('go_to', target: 'chair'), isTrue);
    final cmd = link.sent.last;
    expect(cmd['name'], 'go_to');
    link.push({'type': 'result', 'id': cmd['id'], 'ok': true, 'text': 'on my way'});
    await pumpEventQueue();
    expect(s.events.last.text, 'go_to chair: on my way');
    expect(s.lastLatency, isNotNull);
    s.dispose();
  });

  test('drive is not sent when the robot lacks the feature', () async {
    final (s, link) = await connected(features: ['status']);
    final before = link.sent.length;
    s.drive(0.5, 0.5);
    expect(link.sent.length, before);
    s.dispose();
  });

  test('a drop goes to lost and keeps what was already shown', () async {
    final (s, link) = await connected();
    link.push({'type': 'status', 'behaviour': 'idle'});
    await pumpEventQueue();
    await link.drop();
    await pumpEventQueue();
    expect(s.state, LinkState.lost);
    expect(s.status, isNotNull);
    expect(s.command('stop'), isFalse);
    s.dispose();
  });

  test('a wrong token stops the retries and says why', () async {
    var links = 0;
    final link = FakeLink();
    final s = RobotSession(
      profile,
      linkFactory: (_) {
        links++;
        return link;
      },
    );
    await s.connect();
    // What the robot does: its hello first, then the token check.
    link.push({
      'type': 'hello',
      'version': '0.1',
      'features': ['status'],
    });
    link.push({'type': 'error', 'text': 'wrong token'});
    await pumpEventQueue();
    await link.drop(code: 1008);
    await pumpEventQueue();
    expect(s.state, LinkState.refused);
    expect(s.error, contains('wrong token'));
    await Future<void>.delayed(const Duration(milliseconds: 1200));
    expect(links, 1, reason: 'a refusal must not reconnect');
    s.dispose();
  });

  test('a plain drop still retries', () async {
    var links = 0;
    late FakeLink last;
    final s = RobotSession(
      profile,
      linkFactory: (_) {
        links++;
        return last = FakeLink();
      },
    );
    await s.connect();
    await last.drop();
    await pumpEventQueue();
    expect(s.state, LinkState.lost);
    await Future<void>.delayed(const Duration(milliseconds: 1200));
    expect(links, 2);
    s.dispose();
  });

  Uint8List frameBytes(int seq) {
    final header = ByteData(16)
      ..setUint8(0, 0x4F)
      ..setUint8(1, 0x4A)
      ..setUint8(2, 0x50)
      ..setUint8(3, 0x47)
      ..setUint32(4, seq)
      ..setFloat64(8, DateTime.now().millisecondsSinceEpoch / 1000);
    return Uint8List.fromList([...header.buffer.asUint8List(), 0xFF, 0xD8, 0xFF, 0xD9]);
  }

  test('video is requested only while watched, and frames reach the notifier', () async {
    final (s, link) = await connected(features: ['status', 'video']);
    link.pushBinary(frameBytes(1));
    await pumpEventQueue();
    expect(s.video.value, isNull, reason: 'frames nobody asked for are dropped');

    s.watchVideo();
    expect(link.sent.last, {'type': 'video', 'on': true});
    link.pushBinary(frameBytes(2));
    await pumpEventQueue();
    expect(s.video.value!.seq, 2);
    expect(s.lastFrameAt, isNotNull);

    s.watchVideo(); // a second pane, e.g. during a layout change
    s.unwatchVideo();
    expect(link.sent.last, {'type': 'video', 'on': true}, reason: 'one pane is still watching');
    s.unwatchVideo();
    expect(link.sent.last, {'type': 'video', 'on': false});
    expect(s.video.value, isNull);
    s.dispose();
  });

  test('video is not requested from a robot without it', () async {
    final (s, link) = await connected(features: ['status']);
    final before = link.sent.length;
    s.watchVideo();
    expect(link.sent.length, before);
    s.dispose();
  });

  test('watching survives a reconnect', () async {
    late FakeLink last;
    final s = RobotSession(profile, linkFactory: (_) => last = FakeLink());
    await s.connect();
    last.push({
      'type': 'hello',
      'version': '0.1',
      'features': ['video'],
    });
    await pumpEventQueue();
    s.watchVideo();
    await last.drop();
    await Future<void>.delayed(const Duration(milliseconds: 1200)); // first retry is after 1 s
    last.push({
      'type': 'hello',
      'version': '0.1',
      'features': ['video'],
    });
    await pumpEventQueue();
    expect(last.sent.last, {'type': 'video', 'on': true});
    s.dispose();
  });

  group('connect errors read as reasons', () {
    final robot = Uri.parse('ws://orio:8765/ws');

    test('a closed port says the server is not running', () async {
      final free = await ServerSocket.bind(InternetAddress.loopbackIPv4, 0);
      final port = free.port;
      await free.close();
      final link = WebSocketLink(Uri.parse('ws://127.0.0.1:$port/ws'));
      Object? error;
      try {
        await link.ready;
      } catch (e) {
        error = e;
      }
      expect(error, isNotNull);
      expect(
        explainConnectError(error!, Uri.parse('ws://127.0.0.1:$port/ws')),
        '127.0.0.1 is reachable, but nothing is listening on port $port. Is main.py running on the Jetson?',
      );
    });

    test('an unknown Tailscale name asks about Tailscale', () {
      final e = SocketException("Failed host lookup: 'orio'");
      expect(explainConnectError(e, robot), "can't find orio. Is Tailscale switched on on this device?");
    });

    test('a timeout says the robot may be off', () {
      expect(explainConnectError(TimeoutException('x'), robot), startsWith('no answer from orio.'));
    });

    test('a web server on the port is not mistaken for Orio', () {
      final e = Exception(
        "WebSocketException: Connection to 'http://orio:8765/ws#' was not upgraded "
        'to websocket, HTTP status code: 404',
      );
      expect(explainConnectError(e, robot), contains("not as Orio's app server (HTTP 404)"));
    });
  });
}
