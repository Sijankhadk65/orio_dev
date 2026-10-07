import 'dart:async';

import 'package:flutter_test/flutter_test.dart';
import 'package:orio_app/connection/profiles.dart';
import 'package:orio_app/connection/robot_link.dart';
import 'package:orio_app/connection/robot_session.dart';

class FakeLink implements RobotLink {
  final _in = StreamController<Map<String, dynamic>>.broadcast();
  final sent = <Map<String, dynamic>>[];
  @override
  int? closeCode;

  void push(Map<String, dynamic> m) => _in.add(m);
  Future<void> drop({int? code}) {
    closeCode = code;
    return _in.close();
  }

  @override
  Stream<Map<String, dynamic>> get messages => _in.stream;
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
}
