import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:orio_app/protocol/messages.dart';

void main() {
  test('status parses a full message', () {
    final s = Status.fromJson({
      'type': 'status',
      't': 1760000000.5,
      'state': 'LISTENING',
      'behaviour': 'go_to',
      'behaviour_detail': 'approaching chair',
      'wheel_owner': 'go_to',
      'heading_deg': 12,
      'sensors': {
        'camera': {'ok': true, 'detail': '30 fps'},
        'tof': false,
      },
      'sectors': {
        'fov_deg': 94,
        'distance_m': [1.2, null, 0.15],
        'stop_m': 0.2,
        'clear_m': 0.7,
      },
    });
    expect(s.fsmState, 'LISTENING');
    expect(s.headingDeg, 12.0);
    expect(s.sensors['camera']!.ok, isTrue);
    expect(s.sensors['tof']!.ok, isFalse);
    expect(s.sectors!.distancesM, [1.2, null, 0.15]);
  });

  test('status tolerates missing fields', () {
    final s = Status.fromJson({'type': 'status'});
    expect(s.behaviour, 'unknown');
    expect(s.wheelOwner, isNull);
    expect(s.sectors, isNull);
    expect(s.sensors, isEmpty);
  });

  test('hello checks the major version', () {
    expect(Hello.fromJson({'version': '0.9', 'features': []}).compatible, isTrue);
    expect(Hello.fromJson({'version': '1.0', 'features': []}).compatible, isFalse);
    expect(
      Hello.fromJson({
        'version': '0.1',
        'features': ['drive'],
      }).has(Feature.drive),
      isTrue,
    );
  });

  test('transcript kinds', () {
    final t = TranscriptLine.fromJson({
      'kind': 'tool',
      'name': 'go_to',
      'args': {'target': 'chair'},
      'result': 'arrived',
    });
    expect(t.kind, TranscriptKind.tool);
    expect(t.text, 'go_to');
    expect(t.args['target'], 'chair');
  });

  test('drive and command messages', () {
    expect(driveMessage(3, 0.12345, -1), {'type': 'drive', 'seq': 3, 'x': 0.123, 'y': -1.0});
    expect(commandMessage('c1', 'go_to', target: 'person'), {
      'type': 'command',
      'id': 'c1',
      'name': 'go_to',
      'target': 'person',
    });
    expect(commandMessage('c2', 'stop').containsKey('target'), isFalse);
  });

  test('video frames parse, junk does not', () {
    final header = ByteData(16)
      ..setUint32(0, 0x4F4A5047) // "OJPG"
      ..setUint32(4, 4000000000)
      ..setFloat64(8, 1760000000.25);
    final bytes = Uint8List.fromList([...header.buffer.asUint8List(), 0xFF, 0xD8, 1, 2]);
    final f = VideoFrame.parse(bytes)!;
    expect(f.seq, 4000000000);
    expect(f.captured.millisecondsSinceEpoch, 1760000000250);
    expect(f.jpeg, [0xFF, 0xD8, 1, 2]);
    expect(VideoFrame.parse(Uint8List.fromList(List.filled(20, 0))), isNull);
    expect(VideoFrame.parse(Uint8List(16)), isNull);
  });
}
