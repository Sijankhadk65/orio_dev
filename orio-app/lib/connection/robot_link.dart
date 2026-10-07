import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:web_socket_channel/web_socket_channel.dart';

/// A bidirectional stream of JSON messages to one robot (real, mock or demo).
abstract class RobotLink {
  /// Decoded messages from the robot. Closes when the link drops.
  Stream<Map<String, dynamic>> get messages;

  /// Binary messages from the robot (camera frames).
  Stream<Uint8List> get binary;

  /// Completes once the link is open, or throws if it can't be opened.
  Future<void> get ready;

  void send(Map<String, dynamic> message);

  /// The WebSocket close code once the link has closed, if there was one.
  int? get closeCode;

  Future<void> close();
}

class WebSocketLink implements RobotLink {
  final WebSocketChannel _channel;
  late final Stream<Map<String, dynamic>> _messages;
  late final Stream<Uint8List> _binary;

  WebSocketLink(Uri uri) : _channel = WebSocketChannel.connect(uri) {
    final raw = _channel.stream.asBroadcastStream();
    _binary = raw
        .where((m) => m is List<int>)
        .map((m) => m is Uint8List ? m : Uint8List.fromList(m as List<int>));
    _messages = raw
        .map((raw) => raw is String ? jsonDecode(raw) : null)
        .where((m) => m is Map<String, dynamic>)
        .cast<Map<String, dynamic>>()
        .asBroadcastStream();
  }

  @override
  Stream<Map<String, dynamic>> get messages => _messages;

  @override
  Stream<Uint8List> get binary => _binary;

  @override
  Future<void> get ready => _channel.ready;

  @override
  void send(Map<String, dynamic> message) => _channel.sink.add(jsonEncode(message));

  @override
  int? get closeCode => _channel.closeCode;

  @override
  Future<void> close() => _channel.sink.close();
}
