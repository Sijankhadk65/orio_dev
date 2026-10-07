import 'dart:async';
import 'dart:convert';

import 'package:web_socket_channel/web_socket_channel.dart';

/// A bidirectional stream of JSON messages to one robot (real, mock or demo).
abstract class RobotLink {
  /// Decoded messages from the robot. Closes when the link drops.
  Stream<Map<String, dynamic>> get messages;

  /// Completes once the link is open, or throws if it can't be opened.
  Future<void> get ready;

  void send(Map<String, dynamic> message);

  Future<void> close();
}

class WebSocketLink implements RobotLink {
  final WebSocketChannel _channel;
  late final Stream<Map<String, dynamic>> _messages;

  WebSocketLink(Uri uri) : _channel = WebSocketChannel.connect(uri) {
    _messages = _channel.stream
        .map((raw) => raw is String ? jsonDecode(raw) : null)
        .where((m) => m is Map<String, dynamic>)
        .cast<Map<String, dynamic>>()
        .asBroadcastStream();
  }

  @override
  Stream<Map<String, dynamic>> get messages => _messages;

  @override
  Future<void> get ready => _channel.ready;

  @override
  void send(Map<String, dynamic> message) => _channel.sink.add(jsonEncode(message));

  @override
  Future<void> close() => _channel.sink.close();
}
