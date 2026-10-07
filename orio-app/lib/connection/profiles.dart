import 'dart:convert';

import 'package:shared_preferences/shared_preferences.dart';

/// Where to find a robot. The plan says the server address is a setting, never
/// hard-coded — these are only starting points the user edits.
class ServerProfile {
  final String name;
  final String url; // ws://host:port/path; ignored for the demo
  final String token;
  final bool demo;

  const ServerProfile({required this.name, required this.url, this.token = '', this.demo = false});

  ServerProfile copyWith({String? name, String? url, String? token}) =>
      ServerProfile(name: name ?? this.name, url: url ?? this.url, token: token ?? this.token, demo: demo);

  Map<String, dynamic> toJson() => {'name': name, 'url': url, 'token': token, 'demo': demo};

  factory ServerProfile.fromJson(Map<String, dynamic> j) => ServerProfile(
    name: '${j['name']}',
    url: '${j['url']}',
    token: '${j['token'] ?? ''}',
    demo: j['demo'] == true,
  );

  /// Returns null if the URL is fine, else what is wrong with it.
  String? validate() {
    if (demo) return null;
    final uri = Uri.tryParse(url);
    if (uri == null || !(uri.scheme == 'ws' || uri.scheme == 'wss') || uri.host.isEmpty) {
      return 'Use a ws:// or wss:// address, e.g. ws://10.0.2.2:8765/ws';
    }
    return null;
  }
}

const defaultProfiles = [
  ServerProfile(name: 'Demo (no server)', url: '', demo: true),
  // The Android emulator reaches the laptop at 10.0.2.2.
  ServerProfile(name: 'Laptop mock (emulator)', url: 'ws://10.0.2.2:8765/ws'),
  ServerProfile(name: 'Robot on Wi-Fi', url: 'ws://orio.local:8765/ws'),
  ServerProfile(name: 'Robot over Tailscale', url: 'ws://orio:8765/ws'),
];

class ProfileStore {
  static const _key = 'profiles.v1';
  static const _lastKey = 'profiles.last';

  Future<(List<ServerProfile>, int)> load() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final raw = prefs.getString(_key);
      final list = raw == null
          ? List.of(defaultProfiles)
          : [for (final j in jsonDecode(raw) as List) ServerProfile.fromJson(j)];
      final last = (prefs.getInt(_lastKey) ?? 0).clamp(0, list.length - 1);
      return (list, last);
    } catch (_) {
      return (List.of(defaultProfiles), 0);
    }
  }

  Future<void> save(List<ServerProfile> profiles, int selected) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_key, jsonEncode([for (final p in profiles) p.toJson()]));
      await prefs.setInt(_lastKey, selected);
    } catch (_) {
      // Settings are a convenience; failing to save them is not worth a crash.
    }
  }
}
