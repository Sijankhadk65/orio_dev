import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'screens/connect_screen.dart';

/// A screen whose short side is at least this wide (in logical pixels) is a tablet.
const tabletShortestSide = 600.0;

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  // Tablets stay in landscape, where the two-pane layout fits. Phones rotate freely:
  // a phone held sideways is too short for the panes.
  final display = PlatformDispatcher.instance.displays.first;
  if (display.size.shortestSide / display.devicePixelRatio >= tabletShortestSide) {
    await SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }
  runApp(const OrioApp());
}

class OrioApp extends StatelessWidget {
  const OrioApp({super.key});

  static const _seed = Color(0xFF1F8A8A);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Orio',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(colorSchemeSeed: _seed, brightness: Brightness.light),
      darkTheme: ThemeData(colorSchemeSeed: _seed, brightness: Brightness.dark),
      home: const ConnectScreen(),
    );
  }
}
