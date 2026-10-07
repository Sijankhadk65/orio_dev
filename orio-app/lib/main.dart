import 'package:flutter/material.dart';

import 'screens/connect_screen.dart';

void main() => runApp(const OrioApp());

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
