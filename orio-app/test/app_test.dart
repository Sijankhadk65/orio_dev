import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:orio_app/connection/profiles.dart';
import 'package:orio_app/connection/robot_session.dart';
import 'package:orio_app/main.dart';
import 'package:orio_app/screens/home_screen.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  setUp(() => SharedPreferences.setMockInitialValues({}));

  testWidgets('connect screen lists the default servers', (tester) async {
    await tester.pumpWidget(const OrioApp());
    await tester.pumpAndSettle();
    expect(find.text('Demo (no server)'), findsOneWidget);
    expect(find.text('Robot over Tailscale'), findsOneWidget);
  });

  Future<RobotSession> showHome(WidgetTester tester, Size size) async {
    tester.view.physicalSize = size;
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);
    final session = RobotSession(defaultProfiles.first);
    await tester.pumpWidget(MaterialApp(home: HomeScreen(session: session)));
    await session.connect();
    await tester.pump(const Duration(milliseconds: 500));
    return session;
  }

  Future<void> finish(WidgetTester tester, RobotSession session) async {
    await tester.pumpWidget(const SizedBox());
    session.disconnect();
    await tester.pump(const Duration(seconds: 1));
    session.dispose();
  }

  testWidgets('phone layout uses tabs', (tester) async {
    final session = await showHome(tester, const Size(400, 800));
    expect(find.byType(NavigationBar), findsOneWidget);
    expect(find.text('orio (demo)'), findsOneWidget);
    await tester.tap(find.text('Drive'));
    await tester.pump(const Duration(milliseconds: 300));
    expect(find.text('STOP'), findsOneWidget);
    await finish(tester, session);
  });

  testWidgets('tablet layout shows panes side by side', (tester) async {
    final session = await showHome(tester, const Size(1280, 800));
    expect(find.byType(NavigationBar), findsNothing);
    expect(find.text('STOP'), findsOneWidget);
    expect(find.text('Sector map'), findsOneWidget);
    await finish(tester, session);
  });
}
